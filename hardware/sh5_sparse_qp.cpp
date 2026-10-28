// Sparse assembly of the ROBOTIS VR QP with a weak posture cost and hard elbow barrier.
// Original task weights, physical bounds,
// collision distances, CBF/slack penalties and OSQP defaults. Rows outside the
// native collision buffer have both bounds infinite and a zero-optimum slack:
// omitting only those unbounded rows/unused slacks is algebraically equivalent.
// Included only in the separate SH5 build, never in the physical underlay.
bool VRController::getOptJointVel(Eigen::VectorXd & opt_qdot)
{
  using Triplet = Eigen::Triplet<double>;
  const auto distances = robot_data_->getCollisionPairDistances(true, false, false, collision_buffer_);
  if (static_cast<int>(distances.size()) != si_index_.con_sel_col_size) {return false;}
  std::vector<int> active_pairs;
  for (int i = 0; i < static_cast<int>(distances.size()); ++i) {
    if (!std::isfinite(distances[i].distance) || !distances[i].grad.allFinite()) {return false;}
    if (distances[i].distance <= collision_buffer_) {active_pairs.push_back(i);}
  }
  const int pair_count = static_cast<int>(active_pairs.size());
  const int nx = 3 * joint_dof_ + 1 + pair_count;
  const int nbc = nx;
  const int nc = nbc + 2 * joint_dof_ + 1 + pair_count;
  std::vector<Triplet> hessian_entries, constraint_entries;
  Eigen::MatrixXd tracking = Eigen::MatrixXd::Zero(joint_dof_, joint_dof_);
  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(nx);
  for (const auto & [link_name, desired] : link_xdot_desired_) {
    const Eigen::MatrixXd jacobian = robot_data_->getJacobian(link_name);
    auto weight = cyclo_motion_controller::common::Vector6d::Ones().eval();
    auto found = link_w_tracking_.find(link_name);
    if (found != link_w_tracking_.end()) {weight = found->second;}
    tracking.noalias() += 2.0 * jacobian.transpose() * weight.asDiagonal() * jacobian;
    gradient.head(joint_dof_).noalias() -= 2.0 * jacobian.transpose() * weight.asDiagonal() * desired;
  }
  // Weak posture preference removes unconstrained shoulder/wrist swivel drift.
  // Cartesian tasks and all collision constraints retain their original costs.
  const auto & posture_q = robot_data_->getJointPosition();
  const auto posture_names = robot_data_->getJointNames();
  if (static_cast<int>(posture_names.size()) != joint_dof_) {return false;}
  for (int i = 0; i < joint_dof_; ++i) {
    const auto & name = posture_names[i];
    if (name.rfind("arm_l_joint", 0) != 0 && name.rfind("arm_r_joint", 0) != 0) {continue;}
    const bool elbow = name == "arm_l_joint4" || name == "arm_r_joint4";
    const double reference = elbow ? -1.57 : 0.0;
    const double preferred_velocity = std::max(-0.3, std::min(0.3, 0.5*(reference-posture_q[i])));
    const double posture_weight = 0.02;
    tracking(i,i) += 2.0*posture_weight;
    gradient[i] -= 2.0*posture_weight*preferred_velocity;
  }
  tracking.diagonal() += 2.0 * w_damping_;
  for (int column = 0; column < joint_dof_; ++column) {
    for (int row = 0; row <= column; ++row) {
      if (tracking(row, column) != 0.0) {
        hessian_entries.emplace_back(row, column, tracking(row, column));
      }
    }
  }
  gradient.tail(nx - joint_dof_).setConstant(slack_penalty_);
  Eigen::VectorXd lower = Eigen::VectorXd::Constant(nc, -OSQP_INFTY);
  Eigen::VectorXd upper = Eigen::VectorXd::Constant(nc, OSQP_INFTY);
  // Original bound block, including all nonnegative slack variables.
  for (int i = 0; i < nx; ++i) {constraint_entries.emplace_back(i, i, 1.0);}
  lower.head(joint_dof_) = robot_data_->getJointVelocityLimit().first;
  upper.head(joint_dof_) = robot_data_->getJointVelocityLimit().second;
  // Hard one-sided elbow velocity barrier: NO slack can cross the operational
  // extension boundary. Outside it, refuse solving; never auto-recover a robot.
  for (int i = 0; i < joint_dof_; ++i) {
    if (posture_names[i] == "arm_l_joint4" || posture_names[i] == "arm_r_joint4") {
      if (posture_q[i] > -0.45 + 1e-6) {return false;}
      upper[i] = std::min(upper[i], 2.0 * (-0.45-posture_q[i]));
    }
  }
  lower.segment(joint_dof_, nx - joint_dof_).setZero();
  const auto & q = robot_data_->getJointPosition();
  const auto limits = robot_data_->getJointPositionLimit();
  for (int i = 0; i < joint_dof_; ++i) {
    int row = nbc + si_index_.con_q_min_start + i;
    constraint_entries.emplace_back(row, i, 1.0);
    constraint_entries.emplace_back(row, si_index_.slack_q_min_start + i, 1.0);
    lower[row] = -cbf_alpha_ * (q[i] - limits.first[i]);
    row = nbc + si_index_.con_q_max_start + i;
    constraint_entries.emplace_back(row, i, -1.0);
    constraint_entries.emplace_back(row, si_index_.slack_q_max_start + i, 1.0);
    lower[row] = -cbf_alpha_ * (limits.second[i] - q[i]);
  }
  for (int i = 0; i < pair_count; ++i) {
    const auto & result = distances[active_pairs[i]];
    const int row = nbc + si_index_.con_sel_col_start + i;
    for (int j = 0; j < joint_dof_; ++j) {
      if (result.grad[j] != 0.0) {constraint_entries.emplace_back(row, j, result.grad[j]);}
    }
    constraint_entries.emplace_back(row, si_index_.slack_sel_col_start + i, 1.0);
    lower[row] = -cbf_alpha_ * (result.distance - collision_safe_distance_);
  }
  Eigen::SparseMatrix<double> hessian(nx, nx), constraints(nc, nx);
  hessian.setFromTriplets(hessian_entries.begin(), hessian_entries.end());
  constraints.setFromTriplets(constraint_entries.begin(), constraint_entries.end());
  OsqpEigen::Solver solver;
  solver.settings()->setWarmStart(true);
  solver.settings()->getSettings()->verbose = false;
  solver.data()->setNumberOfVariables(nx);
  solver.data()->setNumberOfConstraints(nc);
  if (!solver.data()->setHessianMatrix(hessian) || !solver.data()->setGradient(gradient) ||
      !solver.data()->setLinearConstraintsMatrix(constraints) ||
      !solver.data()->setLowerBound(lower) || !solver.data()->setUpperBound(upper) ||
      !solver.initSolver() || solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError ||
      solver.getStatus() != OsqpEigen::Status::Solved) {
    opt_qdot = Eigen::VectorXd::Zero(joint_dof_);
    return false;
  }
  qp_status_ = solver.getStatus();
  opt_qdot = solver.getSolution().head(joint_dof_);
  return opt_qdot.allFinite();
}
