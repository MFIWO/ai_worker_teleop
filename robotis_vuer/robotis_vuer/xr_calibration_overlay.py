"""[ADDED 2026-10-14] Read-only calibration geometry for Vuer AR/VR.

Invert the EXISTING calibrated wrist mapping; never invent a new hand zero.
All marker coordinates are WebXR Y-up metres, not ROS base_link coordinates.
This module neither publishes ROS commands nor mutates the session/its hints.
"""
import math

import numpy as np

from robotis_vuer.xr_arm_targets import rigid_pose


SIDES = ('left', 'right')
ROOT_KEY = 'sg2-calibration-overlay'


def controller_pose_for_robot(mapper, side, robot_pose):
    """Inverse of CalibratedWristMapper.target, including scale and SO(3).

    p_c = p_c0 + B.T @ (p_r - p_r0) / scale
    R_c = B.T @ R_r @ R_r0.T @ B @ R_c0
    This returns the tracked controller ORIGIN, not the user's palm centre.
    """
    robot = rigid_pose(robot_pose)
    anchor = mapper.anchors[side]
    original_input = rigid_pose(anchor.input_pose)
    original_robot = rigid_pose(anchor.robot_pose)
    basis = np.asarray(mapper.basis, dtype=float)
    if (basis.shape != (3, 3) or not np.isfinite(basis).all()
            or not np.allclose(basis.T @ basis, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(basis), 1.0, atol=1e-6)):
        raise ValueError('invalid saved calibration basis')
    if not math.isfinite(mapper.scale) or mapper.scale <= 0:
        raise ValueError('invalid saved calibration scale')
    result = np.eye(4)
    result[:3, 3] = original_input[:3, 3] + basis.T @ (
        robot[:3, 3] - original_robot[:3, 3]) / mapper.scale
    result[:3, :3] = (basis.T @ robot[:3, :3]
                      @ original_robot[:3, :3].T @ basis @ original_input[:3, :3])
    return rigid_pose(result)


def panel_position(head):
    """Only the explanatory panel follows the head; target markers never do."""
    if head is None:
        return [0.0, 1.35, -0.65]
    pose = rigid_pose(head)
    forward = pose[:3, 1].copy()  # Existing BODY head +Y forward convention.
    forward[1] = 0.0
    if np.linalg.norm(forward) < .2:
        forward = np.array([0., 0., -1.])
    forward /= np.linalg.norm(forward)
    return (pose[:3, 3] + .65 * forward + [0., -.16, 0.]).tolist()


def overlay_state(session, inputs, measured, phase, fresh, head=None,
                  held=None, fault=False, ramp_remaining=None, reason=''):
    """Build display-only data. Green means pose aligned, NOT authorization.

    On startup the marker represents the held goal; otherwise it represents
    the measured robot wrist. Colour checks BOTH held and measured alignment,
    using the existing session contract (including radial projection).
    """
    state = {'phase': str(phase), 'fresh': bool(fresh), 'fault': bool(fault),
             'panel_position': panel_position(head), 'arms': {}, 'reason': str(reason),
             'ramp_remaining': ramp_remaining, 'calibrated': session is not None}
    if not fresh:
        state['message'] = 'TRACKING / ROBOT FEEDBACK MISSING\nTargets hidden. Do not align to stale markers.'
        return state
    if fault:
        state['message'] = 'CONTROLLER FAULT\nInspect controller log. X+A does not reset faults.'
        return state
    if session is None:
        state['message'] = 'FIRST CALIBRATION\nHold a comfortable pose; release X/A, then X+A.\nWhite dots = tracked controller origins.'
        for side in SIDES:
            state['arms'][side] = {'current': rigid_pose(inputs[side]).tolist(), 'target': None}
        return state

    diagnostics = session.inspect(inputs, measured)
    held_diagnostics = session.inspect(inputs, held) if held else diagnostics
    for side in SIDES:
        current = rigid_pose(inputs[side])
        reference = held[side] if held else measured[side]
        target = controller_pose_for_robot(session.mapper, side, reference)
        d, h = diagnostics[side], held_diagnostics[side]
        matched = all(x['align_position_m'] <= .02 and x['align_rotation_deg'] <= 10.
                      and x['projection_m'] <= 1e-5 for x in (d, h))
        state['arms'][side] = {
            'current': current.tolist(), 'target': target.tolist(),
            'matched': bool(matched), 'robot_error_mm': 1000 * d['align_position_m'],
            'rotation_error_deg': d['align_rotation_deg'],
            'projection_mm': 1000 * d['projection_m'],
            'controller_distance_mm': 1000 * float(np.linalg.norm(target[:3, 3] - current[:3, 3])),
            'tolerance_radius_m': .02 / session.mapper.scale,
        }
    all_matched = all(a['matched'] for a in state['arms'].values())
    if phase in ('SETTLING', 'WARMUP'):
        remaining = '' if ramp_remaining is None else f'\nRamp remaining: {ramp_remaining:.1f} s'
        state['message'] = 'WARMUP - ROBOT HELD' + remaining + '\nKeep dots and axes on targets; do not move yet.'
    elif phase in ('WAIT_START', 'ALIGN_REQUIRED'):
        state['message'] = ('POSES ALIGNED - release X/A, then X+A' if all_matched else
                            'ALIGN BOTH HANDS\nMatch white dots AND coloured axes to targets.')
        state['message'] += '\nPose match only; controller checks still apply.'
    elif phase == 'ACTIVE':
        state['message'] = 'ACTIVE - TELEOP\nY+B stops. Targets now show measured robot correspondence.'
    else:
        state['message'] = str(phase) + '\nUse X+A only when the controller is ready.'
    return state


def _axes(pose, prefix, length, width):
    from vuer.schemas import Line
    matrix = np.asarray(pose)
    origin = matrix[:3, 3]
    return [Line(key=f'{prefix}-{axis}', points=[origin.tolist(),
                 (origin + length * matrix[:3, i]).tolist()],
                 color=color, lineWidth=width)
            for i, (axis, color) in enumerate((('x', '#ff5252'), ('y', '#65e572'), ('z', '#4da6ff')))]


def render_overlay(state):
    """Native 3D elements (not HTML), visible in an immersive AR session.

    No opaque world/background is added. Text is ASCII so no external Korean
    font installation is required. Replace this keyed group with strict UPSERT
    so stale green markers are actually removed, not just left on the scene.
    """
    from vuer.schemas import Billboard, Line, Sphere, Text, group
    text_colour = '#ff7777' if not state['fresh'] or state['fault'] else '#ffffff'
    children = [Billboard(Text(state['message'], key='cal-panel-text', color=text_colour,
                              fontSize=.022, maxWidth=.65, textAlign='center',
                              anchorX='center', anchorY='middle', outlineWidth=.001,
                              outlineColor='#111111'),
                          key='cal-panel', position=state['panel_position'])]
    for side, arm in state['arms'].items():
        short = 'L' if side == 'left' else 'R'
        live = np.asarray(arm['current'])
        colour = '#4de5ff' if side == 'left' else '#ffb347'
        children.append(Sphere(key=f'cal-{side}-live', args=[.006, 12, 8],
                               position=live[:3, 3].tolist(), materialType='basic',
                               material={'color': '#ffffff'}))
        children.extend(_axes(live, f'cal-{side}-live-axis', .045, 2))
        if arm['target'] is None:
            children.append(Billboard(Text(f'{short} LIVE', key=f'cal-{side}-live-text',
                                           fontSize=.018, color=colour, anchorX='center'),
                                      key=f'cal-{side}-label',
                                      position=(live[:3, 3] + [0., .07, 0.]).tolist()))
            continue
        target = np.asarray(arm['target'])
        if arm['matched']:
            colour = '#4dff88'
        children.append(Sphere(key=f'cal-{side}-target', args=[arm['tolerance_radius_m'], 20, 12],
                               position=target[:3, 3].tolist(), materialType='basic',
                               material={'color': colour, 'wireframe': True}))
        children.extend(_axes(target, f'cal-{side}-target-axis', .08, 4))
        # Distinct asymmetric outline makes controller rotation visible.
        local = np.array([[0, 0, .025], [0, 0, -.075], [-.022, 0, -.045],
                          [0, 0, -.075], [.022, 0, -.045]])
        outline = (local @ target[:3, :3].T + target[:3, 3]).tolist()
        children.append(Line(key=f'cal-{side}-outline', points=outline, color=colour, lineWidth=3))
        children.append(Line(key=f'cal-{side}-correction', points=[live[:3, 3].tolist(),
                                target[:3, 3].tolist()], color=colour, lineWidth=2))
        label = (f"{short} TARGET {'MATCH' if arm['matched'] else 'ALIGN'}\n"
                 f"Hand {arm['controller_distance_mm']:.0f}mm | Rot {arm['rotation_error_deg']:.1f}deg\n"
                 f"Robot error {arm['robot_error_mm']:.0f}mm")
        if arm['projection_mm'] > .01:
            label += f"\nCLIPPED {arm['projection_mm']:.1f}mm"
        children.append(Billboard(Text(label, key=f'cal-{side}-text', fontSize=.018,
                                       color=colour, anchorX='center', textAlign='center',
                                       outlineWidth=.001, outlineColor='#111111'),
                                  key=f'cal-{side}-label',
                                  position=(target[:3, 3] + [0., .10, 0.]).tolist()))
    return group(*children, key=ROOT_KEY)
