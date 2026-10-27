// Included inside dynamixel_hardware_interface by the reviewed overlay builder.
// Same serial owner/control loop. No worker, FIFO or second port connection.
bool Dynamixel::HX5DirectVirtual(uint8_t comm, uint8_t id) const
{
  return hx5_direct_enabled_ && comm != id && hx5_direct_boards_.count(comm);
}

DxlError Dynamixel::AddHX5Direct(
  uint8_t comm, uint8_t id, const std::vector<std::string> & names,
  const std::vector<std::shared_ptr<double>> & pointers, bool writing)
{
  if (names.size() != pointers.size() || names.empty()) {return DxlError::SET_READ_ITEM_FAIL;}
  RWItemList row;
  row.comm_id = comm;
  row.item_name = names;
  row.item_data_ptr_vec = pointers;
  const bool sensor = names.front().find("Present Pressure ") == 0;
  if (writing && (names.size() != 2 ||
    std::set<std::string>(names.begin(), names.end()) != std::set<std::string>{"Goal Current", "Goal Position"}))
  {return DxlError::SET_WRITE_ITEM_FAIL;}
  for (const auto & name : names) {
    uint16_t address = 0;
    uint8_t size = 0;
    if (!dxl_info_.GetDxlControlItem(comm, id, name, address, size)) {
      return DxlError::CANNOT_FIND_CONTROL_ITEM;
    }
    uint16_t direct = 0;
    if (sensor && !writing) {
      const int channel = std::stoi(name.substr(17));
      if (channel < 1 || channel > 9 || size != 1) {return DxlError::SET_READ_ITEM_FAIL;}
      direct = 72 + channel - 1;
    } else {
      const std::map<std::string, uint16_t> addresses = writing ?
        std::map<std::string, uint16_t>{{"Goal Current", 230}, {"Goal Position", 232}} :
        std::map<std::string, uint16_t>{{"Present Current", 224}, {"Present Velocity", 226}, {"Present Position", 228}};
      const auto found = addresses.find(name);
      if (found == addresses.end() || size != 2) {return DxlError::SET_READ_ITEM_FAIL;}
      direct = found->second;
    }
    row.id_arr.push_back(id);
    row.item_addr.push_back(direct);
    row.item_size.push_back(size);
  }
  auto & rows = writing ? hx5_direct_writes_ : (sensor ? hx5_direct_sensors_ : hx5_direct_reads_);
  for (const auto & existing : rows) {
    if (existing.id_arr.front() == id) {return DxlError::SET_READ_ITEM_FAIL;}
  }
  rows.push_back(row);
  return DxlError::OK;
}

DxlError Dynamixel::ReadHX5DirectRow(const RWItemList & row)
{
  const bool sensor = row.item_addr.front() < 100;
  const uint16_t start = sensor ? 72 : 224;
  const uint16_t count = sensor ? 9 : 6;
  uint8_t raw[9] = {};
  uint8_t error = 0;
  const auto ident = row.id_arr.front();
  const int result = packet_handler_->readTxRx(port_handler_, ident, start, count, raw, &error);
  if (result != COMM_SUCCESS || (error & 0x7f)) {
    fprintf(stderr, "[HX5 DIRECT READ] ID%u comm=%d error=%u\n", ident, result, error);
    return DxlError::BULK_READ_FAIL;
  }
  for (size_t i = 0; i < row.item_name.size(); ++i) {
    const auto offset = row.item_addr[i] - start;
    const auto size = row.item_size[i];
    if (offset + size > count) {return DxlError::BULK_READ_FAIL;}
    uint32_t value = raw[offset];
    if (size == 2) {value |= static_cast<uint32_t>(raw[offset + 1]) << 8;}
    bool is_signed = false;
    double unit = 0.;
    if (!dxl_info_.GetDxlUnitValue(row.comm_id, ident, row.item_name[i], unit) ||
      !dxl_info_.GetDxlSignType(row.comm_id, ident, row.item_name[i], is_signed))
    {return DxlError::CANNOT_FIND_CONTROL_ITEM;}
    *row.item_data_ptr_vec[i] = ConvertValueWithUnitInfo(
      row.comm_id, ident, row.item_name[i], value, size, is_signed);
  }
  return DxlError::OK;
}

DxlError Dynamixel::ReadHX5Direct(bool prime)
{
  if (hx5_direct_boards_.size() != 2 || hx5_direct_reads_.size() != 40 ||
    hx5_direct_sensors_.size() != 10 || hx5_direct_writes_.size() != 40)
  {return DxlError::SET_READ_ITEM_FAIL;}
  // Startup samples every motor BEFORE SyncJointCommandWithStates/first goal.
  // Runtime samples four motors + one tactile device per 100 Hz iteration.
  const size_t motors = prime ? hx5_direct_reads_.size() : std::min<size_t>(4, hx5_direct_reads_.size());
  const size_t sensors = prime ? hx5_direct_sensors_.size() : std::min<size_t>(1, hx5_direct_sensors_.size());
  for (size_t i = 0; i < motors; ++i) {
    const auto result = ReadHX5DirectRow(hx5_direct_reads_[hx5_direct_read_cursor_]);
    if (result != DxlError::OK) {return result;}
    hx5_direct_read_cursor_ = (hx5_direct_read_cursor_ + 1) % hx5_direct_reads_.size();
  }
  for (size_t i = 0; i < sensors; ++i) {
    const auto result = ReadHX5DirectRow(hx5_direct_sensors_[hx5_direct_sensor_cursor_]);
    if (result != DxlError::OK) {return result;}
    hx5_direct_sensor_cursor_ = (hx5_direct_sensor_cursor_ + 1) % hx5_direct_sensors_.size();
  }
  hx5_direct_primed_ = true;
  return DxlError::OK;
}

DxlError Dynamixel::WriteHX5Direct()
{
  if (hx5_direct_writes_.empty()) {return DxlError::OK;}
  if (!hx5_direct_primed_) {return DxlError::BULK_WRITE_FAIL;}
  // Unicast passthrough works; broadcast SyncWrite to finger IDs does not.
  // Eight current+position packets per tick refresh all forty motors at 20 Hz.
  // Read current command pointers at transmission; never queue old setpoints.
  for (size_t j = 0; j < std::min<size_t>(8, hx5_direct_writes_.size()); ++j) {
    const auto & row = hx5_direct_writes_[hx5_direct_write_cursor_];
    uint8_t data[4] = {};
    for (size_t i = 0; i < row.item_name.size(); ++i) {
      const double value = *row.item_data_ptr_vec[i];
      if (!std::isfinite(value)) {return DxlError::BULK_WRITE_FAIL;}
      const auto raw = ConvertUnitValueToRawValue(row.comm_id, row.id_arr[i], row.item_name[i], value, 2, true);
      const auto offset = row.item_addr[i] - 230;
      data[offset] = raw & 0xff;
      data[offset + 1] = (raw >> 8) & 0xff;
    }
    const int result = packet_handler_->writeTxOnly(port_handler_, row.id_arr.front(), 230, 4, data);
    if (result != COMM_SUCCESS) {return DxlError::BULK_WRITE_FAIL;}
    // Existing Status Return Level=2 is retained. Allow its short ACK to finish
    // on the wire; the SDK clears it before the next transaction. No ACK wait.
    std::this_thread::sleep_for(std::chrono::microseconds(200));
    hx5_direct_write_cursor_ = (hx5_direct_write_cursor_ + 1) % hx5_direct_writes_.size();
  }
  return DxlError::OK;
}
