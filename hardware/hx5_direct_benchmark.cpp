// Actual idle-port workload benchmark. No goal/profile/torque writes exist.
// --led-io exercises the same unicast TX-only path using unchanged LED values.
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_sdk/port_handler_linux.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

class TracePort : public dynamixel::PortHandlerLinux {
public:
  explicit TracePort(const char * path):dynamixel::PortHandlerLinux(path) {}
  int writePort(uint8_t * data,int count) override {
    if (std::getenv("HX5_TRACE")) {std::fprintf(stderr,"TX ");for(int i=0;i<count;++i) {std::fprintf(stderr,"%02x ",data[i]);}std::fprintf(stderr,"\n");}
    return dynamixel::PortHandlerLinux::writePort(data,count);
  }
  int readPort(uint8_t * data,int count) override {
    const int got=dynamixel::PortHandlerLinux::readPort(data,count);
    if (got>0 && std::getenv("HX5_TRACE")) {std::fprintf(stderr,"RX ");for(int i=0;i<got;++i) {std::fprintf(stderr,"%02x ",data[i]);}std::fprintf(stderr,"\n");}
    return got;
  }
};

int main(int argc, char ** argv)
{
  if (argc < 2 || argc > 3) {return 2;}
  const bool led = argc == 3 && std::string(argv[2]) == "--led-io";
  auto port = std::unique_ptr<dynamixel::PortHandler>(new TracePort(argv[1]));
  auto packet = dynamixel::PacketHandler::getPacketHandler(2.0);
  if (!port->openPort() || !port->setBaudRate(4000000)) {return 3;}
  std::this_thread::sleep_for(std::chrono::seconds(1));
  bool running = false;
  auto read = [&](int id, int address, int size) {
    std::vector<uint8_t> data(size);uint8_t error = 0;
    const auto timeout = std::chrono::steady_clock::now()+std::chrono::seconds(2);
    int comm = 0;
    do {
      comm = packet->readTxRx(port.get(), id, address, size, data.data(), &error);
      if ((comm == 0 && !(error & 0x7f)) || running) {break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    } while (std::chrono::steady_clock::now() < timeout);
    if (comm || (error & 0x7f)) {throw std::runtime_error("read ID"+std::to_string(id)+" address "+std::to_string(address)+" error "+std::to_string(comm));}
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    return data;
  };
  try {
    std::vector<int> motors, sensors;
    for (int board : {110, 140}) {
      const auto model = read(board, 0, 2);
      const auto sync = read(board, 70, 1)[0];
      bool sync_safe = sync == 0;
      if (sync == 1 && std::getenv("HX5_PROBE_TABLESYNC")) {
        sync_safe = true;
        for (int f = 0; f < 5; ++f) for (int j = 0; j < 6; ++j) {
          const auto size = read(board, 1066+198*f+2*j, 2);
          if (size[0] || size[1]) {sync_safe = false;}
        }
      }
      if ((model[0]+256*model[1] != 260 && model[0]+256*model[1] != 261) ||
        !sync_safe || read(board, 33, 1)[0] != 0)
      {throw std::runtime_error("idle Direct/TableSync OFF boards required");}
      for (int finger = 0; finger < 5; ++finger) {
        for (int j = 1; j <= 4; ++j) {
          const int id = board+5*finger+j;
          const auto model = read(id, 0, 2);
          if (model[0]+256*model[1] != 1710 || read(id, 64, 1)[0] != 0)
          {throw std::runtime_error("XM335 with torque OFF required");}
          motors.push_back(id);
        }
        sensors.push_back(board+5*finger+5);
      }
    }
    std::vector<uint8_t> leds;
    for (const auto id : motors) {leds.push_back(read(id, 65, 1)[0]);}
    dynamixel::GroupFastBulkRead arms(port.get(), packet);
    for (const auto id : {1,2,3,4,5,6,7,31,32,33,34,35,36,37,81,61,62,101,103,105}) {
      const auto address = (id == 61 || id == 62) ? 634 : 698;
      const auto size = (id == 1 || id == 61) ? 13 : 11;
      if (!arms.addParam(id, address, size)) {throw std::runtime_error("arm handler");}
    }
    // Match driver startup: complete sample before any streamed output.
    for (int id : motors) {read(id, 224, 6);}
    for (int id : sensors) {read(id, 72, 9);}
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();auto deadline = start;
    size_t ri = 0, wi = 0, si = 0, missed = 0;
    std::vector<double> cycles, periods;
    auto previous = start;
    running = true;
    for (int step = 0; step < 600; ++step) {
      const auto tick = Clock::now();
      if (step) {periods.push_back(std::chrono::duration<double, std::milli>(tick-previous).count());}
      previous = tick;
      int comm = arms.txPacket();
      if (comm == 0) {port->setPacketTimeout(5.0);comm = arms.rxPacket();}
      if (comm) {throw std::runtime_error("arm read error "+std::to_string(comm));}
      std::this_thread::sleep_for(std::chrono::microseconds(500));
      for (int j = 0; j < 4; ++j) {read(motors[ri], 224, 6);ri = (ri+1)%40;}
      read(sensors[si], 72, 9);si = (si+1)%10;
      if (led) {
        for (int j = 0; j < 8; ++j) {
          uint8_t data = leds[wi];
          if (packet->writeTxOnly(port.get(), motors[wi], 65, 1, &data))
          {throw std::runtime_error("LED tx failed");}
          std::this_thread::sleep_for(std::chrono::microseconds(200));
          wi = (wi+1)%40;
        }
      }
      const auto end = Clock::now();
      cycles.push_back(std::chrono::duration<double, std::milli>(end-tick).count());
      deadline += std::chrono::milliseconds(10);
      if (end > deadline) {++missed;deadline = end;}
      std::this_thread::sleep_until(deadline);
    }
    const double seconds = std::chrono::duration<double>(Clock::now()-start).count();
    running = false;
    for (size_t i = 0; i < motors.size(); ++i) {
      if (read(motors[i], 64, 1)[0] != 0 || read(motors[i], 65, 1)[0] != leds[i])
      {throw std::runtime_error("torque/LED postcondition failed");}
    }
    std::sort(cycles.begin(), cycles.end());std::sort(periods.begin(), periods.end());
    std::printf("{\"loops\":600,\"arm_hz\":%.3f,\"work_ms_p50\":%.3f,\"work_ms_p95\":%.3f,\"work_ms_max\":%.3f,\"period_ms_p95\":%.3f,\"overruns\":%zu,\"led_io\":%s,\"torques_off\":true,\"leds_unchanged\":true}\n",
      600/seconds, cycles[300], cycles[570], cycles.back(), periods[569], missed, led ? "true" : "false");
  } catch (const std::exception & e) {
    std::fprintf(stderr, "%s\n", e.what());port->closePort();return 1;
  }
  port->closePort();return 0;
}
