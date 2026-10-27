#include "hx5_stage_trace.hpp"
#include <cassert>
#include <iostream>
#include <algorithm>
#include <sys/stat.h>
int main(int argc,char ** argv) {
  assert(argc==2 || argc==3);const std::string dir=argv[1];mkdir(dir.c_str(),0700);
  setenv("AI_WORKER_HX5_TRACE_DIR",dir.c_str(),1);
  auto & recorder=hx5trace::recorder();recorder.source(reinterpret_cast<void *>(1),"fake_follower");
  assert(!recorder.active());
  {std::ofstream request(dir+"/capture.request");request << "1234 2\n";}
  auto deadline=hx5trace::now_ns()+2000000000ULL;
  while(!recorder.active() && hx5trace::now_ns()<deadline) {std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  assert(recorder.active());
  std::vector<uint64_t> times;times.reserve(100000);
  const uint8_t bytes[5]={0xff,0xff,0xfd,0,110};
  {
    hx5trace::Scope scope(reinterpret_cast<void *>(1),hx5trace::DRIVER_READ_ENTER,hx5trace::DRIVER_READ_EXIT,17);
    recorder.emit(hx5trace::RAW_POSITION,reinterpret_cast<void *>(1),110,111,640,0,123.0,bytes,5);
  }
  assert(hx5trace::context().source==0 && hx5trace::context().cycle==0);
  // Deliberately saturate the ring: producers must not wait for disk/consumer.
  const int iterations=argc==3?10000:100000;
  for(int i=0;i<iterations;++i) {
    const auto start=hx5trace::now_ns();
    recorder.emit(hx5trace::GOAL_VALUE,reinterpret_cast<void *>(1),140,141,0,0,i);
    times.push_back(hx5trace::now_ns()-start);
    if(argc==3 && i%20==0) {std::this_thread::sleep_for(std::chrono::milliseconds(1));}
  }
  std::remove((dir+"/capture.request").c_str());
  deadline=hx5trace::now_ns()+2000000000ULL;
  while(hx5trace::now_ns()<deadline) {
    std::ifstream status(dir+"/trace_1234.json");if(status.good()) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::ifstream file(dir+"/trace_1234.bin",std::ios::binary);
  char header[32];file.read(header,32);assert(std::string(header,8)=="HX5TRC01");
  hx5trace::Record r;bool raw=false;size_t records=0;
  while(file.read(reinterpret_cast<char *>(&r),sizeof(r))) {
    ++records;
    if(r.kind==hx5trace::RAW_POSITION) {
      assert(r.cycle==17 && r.comm==110 && r.id==111 && r.value==123. && r.size==5);
      assert(std::equal(bytes,bytes+5,r.data));raw=true;
    }
  }
  assert(raw && records>0);
  std::sort(times.begin(),times.end());
  std::cout << "emit median_ns=" << times[times.size()/2] << " p99_ns=" << times[times.size()*99/100]
            << " max_ns=" << times.back() << " records=" << records << std::endl;
}
