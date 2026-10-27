#ifndef HX5_TRACE_PORT_HPP_
#define HX5_TRACE_PORT_HPP_
#include "dynamixel_sdk/port_handler_linux.h"
#include "hx5_stage_trace.hpp"
// Calls the original SDK methods once, returning exactly their result.
class HX5TracePort:public dynamixel::PortHandlerLinux {
  const void * owner_;
public:
  HX5TracePort(const char * path,const void * owner):dynamixel::PortHandlerLinux(path),owner_(owner) {}
  int writePort(uint8_t * data,int count) override {
    const auto start=hx5trace::now_ns();
    hx5trace::recorder().bytes(hx5trace::SERIAL_TX,owner_,data,count);
    const int result=dynamixel::PortHandlerLinux::writePort(data,count);
    hx5trace::recorder().emit(hx5trace::SERIAL_TX_RETURN,owner_,0,0,count,result,(hx5trace::now_ns()-start)/1000.);
    return result;
  }
  int readPort(uint8_t * data,int count) override {
    const int result=dynamixel::PortHandlerLinux::readPort(data,count);
    if(result>0) {hx5trace::recorder().bytes(hx5trace::SERIAL_RX,owner_,data,result,result);}
    return result;
  }
};
#endif
