// Offline tests execute the production helper with a fake packet transport.
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>
constexpr int COMM_SUCCESS = 0;
enum class DxlError {OK, SET_READ_ITEM_FAIL, SET_WRITE_ITEM_FAIL, CANNOT_FIND_CONTROL_ITEM, BULK_READ_FAIL, BULK_WRITE_FAIL};
struct RWItemList {
  uint8_t comm_id;std::vector<uint8_t> id_arr,item_size;std::vector<uint16_t> item_addr;
  std::vector<std::string> item_name;std::vector<std::shared_ptr<double>> item_data_ptr_vec;
};
struct Packet {
  std::vector<int> reads,writes;std::map<int,std::vector<uint8_t>> sent;bool fail_read=false,fail_write=false;
  int readTxRx(void *,int id,int,int count,uint8_t * data,uint8_t * error) {
    if (fail_read) {return -1;}reads.push_back(id);*error=0;
    for (int i=0;i<count;++i) {data[i]=i+1;}return 0;
  }
  int writeTxOnly(void *,int id,int address,int count,uint8_t * data) {
    assert(address==230 && count==4);if (fail_write) {return -1;}
    writes.push_back(id);sent[id]=std::vector<uint8_t>(data,data+count);return 0;
  }
};
struct Info {
  bool GetDxlControlItem(int,int,const std::string & n,uint16_t & a,uint8_t & s) {a=0;s=n.find("Pressure")!=std::string::npos?1:2;return true;}
  bool GetDxlUnitValue(int,int,const std::string &,double & u) {u=1.;return true;}
  bool GetDxlSignType(int,int,const std::string &,bool & s) {s=true;return true;}
};
class Dynamixel {
public:
  bool hx5_direct_enabled_=true,hx5_direct_primed_=false;
  std::set<uint8_t> hx5_direct_boards_{110,140};
  std::vector<RWItemList> hx5_direct_reads_,hx5_direct_sensors_,hx5_direct_writes_;
  size_t hx5_direct_read_cursor_=0,hx5_direct_sensor_cursor_=0,hx5_direct_write_cursor_=0;
  Packet packet;Packet * packet_handler_=&packet;void * port_handler_=nullptr;Info dxl_info_;
  bool HX5DirectVirtual(uint8_t,uint8_t) const;
  DxlError AddHX5Direct(uint8_t,uint8_t,const std::vector<std::string>&,const std::vector<std::shared_ptr<double>>&,bool);
  DxlError ReadHX5DirectRow(const RWItemList&);DxlError ReadHX5Direct(bool);DxlError WriteHX5Direct();
  double ConvertValueWithUnitInfo(int,int,const std::string&,uint32_t v,int,bool) {return v;}
  uint32_t ConvertUnitValueToRawValue(int,int,const std::string&,double v,int,bool) {return static_cast<uint32_t>(v);}
};
#include "hx5_direct_transport.inc"
int main() {
  Dynamixel d;
  for (int b:{110,140}) for(int f=0;f<5;++f) {
    for(int j=1;j<=4;++j) {
      const int id=b+5*f+j;
      std::vector<std::shared_ptr<double>> r(3),w(2);
      for(auto & p:r) {p=std::make_shared<double>(0.);}
      w[0]=std::make_shared<double>(2000+id);w[1]=std::make_shared<double>(300);
      assert(d.AddHX5Direct(b,id,{"Present Position","Present Velocity","Present Current"},r,false)==DxlError::OK);
      assert(d.AddHX5Direct(b,id,{"Goal Position","Goal Current"},w,true)==DxlError::OK);
    }
    std::vector<std::string> n;std::vector<std::shared_ptr<double>> p;
    for(int i=1;i<=9;++i) {n.push_back("Present Pressure "+std::to_string(i));p.push_back(std::make_shared<double>(0.));}
    assert(d.AddHX5Direct(b,b+5*f+5,n,p,false)==DxlError::OK);
  }
  assert(d.WriteHX5Direct()==DxlError::BULK_WRITE_FAIL);assert(d.packet.writes.empty());
  assert(d.ReadHX5Direct(true)==DxlError::OK);assert(d.packet.reads.size()==50);
  assert(*d.hx5_direct_reads_[0].item_data_ptr_vec[0]==0x0605);
  assert(*d.hx5_direct_reads_[0].item_data_ptr_vec[1]==0x0403);
  assert(*d.hx5_direct_reads_[0].item_data_ptr_vec[2]==0x0201);
  assert(d.ReadHX5Direct(false)==DxlError::OK);assert(d.packet.reads.size()==55);
  for(int i=0;i<5;++i) {assert(d.WriteHX5Direct()==DxlError::OK);}
  assert(d.packet.writes.size()==40 && d.packet.sent.size()==40);
  *d.hx5_direct_writes_[0].item_data_ptr_vec[0]=1234.;
  assert(d.WriteHX5Direct()==DxlError::OK);
  assert(d.packet.sent[111][0]==44 && d.packet.sent[111][1]==1);
  assert(d.packet.sent[111][2]==210 && d.packet.sent[111][3]==4);
  d.packet.fail_read=true;assert(d.ReadHX5Direct(false)==DxlError::BULK_READ_FAIL);
  d.packet.fail_write=true;assert(d.WriteHX5Direct()==DxlError::BULK_WRITE_FAIL);
  d.packet.fail_write=false;*d.hx5_direct_writes_[8].item_data_ptr_vec[0]=NAN;
  assert(d.WriteHX5Direct()==DxlError::BULK_WRITE_FAIL);
  assert(!d.HX5DirectVirtual(1,3) && d.HX5DirectVirtual(110,111));
  std::puts("HX5 direct: priming, field order, read/write cadence, latest values, errors OK");
}
