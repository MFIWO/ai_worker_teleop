#ifndef HX5_STAGE_TRACE_HPP_
#define HX5_STAGE_TRACE_HPP_
// Diagnostic observation only. No serial operation is introduced here.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

namespace hx5trace {
enum Kind : uint16_t {
  HW_READ_ENTER=1, HW_READ_RESULT=2, HW_STATE=3,
  HW_WRITE_ENTER=4, HW_GOAL=5, HW_WRITE_EXIT=6,
  DRIVER_READ_ENTER=7, DRIVER_READ_EXIT=8,
  DRIVER_WRITE_ENTER=9, DRIVER_WRITE_EXIT=10,
  GOAL_VALUE=11, PACKED_GOAL=12, RAW_POSITION=13, CONVERTED_POSITION=14,
  SERIAL_TX=15, SERIAL_TX_RETURN=16, SERIAL_RX=17
};
struct Record {
  uint64_t mono_ns=0, sequence=0, source=0, cycle=0;
  uint16_t kind=0, comm=0, id=0, aux=0;
  int32_t result=0;
  uint32_t size=0;
  double value=0;
  uint8_t data[48]{};
};
static_assert(sizeof(Record)==104, "Trace format changed");
inline uint64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline bool hand(unsigned comm) {return comm==110 || comm==140;}
struct Context {uint64_t source=0, cycle=0;};
inline Context & context() {static thread_local Context c; return c;}

class Recorder {
  static constexpr size_t capacity=32768;
  std::vector<Record> ring_;
  size_t head_=0, tail_=0;
  std::mutex mutex_;
  std::atomic<bool> active_{false}, stopping_{false};
  std::atomic<uint64_t> sequence_{0}, dropped_{0}, lock_dropped_{0}, max_emit_ns_{0};
  std::string directory_;
  std::thread worker_;
  std::ofstream file_;
  std::string token_;
  uint64_t deadline_=0, written_=0, started_=0;
  bool good_=true;

  bool drain() {
    std::array<Record,256> batch;
    size_t count=0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      while (tail_!=head_ && count<batch.size()) {
        batch[count++]=ring_[tail_]; tail_=(tail_+1)%capacity;
      }
    }
    if (count && file_.is_open()) {
      file_.write(reinterpret_cast<const char *>(batch.data()), count*sizeof(Record));
      good_=good_ && static_cast<bool>(file_); written_+=count;
    }
    return count!=0;
  }
  void finish() {
    if (!file_.is_open()) {return;}
    active_.store(false);
    while (drain()) {}
    file_.flush(); good_=good_ && static_cast<bool>(file_); file_.close();
    std::ofstream report(directory_+"/trace_"+token_+".json");
    report << "{\"schema\":1,\"record_bytes\":" << sizeof(Record)
      << ",\"written\":" << written_ << ",\"dropped\":" << dropped_.load()
      << ",\"lock_dropped\":" << lock_dropped_.load()
      << ",\"max_emit_ns\":" << max_emit_ns_.load()
      << ",\"duration_s\":" << (now_ns()-started_)/1e9
      << ",\"io_ok\":" << (good_?"true":"false") << "}\n";
  }
  void run() {
    std::string seen;
    while (!stopping_.load()) {
      if (file_.is_open()) {
        std::ifstream request(directory_+"/capture.request");
        std::string token; unsigned seconds=0; request >> token >> seconds;
        if (now_ns()>=deadline_ || token!=token_) {finish();}
        else {while (drain()) {}}
      } else {
        std::ifstream request(directory_+"/capture.request");
        std::string token; unsigned seconds=0; request >> token >> seconds;
        if (!token.empty() && token!=seen && token.find_first_not_of("0123456789")==std::string::npos
            && seconds>=1 && seconds<=190) {
          seen=token; token_=token;
          file_.open(directory_+"/trace_"+token_+".bin", std::ios::binary|std::ios::trunc);
          if (file_) {
            const char magic[8]={'H','X','5','T','R','C','0','1'};
            const uint32_t size=sizeof(Record), pid=static_cast<uint32_t>(getpid());
            const uint64_t mono=now_ns();
            const uint64_t wall=std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::system_clock::now().time_since_epoch()).count();
            file_.write(magic,8);file_.write(reinterpret_cast<const char *>(&size),4);
            file_.write(reinterpret_cast<const char *>(&pid),4);
            file_.write(reinterpret_cast<const char *>(&mono),8);file_.write(reinterpret_cast<const char *>(&wall),8);
            written_=0;good_=true;sequence_=0;dropped_=0;lock_dropped_=0;max_emit_ns_=0;
            started_=mono;deadline_=mono+static_cast<uint64_t>(seconds)*1000000000ULL;
            active_.store(true);
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    finish();
  }
public:
  Recorder() {
    const char * dir=std::getenv("AI_WORKER_HX5_TRACE_DIR");
    if (dir && *dir) {directory_=dir;ring_.resize(capacity);worker_=std::thread([this]{run();});}
  }
  ~Recorder() {active_=false;stopping_=true;if(worker_.joinable()) {worker_.join();}}
  bool active() const noexcept {return active_.load(std::memory_order_relaxed);}
  void source(const void * owner,const std::string & name) {
    if (directory_.empty()) {return;}
    // Setup only, before cyclic control; never called by emit().
    std::ofstream f(directory_+"/sources.tsv",std::ios::app);
    f << reinterpret_cast<uint64_t>(owner) << '\t' << name << '\n';
  }
  void emit(Kind kind,const void * owner,unsigned comm=0,unsigned id=0,
    unsigned aux=0,int result=0,double value=0,const uint8_t * data=nullptr,unsigned size=0) noexcept {
    if (!active()) {return;}
    const auto start=now_ns();
    Record r;
    r.mono_ns=start;r.sequence=sequence_.fetch_add(1);r.source=reinterpret_cast<uint64_t>(owner);
    r.cycle=context().source==r.source?context().cycle:0;
    r.kind=kind;r.comm=comm;r.id=id;r.aux=aux;r.result=result;r.value=value;
    r.size=size>48?48:size;if(data && r.size) {std::memcpy(r.data,data,r.size);}
    if (!mutex_.try_lock()) {++lock_dropped_;++dropped_;return;}
    if (!active()) {mutex_.unlock();return;}
    const auto next=(head_+1)%capacity;
    if (next==tail_) {++dropped_;} else {ring_[head_]=r;head_=next;}
    mutex_.unlock();
    const auto elapsed=now_ns()-start;
    auto old=max_emit_ns_.load();
    while (elapsed>old && !max_emit_ns_.compare_exchange_weak(old,elapsed)) {}
  }
  void bytes(Kind kind,const void * owner,const uint8_t * data,int count,int result=0) noexcept {
    if(!active() || count<=0) {return;}
    for(int offset=0;offset<count;offset+=48) {
      emit(kind,owner,0,0,offset,result,0,data+offset,static_cast<unsigned>(std::min(48,count-offset)));
    }
  }
};
inline Recorder & recorder() {static Recorder r;return r;}
struct Scope {
  const void * owner; Kind end; Context saved; uint64_t start;
  Scope(const void * o,Kind begin,Kind finish,uint64_t cycle):owner(o),end(finish),saved(context()),start(now_ns()) {
    context().source=reinterpret_cast<uint64_t>(o);context().cycle=cycle;
    recorder().emit(begin,o);
  }
  ~Scope() {recorder().emit(end,owner,0,0,0,0,(now_ns()-start)/1000.);context()=saved;}
};

template<class Rows>
void joint_values(Kind kind,const void * owner,const Rows & rows) {
  auto & r=recorder();if(!r.active()) {return;}
  for(const auto & row:rows) {
    const std::string lp="finger_l_joint",rp="finger_r_joint";
    const bool left=row.name.compare(0,lp.size(),lp)==0,right=row.name.compare(0,rp.size(),rp)==0;
    if(!left && !right) {continue;}
    unsigned index=0;
    for(size_t i=lp.size();i<row.name.size();++i) {
      if(row.name[i]<'0'||row.name[i]>'9') {index=0;break;}
      index=index*10+static_cast<unsigned>(row.name[i]-'0');
    }
    if(index<1||index>20) {continue;}
    for(size_t j=0;j<row.interface_name_vec.size();++j) {
      if(row.interface_name_vec[j]=="position") {
        const unsigned board=left?140:110,motor=board+1+(index-1)/4*5+(index-1)%4;
        r.emit(kind,owner,board,motor,index,0,*row.value_ptr_vec[j]);break;
      }
    }
  }
}
} // namespace hx5trace
#endif
