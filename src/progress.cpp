#include "progress.h"
#include <atomic>
#include <algorithm>
namespace arnis::progress { static std::atomic<unsigned> floor{0},suppress{0}; void reset_progress_floor(){floor=0;} void set_progress_suppressed(bool v){if(v)++suppress;else{auto n=suppress.load();while(n&&!suppress.compare_exchange_weak(n,n-1));}} bool emits_suppressed(double p,const std::string&m){return suppress.load()>0&&p<100.0&&m.rfind("Error!",0)!=0;} double clamp_progress(double p){if(p<=0)return p;auto n=static_cast<unsigned>(p*100);auto old=floor.load();while(old<n&&!floor.compare_exchange_weak(old,n));return std::max(n,old)/100.0;} }
