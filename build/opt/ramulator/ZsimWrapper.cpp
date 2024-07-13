#include "ZsimWrapper.h"
#include "Config.h"
#include "Request.h"
#include "MemoryFactory.h"
#include "Memory.h"
#include "DDR3.h"
#include "DDR4.h"
#include "LPDDR3.h"
#include "LPDDR4.h"
#include "GDDR5.h"
#include "WideIO.h"
#include "WideIO2.h"
#include "HBM.h"
#include "SALP.h"
#include <functional>
#include "Request.h"
// #include "log.h"

#include <iostream>
#include "Statistics.h"

using namespace std;
using namespace ramulator;

// static map<string, function<MemoryBase *(const Config&, int)> > name_to_func = {
//     {"DDR3", &MemoryFactory<DDR3>::create}, {"DDR4", &MemoryFactory<DDR4>::create}
// };

static map<string, function<MemoryBase *(const Config&, int)> > name_to_func = {
    {"DDR3", &MemoryFactory<DDR3>::create}, {"DDR4", &MemoryFactory<DDR4>::create},
    {"LPDDR3", &MemoryFactory<LPDDR3>::create}, {"LPDDR4", &MemoryFactory<LPDDR4>::create},
    {"GDDR5", &MemoryFactory<GDDR5>::create}, 
    {"WideIO", &MemoryFactory<WideIO>::create}, {"WideIO2", &MemoryFactory<WideIO2>::create},
    {"HBM", &MemoryFactory<HBM>::create},
    {"SALP-1", &MemoryFactory<SALP>::create}, {"SALP-2", &MemoryFactory<SALP>::create}, {"SALP-MASA", &MemoryFactory<SALP>::create},
};

ZsimWrapper::ZsimWrapper(Config& configs, string& outDir, string& statsPrefix, uint64_t cpuFreqHz, int cacheline){
  printf("init wrapper");

  const string& std_name = configs["standard"];
  assert(name_to_func.find(std_name) != name_to_func.end() && "unrecognized standard name");
  mem = name_to_func[std_name](configs, cacheline);

  cpu_ns = (1000000000.0/cpuFreqHz);
  tCK = mem->clk_ns();

  stats_name = outDir + "/";
  stats_name = stats_name + statsPrefix;
  stats_name = stats_name + "ramulator.stats";
  Stats::statlist.output(stats_name);
}

ZsimWrapper::~ZsimWrapper() {
    delete mem;
}

void ZsimWrapper::tick(){
  mem->tick();
}

double ZsimWrapper::GetTCK(){
  printf("GetTck:%f",tCK);
  // info("GetTck:%f",tCK);
  return tCK;
}

bool ZsimWrapper::send(Request req){
  return mem->send(req);
}

void ZsimWrapper::printall()
{
    Stats::statlist.output(stats_name);
    Stats::statlist.printall();
}

void ZsimWrapper::finish(void){
  mem->finish();
}
// #endif  /*__ZSIM_WRAPPER_H*/
