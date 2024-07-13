#ifndef __ZSIM_WRAPPER_H
#define __ZSIM_WRAPPER_H

#include <string>

#include "Config.h"
#include "galloc.h"

using namespace std;

namespace ramulator
{
class Request;
class MemoryBase;
class Config;

class ZsimWrapper : public GlobAlloc
// class ZsimWrapper
{
  private:

    MemoryBase *mem;
    string stats_name;
    double cpu_ns;
    
  public:

    double tCK;

    ZsimWrapper(Config& configs, string& outDir, string& statsPrefix, uint64_t cpuFreqHz, int cacheline);
    ~ZsimWrapper();
    void tick();
    double GetTCK();
    bool send(Request req);
    void printall();
    void finish(void);

};
}

#endif  //__ZSIM_WRAPPER_H



