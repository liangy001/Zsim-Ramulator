#ifndef __RAMULATOR_WRAPPER_H
#define __RAMULATOR_WRAPPER_H

#include <string>

#include "Config.h"

using namespace std;

namespace ramulator
{

class Request;
class MemoryBase;

class RamulatorWrapper
{
public:
    MemoryBase *mem;
    double tCK;

    RamulatorWrapper(const char* config_path, unsigned num_cpus, int cacheline);
    ~RamulatorWrapper();
    void tick();
    bool send(Request req);
    void finish();
    double get_tCK();
};

} /*namespace ramulator*/

#endif /*__RAMULATOR_WRAPPER_H*/
