/** $lic$
 * Copyright (C) 2012-2015 by Massachusetts Institute of Technology
 * Copyright (C) 2010-2013 by The Board of Trustees of Stanford University
 *
 * This file is part of zsim.
 *
 * zsim is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, version 2.
 *
 * If you use this software in your research, we request that you reference
 * the zsim paper ("ZSim: Fast and Accurate Microarchitectural Simulation of
 * Thousand-Core Systems", Sanchez and Kozyrakis, ISCA-40, June 2013) as the
 * source of the simulator in any publications that use this software, and that
 * you send us a citation of your work.
 *
 * zsim is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RAMULATOR_MEM_CTRL_H_
#define RAMULATOR_MEM_CTRL_H_

#include <map>
#include <set>
#include <string>
#include <functional>
#include "g_std/g_string.h"
#include "memory_hierarchy.h"
#include "pad.h"
#include <list>
#include "stats.h"
#include "ramulator/StatType.h"

using namespace std;

namespace ramulator {
    class Request;
    class RamulatorWrapper;
}

class RamulatorAccEvent;
class RamulatorMemory : public MemObject
{ //one DRAMsim3 controller
  private:
      static int gcd(int u, int v) {
      if (v > u) {
        swap(u,v);
      }

      while (v != 0) {
        int r = u % v;
        u = v;
        v = r;
      }

      return u;
    }


    g_string name;
    uint32_t domain;
    uint32_t minLatency;
    uint32_t cpuFreq;
    uint32_t clockDivider;
    double tCK;
    double memFreq;
    unsigned freqRatio;
    unsigned long long tickCounter = 0;
    int cpu_tick, mem_tick, tick_gcd;
    ramulator::RamulatorWrapper* wrapper;

    std::multimap<uint64_t, RamulatorAccEvent*> inflightRequests;

    uint64_t curCycle; //processor cycle, used in callbacks

    // R/W stats
    PAD();
    Counter profReads;
    Counter profWrites;
    Counter profTotalRdLat;
    Counter profTotalWrLat;
  	Counter reissuedAccesses;
    
    Counter profAccesses;
    Counter profCompleteNum;
    Counter profEnqueueNum;
    Counter profInflightRequestsNum;
    Counter profSimulateNum;

    Counter incomingRdWr;
    Counter issuedRdWr;
    PAD();
    int inflight_r = 0;
    int inflight_w = 0;
  public:
    RamulatorMemory(std::string config_file, unsigned num_cpus, unsigned cache_line_size, uint32_t _minLatency, uint32_t _domain, const g_string& _name, unsigned _cpuFreq);
    ~RamulatorMemory();
    void finish();

    const char* getName() {return name.c_str();}
    void initStats(AggregateStat* parentStat);

    // Record accesses
    uint64_t access(MemReq& req);

    // Event-driven simulation (phase 2)
    uint32_t tick(uint64_t cycle);
    void enqueue(RamulatorAccEvent* ev, uint64_t cycle);

  private:
    std::function<void(ramulator::Request&)> read_cb_func;
	  std::function<void(ramulator::Request&)> write_cb_func;
    bool resp_stall;
	  bool req_stall;

    void DRAM_read_return_cb(ramulator::Request&);
    void DRAM_write_return_cb(ramulator::Request&);
	  unsigned m_num_cores;

    vector<RamulatorAccEvent> ramulatorAccEvent;
    set<uint64_t> inflightCheck;
    set<uint64_t> inflightCheck2;
    map<uint64_t, uint64_t> addr_counter;

    std::list<RamulatorAccEvent*> overflowQueue;
};

class SplitAddrMemory : public MemObject
{
  private:
    const g_vector<MemObject *> mems;
    const g_string name;

  public:
    SplitAddrMemory(const g_vector<MemObject *> &_mems, const char *_name) : mems(_mems), name(_name) {}

    uint64_t access(MemReq &req)
    {
        Address addr = req.lineAddr;
        uint32_t mem = addr % mems.size();
        Address ctrlAddr = addr / mems.size();
        req.lineAddr = ctrlAddr;
        uint64_t respCycle = mems[mem]->access(req);
        req.lineAddr = addr;
        return respCycle;
    }

    const char *getName()
    {
        return name.c_str();
    }

    void initStats(AggregateStat *parentStat)
    {
        for (auto mem : mems)
            mem->initStats(parentStat);
    }
};

#endif // RAMULATOR_MEM_CTRL_H_
