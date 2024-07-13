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

#include "ramulator_mem_ctrl.h"
#include <map>
#include <string>
#include "event_recorder.h"
#include "tick_event.h"
#include "timing_event.h"
#include "zsim.h"
#include "ramulator/RamulatorWrapper.h"
#include "ramulator/Request.h"

using namespace std; // NOLINT(build/namespaces)

class RamulatorAccEvent : public TimingEvent {
  private:
    RamulatorMemory *dram;
    bool write;
    Address addr;
    uint32_t coreId;

  public:
    uint64_t sCycle;

    RamulatorAccEvent(RamulatorMemory * _dram, bool _write, Address _addr, uint32_t _coreId,  int32_t _domain) 
        :  TimingEvent(0, 0, _domain), dram(_dram), write(_write), addr(_addr), coreId(_coreId) {}
    

    bool isWrite() const {
        return write;
    }

    Address getAddr() const {
        return addr;
    }

    uint32_t getCoreId() const {
        return coreId;
    }

    void simulate(uint64_t startCycle) {
      sCycle = startCycle;
      dram->enqueue(this, startCycle);
    }
};

RamulatorMemory::RamulatorMemory(std::string config_file, unsigned num_cpus, unsigned cache_line_size, uint32_t _minLatency, uint32_t _domain,
  const g_string& _name, unsigned _cpuFreq):wrapper(NULL),
	read_cb_func(std::bind(&RamulatorMemory::DRAM_read_return_cb, this, std::placeholders::_1)),
	write_cb_func(std::bind(&RamulatorMemory::DRAM_write_return_cb, this, std::placeholders::_1)),
	resp_stall(false),
	req_stall(false){
  minLatency = _minLatency;
  m_num_cores=num_cpus;
  const char* config_path = config_file.c_str();
  string pathStr = zinfo->outputDir;

  wrapper = new ramulator::RamulatorWrapper(config_path, num_cpus, cache_line_size);

  cpu_tick = int(1000000.0/_cpuFreq);
  mem_tick = wrapper->get_tCK()*1000;
  tick_gcd = gcd(cpu_tick, mem_tick);
  cpu_tick /= tick_gcd;
  mem_tick /= tick_gcd;

  tCK = wrapper->get_tCK();
  cpuFreq = _cpuFreq;
  clockDivider = 0;
  memFreq = (1/(tCK /1000000))/1000;
  info ("[RAMULATOR] Mem frequency %f", memFreq);
  freqRatio = ceil(cpuFreq/memFreq);
  info("[RAMILATOR] CPU/Mem frequency ratio %d", freqRatio);

  Stats_ramulator::statlist.output(pathStr+"/"+"ramulator.stats");
  curCycle = 0;
  domain = _domain;
  TickEvent<RamulatorMemory>* tickEv = new TickEvent<RamulatorMemory>(this, domain);
  tickEv->queue(0);  // start the sim at time 0
  name = _name;
}

RamulatorMemory::~RamulatorMemory(){
  delete wrapper;
}

void RamulatorMemory::initStats(AggregateStat* parentStat) {
  AggregateStat* memStats = new AggregateStat();
  memStats->init(name.c_str(), "Memory controller stats");
  profReads.init("rd", "Completed Read requests"); memStats->append(&profReads);
  profWrites.init("wr", "Completed Write requests"); memStats->append(&profWrites);
  profTotalRdLat.init("rdlat", "Total latency experienced by completed read requests"); memStats->append(&profTotalRdLat);
  profTotalWrLat.init("wrlat", "Total latency experienced by completed write requests"); memStats->append(&profTotalWrLat);
  reissuedAccesses.init("reissuedAccesses", "Number of accesses that were reissued due to full queue"); memStats->append(&reissuedAccesses);

  profAccesses.init("accesses", "Total accesses"); memStats->append(&profAccesses);
  profCompleteNum.init("completed", "Total completed number"); memStats->append(&profCompleteNum);
  profEnqueueNum.init("enqueue", "Total enqueue number"); memStats->append(&profEnqueueNum);
  profInflightRequestsNum.init("InflightRequests", "Total InflightRequests number"); memStats->append(&profInflightRequestsNum);
  profSimulateNum.init("simulateNum", "Total simulate number"); memStats->append(&profSimulateNum);

  incomingRdWr.init("incomingRdWr", "Total incoming RdWr requests"); memStats->append(&incomingRdWr);
  issuedRdWr.init("issuedRdWr","Total issued RdWr requests"); memStats->append(&issuedRdWr);
  parentStat->append(memStats);
}

inline uint64_t RamulatorMemory::access(MemReq& req) {
  switch (req.type) {
      case PUTS:
      case PUTX:
          *req.state = I;
          break;
      case GETS:
          *req.state = req.is(MemReq::NOEXCL)? S : E;
          break;
      case GETX:
          *req.state = M;
          break;
      default: panic("!?");
  }

  if(req.type == PUTS){
    return req.cycle; //must return an absolute value, 0 latency
  } else {

    bool isWrite = (req.type == PUTX);
    uint64_t respCycle = req.cycle + minLatency;

    if (zinfo->eventRecorders[req.srcId]) {
      profAccesses.inc();
      Address addr = req.lineAddr <<lineBits;
      RamulatorAccEvent* memEv = new (zinfo->eventRecorders[req.srcId]) RamulatorAccEvent(this, isWrite, addr, domain,req.srcId);
      memEv->setMinStartCycle(req.cycle);
      TimingRecord tr = {addr, req.cycle, respCycle, req.type, memEv, memEv};
      zinfo->eventRecorders[req.srcId]->pushRecord(tr);
    }
    return respCycle;
  }
}

uint32_t RamulatorMemory::tick(uint64_t cycle) {
  wrapper->tick();
if(overflowQueue.size() > 0){
    RamulatorAccEvent *ev = overflowQueue.front();
    if(ev->isWrite()){
      ramulator::Request req((long)ev->getAddr(), ramulator::Request::Type::WRITE, write_cb_func,ev->getCoreId());
      long addr_tmp = req._addr;

      if(wrapper->send(req)){
        overflowQueue.pop_front();
        inflight_w++;

        inflightRequests.insert(std::pair<uint64_t, RamulatorAccEvent*>((long)ev->getAddr(), ev));
        ev->hold();
      }
    }
    else {
      ramulator::Request req((long)ev->getAddr(), ramulator::Request::Type::READ, read_cb_func ,ev->getCoreId());
      long addr_tmp = req._addr;

      if(wrapper->send(req)){
        overflowQueue.pop_front();
        inflight_r++;

        inflightRequests.insert(std::pair<uint64_t, RamulatorAccEvent*>((long)ev->getAddr(), ev));
        ev->hold();
      }
    }
  }

  curCycle++;
  return 1;
}

void RamulatorMemory::finish(){
  wrapper->finish();
  Stats_ramulator::statlist.printall();
}

void RamulatorMemory::enqueue(RamulatorAccEvent* ev, uint64_t cycle) {

  profEnqueueNum.inc();

  long addr_tmp;

  if(ev->isWrite()){
    ramulator::Request req((long)ev->getAddr(), ramulator::Request::Type::WRITE, write_cb_func,ev->getCoreId());
    addr_tmp = req._addr;

    if(!wrapper->send(req)){
      overflowQueue.push_back(ev);
      reissuedAccesses.inc();
      return;
    }
      inflight_w++;
  }
  else {
    ramulator::Request req((long)ev->getAddr(), ramulator::Request::Type::READ, read_cb_func, ev->getCoreId());
    long addr_tmp = req._addr;

    if(!wrapper->send(req)){
      overflowQueue.push_back(ev);
      reissuedAccesses.inc();
      return;
    }

    inflight_r++;
  }

  inflightRequests.insert(std::pair<uint64_t, RamulatorAccEvent*>((long)ev->getAddr(), ev));
  ev->hold();

    // info("[%s] %s access to %lx added at %ld, %ld inflight reqs", getName(), ev->isWrite()? "Write" : "Read", ev->getAddr(), cycle, inflightRequests.size());
  // ramulator::Request::Type req_type;
  // if(ev->isWrite())
  //   req_type = ramulator::Request::Type::WRITE;
  // else
  //   req_type = ramulator::Request::Type::READ;

  // int core_id = ev->getCoreId();

  // Address ramulator_addr = ev->getAddr();
  
  // futex_lock(&enqueue_lock);
  // ramulator::Request req(ramulator_addr, req_type, callBackFn, core_id);

  // if(zinfo->ramulatorWrapper->send(req)){

  //   // printf("Issuing memacc to Ramulator @ domain %d cycle %d\n",ev->getDomain(), curCycle+1);
  //   inflightRequests.insert(std::pair<uint64_t, RamulatorAccEvent*>(req.addr, ev));
  //   profInflightRequestsNum.inc();
  //   issuedRdWr.inc();
  //   ev->hold();
  // }else{
  //     ev->requeue(cycle+1);
  // }

}

void RamulatorMemory::DRAM_read_return_cb(ramulator::Request& req) {

  profCompleteNum.inc();

  std::multimap<uint64_t, RamulatorAccEvent*>::iterator it = inflightRequests.find(req._addr);
    if(it == inflightRequests.end()){
    info("[RAMULATOR] I didn't request address %ld (%ld)", req._addr, req.addr);
  }

  assert((it != inflightRequests.end()));
  RamulatorAccEvent* ev = it->second;

  uint32_t lat = curCycle + 1 - ev->sCycle;

  if (ev->isWrite()){
    profWrites.inc();
    profTotalWrLat.inc(lat);
    inflight_w--;
  }else{
    profReads.inc();
    profTotalRdLat.inc(lat);
    inflight_r--;
  }

  ev->release();
  ev->done(curCycle+1);

  inflightRequests.erase(it);
}

void RamulatorMemory::DRAM_write_return_cb(ramulator::Request& req){
    //Same as read for now
    DRAM_read_return_cb(req);
}
