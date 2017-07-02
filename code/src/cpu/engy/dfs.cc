//
// Created by lf-z on 3/16/17.
//

#include "dfs.hh"
#include "debug/EnergyMgmt.hh"

DFS::DFS(const Params *p)
    : BaseEnergySM(p), state(DFS::State::STATE_INIT),
      thres_convert(p->thres_convert), thres_poweroff(p->thres_poweroff)
{

}

void DFS::init()
{
    state = DFS::State::STATE_POWEROFF;
    EnergyMsg msg;
    msg.val = 0;
    msg.type = MsgType::POWEROFF;
    broadcastMsg(msg);
}

void DFS::update(double _energy)
{
    EnergyMsg msg;
    msg.val = 0;

    if (state == STATE_INIT) {
        state = STATE_HIGH_FREQUENCY;
    } else if (state == STATE_HIGH_FREQUENCY && _energy < thres_convert) {
        DPRINTF(EnergyMgmt, "State change: high_freq->low_freq state=%d, _energy=%lf, thres_convert=%lf\n", state, _energy, thres_convert);
        state = STATE_LOW_FREQUENCY;
        msg.type = MsgType::HIGH_TO_LOW;
        broadcastMsg(msg);
    } else if (state == STATE_LOW_FREQUENCY && _energy > thres_convert) {
        DPRINTF(EnergyMgmt, "State change: low_freq->high_freq state=%d, _energy=%lf, thres_convert=%lf\n", state, _energy, thres_convert);
        state = STATE_HIGH_FREQUENCY;
        msg.type = MsgType::LOW_TO_HIGH;
        broadcastMsg(msg);
    } else if (state == STATE_LOW_FREQUENCY && _energy < thres_poweroff) {
        DPRINTF(EnergyMgmt, "State change: low_freq->off state=%d, _energy=%lf, thres_poweroff=%lf\n", 
        state, _energy, thres_poweroff);
        state = STATE_POWEROFF;
        msg.type = MsgType::POWEROFF;
        broadcastMsg(msg);
    } else if (state == STATE_POWEROFF && _energy > thres_convert) {
        DPRINTF(EnergyMgmt, "State change: off->high_freq state=%d, _energy=%lf, thres_convert=%lf\n", 
        state, _energy, thres_convert);
        state = STATE_HIGH_FREQUENCY;
        msg.type = MsgType::POWERON;
        broadcastMsg(msg);
    }
}

DFS *
DFSParams::create()
{
    return new DFS(this);
}
