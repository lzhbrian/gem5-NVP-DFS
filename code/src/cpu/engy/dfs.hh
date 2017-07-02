
#ifndef GEM5_DFS_HH
#define GEM5_DFS_HH

#include "state_machine.hh"
#include "params/DFS.hh"

class DFS : public BaseEnergySM
{

public:
    typedef DFSParams Params;
    const Params *params() const
    {
        return reinterpret_cast<const Params *>(_params);
    }
    DFS(const Params *p);
    ~DFS() {}
    virtual void init();
    virtual void update(double _energy);

    enum State {
        STATE_INIT = 0,
        STATE_POWEROFF = 1,
        STATE_HIGH_FREQUENCY = 2,
        STATE_LOW_FREQUENCY = 3
    };

    enum MsgType {
        CONSUMEENERGY = 0,
        POWEROFF = 1,
        POWERON = 2,
        LOW_TO_HIGH = 3,
        HIGH_TO_LOW = 4
    };

protected:
    State state;
    double thres_convert;
    double thres_poweroff;

};
#endif //GEM5_DFS_HH
