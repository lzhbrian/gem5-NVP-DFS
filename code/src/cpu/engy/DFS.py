from m5.SimObject import SimObject
from BaseEnergySM import BaseEnergySM
from m5.params import *
from m5.proxy import *

class DFS(BaseEnergySM):
    type = 'DFS'
    cxx_header = "engy/dfs.hh"
    thres_convert = Param.Float(Parent.thres_high, "convert threshold of frequency switch")
    thres_poweroff = Param.Float(Parent.thres_low, "poweroff threshold of energy state machine")
