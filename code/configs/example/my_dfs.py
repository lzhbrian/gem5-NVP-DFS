# Usage:
#   build/ARM/gem5.debug --debug-flags=EnergyMgmt configs/example/my_dfs.py --cpu-type=atomic

import m5
from m5.objects import *

system = System()
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1MHz'
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = 'atomic'
system.mem_ranges = [AddrRange('512MB')]

system.cpu = AtomicSimpleCPU()


# system.energy_mgmt = EnergyMgmt(path_energy_profile = 'profile/energy_prof',
#               energy_time_unit = '10us')

## problem 5, add by me
# Set energy state machine
system.energy_mgmt = EnergyMgmt(path_energy_profile = 'profile/energy_prof',
                                state_machine = DFS(),
                                energy_time_unit = '10us')
system.cpu.s_energy_port = system.energy_mgmt.m_energy_port
system.energy_mgmt.state_machine.thres_convert = 20000
system.energy_mgmt.state_machine.thres_poweroff = 10000
# Set DFS working high, low freq
system.cpu.high_freq = 1.0e6
system.cpu.low_freq = 0.5e6
##


system.membus = SystemXBar()

system.cpu.icache_port = system.membus.slave
system.cpu.dcache_port = system.membus.slave

system.cpu.createInterruptController()

system.mem_ctrl = DDR3_1600_x64()
system.mem_ctrl.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.master

system.system_port = system.membus.slave

process = LiveProcess()
process.cmd = ['queens_arm']
system.cpu.workload = process
system.cpu.createThreads()

root = Root(full_system = False, system = system)
m5.instantiate()

print "Beginning simulation!"
exit_event = m5.simulate()
print 'Exiting @ tick %i because %s' % (m5.curTick(), exit_event.getCause())
