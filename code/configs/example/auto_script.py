# auto script for simulation
# created at: 2017.6.30 10:05
# Last edit at: 2017.6.30 14:02
# e.g.
# build/ARM/gem5.debug --debug-flags=EnergyMgmt configs/example/auto_script.py --cmd=queens_arm --thres-poweroff=10000 --thres-convert=20000 --energy-prof=profile/energy_prof --energy-time-unit=10us --high-freq=1.0e6 --low-freq=0.5e6 --result-file=config/example/traverse_high_result.txt

import m5
from m5.objects import *

from optparse import OptionParser
parser = OptionParser()

# cmd, binary file to exec
parser.add_option("-c", "--cmd", default="",
                  help="The binary to run in syscall emulation mode.")

# Energy
parser.add_option("--energy-profile", default="",
                  help="Path to energy profile.")
parser.add_option("--energy-time-unit", action="store", type="string",
                  default='10us',
                  help="Energy time unit of energy profile.")
parser.add_option("--thres-convert", type="float", default=20000,
                  help="high threshold of energy state machine")
parser.add_option("--thres-poweroff", type="float", default=10000,
                  help="low threshold of energy state machine")

# high, low freq
parser.add_option("--high-freq", type="int", default=1.0e6,
                  help="high frequency of DFS")
parser.add_option("--low-freq", type="int", default=0.5e6,
                  help="low frequency of DFS")

# result file
parser.add_option("--result-file", type="string", default="",
                  help="File write result to")

(options, args) = parser.parse_args()


# print parsed arg
print ''
print 'Running instance: '
print 'cmd:', options.cmd
print 'energy profile:', options.energy_profile
print 'energy time unit:', options.energy_time_unit
print 'thres convert:', options.thres_convert
print 'thres poweroff:', options.thres_poweroff
print 'high frequency:', options.high_freq
print 'low frequency:', options.low_freq
print ''
print ''

system = System()
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1MHz'
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = 'atomic'
system.mem_ranges = [AddrRange('512MB')]

system.cpu = AtomicSimpleCPU()
# Set energy state machine
system.energy_mgmt = EnergyMgmt(path_energy_profile = options.energy_profile,
                                energy_time_unit = options.energy_time_unit,
                                state_machine = DFS(thres_convert = options.thres_convert,
                                                    thres_poweroff = options.thres_poweroff))
system.cpu.s_energy_port = system.energy_mgmt.m_energy_port
# Set DFS working high, low freq
system.cpu.high_freq = options.high_freq
system.cpu.low_freq = options.low_freq
##

system.membus = SystemXBar()

system.cpu.icache_port = system.membus.slave
system.cpu.dcache_port = system.membus.slave

system.cpu.createInterruptController()

system.mem_ctrl = DDR3_1600_x64()
system.mem_ctrl.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.master

system.system_port = system.membus.slave

# Set exec binary file
process = LiveProcess()
process.cmd = options.cmd # ['queens_arm']
system.cpu.workload = process
system.cpu.createThreads()

root = Root(full_system = False, system = system)
m5.instantiate()

print "Beginning simulation!"
exit_event = m5.simulate()
cur_tick = m5.curTick()
print 'Exiting @ tick %i because %s' % (cur_tick, exit_event.getCause())

# write result to file
energy_profile_file = options.energy_profile
result_file = options.result_file
fp = open(result_file, 'a')
print >> fp, energy_profile_file, float(cur_tick) / 1.0e12
