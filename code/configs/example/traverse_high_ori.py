# auto script for traversing high freq
# created at: 2017.6.30 22:00
# Last edit at: 2017.7.1 16:35
# usage: python traverse_high_ori.py

import os
import commands
basics = 'build/ARM/gem5.debug --debug-flags=EnergyMgmt configs/example/auto_script.py --cmd=queens_arm --thres-poweroff=10000 --thres-convert=20000 --energy-time-unit=10us'

# --energy-prof=profile/energy_prof --high-freq=1.0e6 --low-freq=0.5e6
def generate_param(engy_prof_filename, high_freq, low_freq, result_filename):
    return ' --energy-prof=' + engy_prof_filename + \
           ' --high-freq=' + str(high_freq) + \
           ' --low-freq=' + str(low_freq) + \
           ' --result-file=' + result_filename

foldername = 'profile/traverse_high'
duty_ratio = 0.5 # = 1 means an all-high wave
low = 0
traverse_list = [0.5,   1. ,   1.5,   2. ,   2.5,   3. , 3.5,   4. , 4.5,   5. ,   5.5,   6. ,   6.5,   7. ,   7.5,   8. , 8.5, 9. ,   9.5,  10. ]

fp = open('traverse_high_result_ori_time_count.txt','w')

for high in traverse_list: # np.linspace(0, 10, 21):
    if high == 0:
        continue
    engy_prof_filename = os.path.join(foldername, str(high)) + '.txt'
    result_filename = 'traverse_high_result_ori.txt'
    print 'to exec:', basics + ' ' + generate_param(engy_prof_filename, 1000000, 1000000, result_filename)

    status, output = commands.getstatusoutput(basics + ' ' + generate_param(engy_prof_filename, 1000000, 1000000, result_filename))
    on_off_count = 0
    freq_switch_count = 0
    for line in output.split('\n'):
        line = line.strip()
        if ('off->' in line) or ('->off' in line):
            on_off_count += 1
        if ('high_freq->low_freq' in line) or ('low_freq->high_freq' in line):
            freq_switch_count += 1
    print >> fp, engy_prof_filename, on_off_count, freq_switch_count
