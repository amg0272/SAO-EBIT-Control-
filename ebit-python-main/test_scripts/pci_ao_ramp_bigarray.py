import threading

import nidaqmx
from nidaqmx.system.system import System
import time
import numpy as np
import sys
from nidaqmx.constants import AcquisitionType, WAIT_INFINITELY, TaskMode, Signal

frequency = 200000  # Hz
ramp_duration = .001
ramp_width = .005
wait_time = 5
starting_voltage = 0
high_voltage = 5

num_ramp_steps = int(frequency * ramp_duration)
num_width_steps = int(frequency * ramp_width)

print("Number of ramp steps", num_ramp_steps)
print("Number of width steps", num_width_steps)

ramp_up_array = np.linspace(starting_voltage, high_voltage, num=num_ramp_steps)
ramp_width_array = [high_voltage] * num_width_steps
ramp_down_array = np.linspace(high_voltage, starting_voltage, num=num_ramp_steps)
wait_array = [starting_voltage] * (wait_time * frequency)

full_command = []
full_command.extend(ramp_up_array)
full_command.extend(ramp_width_array)
full_command.extend(ramp_down_array)
full_command.extend(wait_array)

print("length ", len(full_command))
print(sys.getsizeof(full_command) / 1024 / 1024, "MB")

# Ramp Up, hardware timed.
task = nidaqmx.Task("POOP")

# NEVER SELECT ANALOG OUTPUT 0 for this.
task.ao_channels.add_ao_voltage_chan('pci-6733/ao1')

task.timing.cfg_samp_clk_timing(frequency,  # How many samples per second.
                                sample_mode=AcquisitionType.CONTINUOUS,
                                # samps_per_chan=len(full_command)
                                )
task.write(full_command)

counter = 0
flag = False
def callback(task_handle, every_n_samples_event_type, number_of_samples, callback_data):
    global counter
    global flag
    counter += 1
    # print("counterallback count ", counter)
    if counter == 1:
        task.stop()
        task.close()
        flag = True
    # print("attempting to stop")
    # task.stop()
    # task.close()
    return 0


task.register_every_n_samples_transferred_from_buffer_event(len(full_command), callback_method=callback)
task.start()
print("Task started")
print("main thread sleeping")
while not flag:
    time.sleep(3)
print("THE END!")