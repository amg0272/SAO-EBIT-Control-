import nidaqmx
from nidaqmx.constants import AcquisitionType, FrequencyUnits, Level, Edge, TaskMode, SyncType
import numpy as np
import time

steps = 100
duration_sec = .005
start_voltage = -1
end_voltage = 10
with nidaqmx.Task() as ao_task:

    # NEVER SELECT ANALOG OUTPUT 0 for this.
    ao_task.ao_channels.add_ao_voltage_chan('pci-6733/ao1')


    # Compute the samples per second as an integer
    frequency = steps // duration_sec

    ao_task.timing.cfg_samp_clk_timing(frequency,  # How many samples per second.
                                       sample_mode=AcquisitionType.FINITE,
                                       samps_per_chan=steps)  # How many total samples, doesn't care about the time.

    ao_task.triggers.start_trigger.cfg_dig_edge_start_trig('/pci-6733/PFI0')


    ao_task.write(np.linspace(start_voltage, end_voltage, steps), auto_start=True, timeout=20)

    ao_task.wait_until_done()
