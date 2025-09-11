import nidaqmx
import time
import numpy as np

from nidaqmx.constants import AcquisitionType, Edge

# Ramp Up, hardware timed.
with nidaqmx.Task() as task:

    # NEVER SELECT ANALOG OUTPUT 0 for this.
    task.ao_channels.add_ao_voltage_chan('pci-6733/ao1')

    task.timing.cfg_samp_clk_timing(100, # How many samples per second.
                                    sample_mode=AcquisitionType.FINITE,
                                    samps_per_chan=100 # How many total samples, doesn't care about the time.
                                    )

    print(task.write(np.linspace(0, 5, 100), auto_start=True))
    task.wait_until_done()
    task.stop()

# Hold value, software timed.
with nidaqmx.Task() as task:
    # NEVER SELECT ANALOG OUTPUT 0 for this.
    task.ao_channels.add_ao_voltage_chan('pci-6733/ao1')

    print(task.write(8, auto_start=True))
    time.sleep(5)
    task.stop()

# Ramp Down, hardware timed.
with nidaqmx.Task() as task:
    # NEVER SELECT ANALOG OUTPUT 0 for this.
    task.ao_channels.add_ao_voltage_chan('pci-6733/ao1')

    task.timing.cfg_samp_clk_timing(50,  # How many samples per second.
                                    sample_mode=AcquisitionType.FINITE,
                                    samps_per_chan=100  # How many total samples, doesn't care about the time.
                                    )

    print(task.write(np.linspace(5, 0, 100), auto_start=True))
    task.wait_until_done()
    task.stop()
