import nidaqmx
import time
import numpy as np

from nidaqmx.constants import AcquisitionType, Edge

# Ramp Up, hardware timed.
# with nidaqmx.Task() as ao_task, nidaqmx.Task() as do_task:
#
#     # NEVER SELECT ANALOG OUTPUT 0 for this.
#     ao_task.ao_channels.add_ao_voltage_chan('pci-6733/ao2')
#
#     ao_task.triggers.start_trigger.cfg_dig_edge_start_trig('pci-6733/ao/SampleClock',
#                                                            trigger_edge=Edge.RISING)
#
#     ao_task.timing.cfg_samp_clk_timing(100,
#                                        # source='pci-6733/ao/SampleClock',
#                                        active_edge=Edge.RISING,
#                                        sample_mode=AcquisitionType.FINITE,
#                                        samps_per_chan=1000)
#
#     print(ao_task.write(np.linspace(0, 5, 100), auto_start=True))
#     ao_task.wait_until_done()

with nidaqmx.Task() as do_task:
    do_task.do_channels.add_do_chan('pci-6733/port0/line1')
    # do_task.do_channels.add_do_chan('/pci-6733/PFI5')

    # do_task.triggers.start_trigger.cfg_dig_edge_start_trig("pci-6733/ao/SampleClock", trigger_edge=Edge.RISING)

    do_task.timing.cfg_samp_clk_timing(1,
                                       source="/pci-6733/PFI5",
                                       active_edge=Edge.RISING,
                                       sample_mode=AcquisitionType.FINITE,
                                       samps_per_chan=100)

    print('Alternate True and False until done.')
    # print(do_task.write(True, auto_start=True))
    # # time.sleep(2)
    print(do_task.write(False, auto_start=True))
    # time.sleep(2)
    # print(do_task.write(True, auto_start=True))
    do_task.wait_until_done()
    do_task.stop()

