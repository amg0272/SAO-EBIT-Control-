import nidaqmx
from nidaqmx.constants import (
    LineGrouping, AcquisitionType, Edge)
import time

with nidaqmx.Task() as task:
    task.do_channels.add_do_chan(
        'pci-6733/port0/line1')

    task.timing.cfg_samp_clk_timing(2, # How many samples per second.
                                    source='/pci-6733/ao/SampleClock',
                                    active_edge=Edge.RISING,
                                    sample_mode=AcquisitionType.FINITE,
                                    samps_per_chan=200 # How many total samples, doesn't care about the time.
                                    )

    # python sleep can only go down to 10-13ms.
    print('1 Channel N Lines 1 Sample Unsigned Integer Write: ')
    for i in range(100):
        print(task.write(True, auto_start=False))
        print(task.write(False, auto_start=False))
    task.start()
    task.wait_until_done()
    task.stop()

    # print('1 Channel N Lines N Samples Unsigned Integer Write: ')
    # print(task.write([1, 2, 4, 8], auto_start=True))