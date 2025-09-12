import nidaqmx
from nidaqmx.constants import LineGrouping, AcquisitionType

# Writes a single value to a digital output, using SW timing (whenever you call write).
with nidaqmx.Task() as task:
    task.do_channels.add_do_chan('pcie-6738/port1/line0')

    print('1 Channel 1 Lines 1 Sample Write SW Timed ')
    print(task.write(False))

# Attempts to use HW timing to generate a digital pulse with a 5ms width.
# with nidaqmx.Task() as task:
#     task.do_channels.add_do_chan('pci-6733/port0/line3')
#
#     task.timing.cfg_samp_clk_timing(200,
#                                     source="20MHzTimebase",
#                                     sample_mode=AcquisitionType.FINITE,
#                                     samps_per_chan=3)
#
#     print('1 Channel 1 Lines 3 Sample pulse [0,1,0]: ')
#     print(task.write([False, True, False], auto_start=True, timeout=2.0))
#     task.wait_until_done()
