import nidaqmx
import time

from nidaqmx.constants import AcquisitionType

with nidaqmx.Task() as task:
    task.ao_channels.add_ao_voltage_chan('pcie-6361/ao0')

    task.timing.cfg_samp_clk_timing(1)

    print('1 Channel 1 Sample Write: ')
    # for i in range(9):
    #     task.write(i)
    #     print("Printed voltage: " + str(i))
    #     time.sleep(1)
    task.write(10)
    time.sleep(5)
    task.write(0)
    time.sleep(1)
    task.stop()