import nidaqmx
import time

from nidaqmx.constants import AcquisitionType

with nidaqmx.Task() as task:

    # NEVER SELECT ANALOG OUTPUT 0 for this.
    task.ao_channels.add_ao_voltage_chan('pci-6733/ao1')

    # task.timing.cfg_samp_clk_timing(1)

    # print('1 Channel N Samples Write: ')
    # print(task.write([1.1, 2.2, 3.3, 4.4, 5.5, 6.6], auto_start=True))
    # task.stop()

    # print('1 Channel 1 Sample Write: ')
    # for i in range(9):
    #     task.write(i)
    #     print("Printed voltage: " + str(i))
    #     time.sleep(1)
    task.write(-9.5)
    print("Wrote voltage to analog output")
    time.sleep(1)
    task.stop()