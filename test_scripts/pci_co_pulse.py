import nidaqmx
from nidaqmx.types import CtrTime


with nidaqmx.Task() as task:
    task.co_channels.add_co_pulse_chan_time("pci-6733/ctr0")

    sample = CtrTime(high_time=0.5, low_time=0.5)

    print('1 Channel 1 Sample Write: ')
    print(task.write(sample))

    # samples = []
    # for i in range(1, 5):
    #     x = i/float(1000)
    #     samples.append(CtrTime(high_time=x, low_time=x))
    #
    # print('1 Channel N Samples Write: ')
    # print(task.write(samples, auto_start=True))