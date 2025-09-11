import nidaqmx
import time

from nidaqmx.constants import AcquisitionType, TerminalConfiguration

with nidaqmx.Task() as task:
    task.ai_channels.add_ai_voltage_chan('pcie-6361/ai0', min_val=-10.0, max_val=10.0, terminal_config=TerminalConfiguration.RSE)

    task.timing.cfg_samp_clk_timing(1, sample_mode=AcquisitionType.CONTINUOUS)

    print('1 Channel 1 Sample Read: ')

    print(task.read(1, timeout=20))

    task.stop()