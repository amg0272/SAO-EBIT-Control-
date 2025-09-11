import nidaqmx
from nidaqmx.constants import AcquisitionType, FrequencyUnits, Level, Edge, TaskMode, SyncType
import numpy as np


def create_clock_task(frequency=200,  # 5ms
                      units=FrequencyUnits.HZ,
                      duty_cycle=.5,  # Pulse width for high and low are the same at .5 (aka 50%)
                      initial_delay_sec=1.0,
                      pci_card="pci-6733",
                      ctr="1"):

    # Create an NI Task object for the clock.
    task = nidaqmx.Task()

    # Use the counter ctr0 as the source for the timing.
    task.co_channels.add_co_pulse_chan_freq(f"{pci_card}/ctr{str(ctr)}",
                                            units=units,
                                            idle_state=Level.LOW,
                                            initial_delay=initial_delay_sec,
                                            freq=frequency,
                                            duty_cycle=duty_cycle)

    # When the timing is set to continuous the total number of samples is only used for the buffersize.
    task.timing.cfg_implicit_timing(sample_mode=AcquisitionType.CONTINUOUS)

    return task


def create_ramp_task(pci_card="pci-6733",
                     ctr="1",
                     analog_output="1",
                     start_voltage=0,
                     end_voltage=5,
                     steps=100,
                     duration_sec=2,
                     ctr_frequency=200):
    # Ramp Up, hardware timed.
    task = nidaqmx.Task()

    # NEVER SELECT ANALOG OUTPUT 0 for this.
    task.ao_channels.add_ao_voltage_chan(f'{pci_card}/ao{str(analog_output)}')

    # task.triggers.start_trigger.cfg_dig_edge_start_trig("/pci-6733/PFI8", trigger_edge=Edge.RISING)

    # Compute the samples per second as an integer
    frequency = steps // duration_sec

    task.timing.cfg_samp_clk_timing(frequency,  # How many samples per second.
                                    # source=f'/{pci_card}/ctr{str(ctr)}Out',
                                    sample_mode=AcquisitionType.FINITE,
                                    samps_per_chan=steps)  # How many total samples, doesn't care about the time.

    task.write(np.linspace(start_voltage, end_voltage, steps), auto_start=False)
    return task


def create_digital_pulse_task(pci_card="pci-6733",
                              ctr="1",
                              line="0",
                              port="0",
                              num_pulses=1,
                              pulse_width_ms=5,
                              ctr_frequency=2000):
    task = nidaqmx.Task()
    task.do_channels.add_do_chan(f'{pci_card}/port{port}/line{line}')

    frequency = 1000 // pulse_width_ms

    task.timing.cfg_samp_clk_timing(ctr_frequency,
                                    source=f'/{pci_card}/ctr{str(ctr)}Out',
                                    sample_mode=AcquisitionType.FINITE,
                                    samps_per_chan=num_pulses * 2)

    task.write([True, False], auto_start=False, timeout=2.0)
    return task
