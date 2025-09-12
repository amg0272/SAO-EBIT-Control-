import nidaqmx
from nidaqmx.constants import AcquisitionType, FrequencyUnits, Level, Edge, TaskMode, SyncType
import numpy as np
from ebitdaq import *
import time

COUNTER = "0"
ctr_frequency = 20000

clock = create_clock_task(frequency=ctr_frequency, ctr=COUNTER)

ramp = create_ramp_task(ctr=COUNTER,
                        duration_sec=.005,
                        steps=10,
                        ctr_frequency=ctr_frequency)

pulse = create_digital_pulse_task(ctr=COUNTER,
                                  ctr_frequency=2000)

clock.start()
ramp.start()
pulse.start()

ramp.wait_until_done()

clock.stop()
ramp.stop()
pulse.stop()


