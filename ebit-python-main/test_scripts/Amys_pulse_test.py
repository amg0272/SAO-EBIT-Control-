import nidaqmx
from nidaqmx.types import CtrTime
from nidaqmx.constants import AcquisitionType, Edge
from nidaqmx.constants import LineGrouping
import numpy as np
import time

#setup initial clock parameter
f = 1000 #desired clock frequency
period = 1/f # period
t = .1  # desired total cylce time
Nosamps = int(t*f) # number of samples needed to get desired time

print('the period (s) is:',period,', the frequency (Hz) is',f,', cycle time (s) is:',t, ', number of samples is:', Nosamps)

#setup a clock to use with DO channels
#create task and add counter output channel (C0) to it
with nidaqmx.Task() as task:
    task.co_channels.add_co_pulse_chan_freq("pci-6733/ctr0", freq =f,duty_cycle=0.5)
    #Set the acquisition mode
    task.timing.cfg_implicit_timing(
    sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS,
    samps_per_chan=Nosamps) #setup the timing

    task.start()
    #task.wait_until_done() #this seems to mess things up, as does taks.stop()

    #setup task and add the DO channels
    with nidaqmx.Task() as task2:
        task2.do_channels.add_do_chan('pci-6733/port0/line0')
        task2.do_channels.add_do_chan('pci-6733/port0/line1')
        task2.do_channels.add_do_chan('pci-6733/port0/line2')

        #setup clock/timing
        task2.timing.cfg_samp_clk_timing(f,  # How many samples per second.
             source="/pci-6733/Ctr0Out",active_edge= Edge.RISING, sample_mode=AcquisitionType.FINITE,
             samps_per_chan=Nosamps  # How many total samples, doesn't care about the time.
                                          )

        #user inputs to specifiy when triggers are sent (may change)
        starttrig = period * 2 #s, start reference.by default is two ticks delayed from the timebase
        starttrigdur = period
        trig1 = starttrig +.002 #s, time you would like to send a trigger
        trig1dur = .002 #s, duration of trigger pulse
        trig2 = starttrig +.005  # s, time of trigger 2
        trig2dur = .005 #s, duration of trigger pulse 2

        # initially set all samples to false
        #Note: each samples = 1 period
        w, h = Nosamps, 3
        DOdata = np.array([[False for x in range(w)] for y in range(h)])

        # set DO start ref trigger
        trig0start = int(starttrig// period) - 1
        trig0nosamples = int(starttrigdur // period)
        trig0sampend = trig0nosamples + trig0start
        DOdata[0][trig0start] = True
        DOdata[0][Nosamps - 2]= True
        DOdata[0][Nosamps - 1] = False

        #set DO to true at the time and duration give above for trigger 1
        trig1sampstart = int(trig1//period)-1
        trig1nosamples  = int(trig1dur//period)
        trig1sampend = trig1nosamples+trig1sampstart
        DOdata[1][trig1sampstart:trig1sampend] = True

        # set DO to true at the time and duration give above for trigger 2
        trig2sampstart = int(trig2 // period)-1
        trig2nosamples = int(trig2dur // period)
        trig2sampend = trig2nosamples + trig2sampstart
        DOdata[2][trig2sampstart:trig2sampend] = True

        print('trigger 1 sent at', trig1-starttrig,'s and held for',trig1dur)
        print('trigger 2 sent at',trig2-starttrig, 's and held for', trig2dur)
        print(task2.write(DOdata, auto_start=True))
        task2.wait_until_done()
    time.sleep(2)
    task.stop()

