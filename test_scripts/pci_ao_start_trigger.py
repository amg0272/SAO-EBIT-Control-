import nidaqmx
import time

from nidaqmx.constants import AcquisitionType

import nidaqmx
from nidaqmx.types import CtrTime
from nidaqmx.constants import AcquisitionType, Edge
from nidaqmx.constants import LineGrouping
import numpy as np
import time

#setup initial clock parameter
f = 1000 #desired clock frequency
period = 1/f # period
t = 1  # desired total cylce time
Nosamps = int(t*f) # number of samples needed to get desired time
Trig_delay =0.02 #s


print('the period (s) is:',period,', the frequency (Hz) is',f,', cycle time (s) is:',t,
      ', number of samples is:', Nosamps, ', delay is', Trig_delay, ', extra samples in trigger',Trig_delay/period )
#setup a clock to use with DO channels
#create task and add counter output channel (C0) to it
with nidaqmx.Task() as task:
    task.co_channels.add_co_pulse_chan_freq("pci-6733/ctr0", freq =f,duty_cycle=0.5)
    #Set the acquisition mode
    task.timing.cfg_implicit_timing(
    sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS,
    samps_per_chan=Nosamps) #setup the timing

    task.start()


    #setup task and add the DO channels
    with nidaqmx.Task() as task2:
        task2.do_channels.add_do_chan('pci-6733/port0/line1')
        task2.do_channels.add_do_chan('pci-6733/port0/line2')


        #setup clock/timing
        task2.timing.cfg_samp_clk_timing(f,  # How many samples per second.
             source="/pci-6733/Ctr0Out",active_edge= Edge.RISING, sample_mode=AcquisitionType.FINITE,
             # samps_per_chan=Nosamps+int(Trig_delay/period) # How many total samples, doesn't care about the time.
             samps_per_chan=Nosamps
            )

        #user inputs to specifiy when triggers are sent (may change)
        starttrig = Trig_delay #s, start reference.by default is two ticks delayed from the timebase
        starttrigdur = period
        trig1 = starttrig +.58 #s, time you would like to send a trigger to the voltage ramp
        trig1dur = .08 #s, duration of trigger pulse

        # initially set all samples to false
        #Note: each samples = 1 period
        w, h = Nosamps+int(Trig_delay/period), 2
        DOdata = np.array([[False for x in range(w)] for y in range(h)])

        # set DO start ref trigger
        trig0start = int(starttrig// period) - 1
        trig0nosamples = int(starttrigdur // period)
        trig0sampend = trig0nosamples + trig0start
        DOdata[0][trig0start] = True
        #DOdata[0][trig0start+5] = True
        #trig0sampstart = int(trig1 // period) - 1
        #trig0sampend = trig0nosamples + trig0sampstart
        #DOdata[0][trig0sampstart:trig0sampend] = True
        # DOdata[0][Nosamps+int(Trig_delay/period)-3]= True # does weird things if its at the very end of the cycle
        DOdata[0][Nosamps - 2] = True
        DOdata[0][Nosamps - 1] = False

        #set DO to true at the time and duration give above for trigger 1
        trig1sampstart = int(trig1//period)-1
        trig1nosamples  = int(trig1dur//period)
        trig1sampend = trig1nosamples+trig1sampstart
        DOdata[1][trig1sampstart:trig1sampend] = True


        print('trigger 1 sent at', trig1-starttrig,'s and held for',trig1dur)
        print(task2.write(DOdata, auto_start=True))

        with nidaqmx.Task() as task3:
            # NEVER SELECT ANALOG OUTPUT 0 for this.
            task3.ao_channels.add_ao_voltage_chan("pci-6733/ao1")
            task3.ao_channels.add_ao_voltage_chan("pci-6733/ao2")
            # hmm can trigger, but it can't register a trigger signal before ~0.02. Also, even on rising edge triggers at very end of pulse
            task3.triggers.start_trigger.cfg_dig_edge_start_trig("/pci-6733/PFI0",trigger_edge=Edge.RISING)
            task3.timing.cfg_samp_clk_timing(f,  # How many samples per second.
                source="/pci-6733/Ctr0Out",active_edge= Edge.RISING,
                                            sample_mode=AcquisitionType.FINITE,
                samps_per_chan=Nosamps)

            w, h = Nosamps, 2
            ramp = np.array([[0 for x in range(w)] for y in range(h)])
            ramp[0][0]= 1
            ramp[0][1] =1
            ramp[0][2] =2
            ramp[0][3]=3
            ramp[0][4]=4
            ramp[0][5:10]=5
            ramp[0][10] = 4
            ramp[0][11] = 3
            ramp[0][12] = 2
            ramp[0][13] = 1
            ramp[0][14] = 0
            ramp[0][Nosamps-3]=2
            ramp[0][Nosamps-1]=0

            ramp[1][50] = 1
            ramp[1][51] = 1
            ramp[1][52] = 2
            ramp[1][53] = 3
            ramp[1][54] = 4
            ramp[1][55:60] = 5
            ramp[1][60] = 4
            ramp[1][61] = 3
            ramp[1][62] = 2
            ramp[1][63] = 1
            ramp[1][64] = 0
            ramp[1][Nosamps - 3] = 2
            ramp[1][Nosamps - 1] = 0
            #print(ramp)

            #ramp[5] = 0
            #print(ramp)
            print(task3.write(ramp, auto_start=True))

            task3.wait_until_done()
            task3.stop()
    #
    task.stop()




    ###########################



