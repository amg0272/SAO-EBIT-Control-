from random import uniform
import numpy as np

import nidaqmx
from nidaqmx.constants import TerminalConfiguration, AcquisitionType, LineGrouping
from scipy.signal import butter, lfilter


class EbitVoltageController:
    tasks = {}
    AO_components_in_task = []
    DO_components_in_task = []

    def __init__(self, ebit_data_model):
        self.ebit_data_model = ebit_data_model

    def stop_timing_loop(self):
        if "AO_timing" in self.tasks:
            AO_task = self.tasks.pop("AO_timing")
            AO_task.stop()
            AO_task.close()
            for component_name in self.AO_components_in_task:
                self.set_voltage(component_name, 0)
            self.AO_components_in_task = []

        if "DO_timing" in self.tasks:
            DO_task = self.tasks.pop("DO_timing")
            DO_task.stop()
            DO_task.close()
            for component_name in self.DO_components_in_task:
                self.set_digital(component_name, False)
            self.DO_components_in_task = []


    def start_timing_loop(self, AO_plans, AO_trigger_plans, DO_plans, cycle_time, frequency=200000, enable_LPF=False):
        #This function takes in lists of timing loop "plans":
        #  e.g., AO_plans = [ {Drifttube2, ramp_duration, ramp_width, delay_duration, low_voltage, high_voltage}, {...}]
        #This is called whenever the timing loop was made with the GUI text boxes (i.e., not a timing csv file).
        #The goal is to synchronize all the plans together. To do this, we will use the AO clock for all plans and
        # trigger the DO plans with the AO clock.
        #enable_LPF: True/False on whether to apply a low-pass filter

        # Ensure a voltage ramp isn't in progress since the AO clock is needed for both ramps and timing.
        if hasattr(self, 'ramp_task'):
            print("A voltage ramp is already in progress. Voltage not changed.")
            return 2

        #Define functions for low pass filter
        def butter_lowpass(cutoff, fs, order=1):
            return butter(order, cutoff, fs=fs, btype='low', analog=False)

        def butter_lowpass_filter(data, cutoff, fs, order=1):
            b, a = butter_lowpass(cutoff, fs, order=order)
            y = lfilter(b, a, data)
            return y

        total_samples = int(frequency * cycle_time)

        #Create tasks for each plan
        if len(AO_plans) + len(AO_trigger_plans)>0:
            AO_task = nidaqmx.Task()
            self.tasks["AO_timing"] = AO_task
        if len(DO_plans)>0:
            DO_task = nidaqmx.Task()
            self.tasks["DO_timing"] = DO_task

        # AO_command and DO_command are lists of lists telling each AO/DO port to do
        AO_command = []
        DO_command = []

        for AO_plan in AO_plans:
            component_name  = AO_plan['component_name']
            ramp_duration   = AO_plan['ramp_duration']  #seconds
            ramp_width      = AO_plan['ramp_width']     #seconds
            delay_duration  = AO_plan['delay_duration'] #seconds
            low_voltage     = AO_plan['low_voltage']    #volts
            high_voltage    = AO_plan['high_voltage']   #volts

            #tracks which components have an active task
            self.AO_components_in_task.append(component_name)


            model = self.ebit_data_model.components[component_name]
            # Scale voltages based on their slope/y-intercept values in the CSV.
            low_voltage = low_voltage * model.output_voltage_slope + model.output_voltage_y_intercept
            high_voltage = high_voltage * model.output_voltage_slope + model.output_voltage_y_intercept

            num_ramp_steps = int(frequency * ramp_duration)
            num_width_steps = int(frequency * ramp_width)
            delay_steps = int(delay_duration * frequency)

            ramp_up_array = np.linspace(low_voltage, high_voltage, num=num_ramp_steps).tolist()
            ramp_width_array = [high_voltage] * num_width_steps
            ramp_down_array = np.linspace(high_voltage, low_voltage, num=num_ramp_steps).tolist()
            delay_array = [low_voltage] * delay_steps

            full_command = []
            full_command.extend(delay_array)
            full_command.extend(ramp_up_array)
            full_command.extend(ramp_width_array)
            full_command.extend(ramp_down_array)

            wait_steps = total_samples-len(full_command)  # remaining samples until we fill out the total time
            assert wait_steps >= 0
            wait_array = [low_voltage]*wait_steps
            full_command.extend(wait_array)

            if enable_LPF:
                ### Low-pass filter
                # Filter requirements.
                order = 3
                fs = frequency  # sample rate, Hz
                # slew rate of the trek is 350 V/us
                cutoff = 1000  # fs/2-1  # desired cutoff frequency of the filter, Hz

                T = cycle_time  # seconds
                n = int(T*fs)  # total number of samples
                t = np.linspace(0, T, n, endpoint=False)

                full_command_baseline = full_command[0]
                data = np.array(
                    full_command) - full_command_baseline  # sets the first value to 0. the filter misbehaves otherwise.

                y = butter_lowpass_filter(data, cutoff, fs, order)
                y = y + full_command_baseline

                ### To skip the low-pass filter, comment out the line below.
                full_command = y.tolist()

            AO_task.ao_channels.add_ao_voltage_chan(f'{model.output_card}/{model.output_pin}',
                                                    min_val=model.output_min_volts,
                                                    max_val=model.output_max_volts)

            full_command = np.clip(full_command, model.output_min_volts, model.output_max_volts).tolist()
            AO_command.append(full_command)

        for DO_plan in DO_plans:
            component_name  = DO_plan['component_name']
            #ramp_duration   = DO_plan['ramp_duration']  #seconds
            ramp_width      = DO_plan['ramp_width']     #seconds
            delay_duration  = DO_plan['delay_duration'] #seconds
            #low_voltage     = DO_plan['low_voltage']    #volts
            #high_voltage    = DO_plan['high_voltage']   #volts

            model = self.ebit_data_model.components[component_name]
            self.DO_components_in_task.append(component_name)

            num_width_steps = int(frequency * ramp_width)
            delay_steps = int(delay_duration * frequency)

            ramp_width_array = [True] * num_width_steps
            delay_array = [False] * delay_steps

            full_command = []
            full_command.extend(delay_array)
            full_command.extend(ramp_width_array)

            wait_steps = total_samples-len(full_command)  # remaining samples until we fill out the total time
            assert wait_steps >= 0
            wait_array = [False]*wait_steps
            full_command.extend(wait_array)

            DO_task.do_channels.add_do_chan(f'{model.output_card}/{model.output_pin}', line_grouping=LineGrouping.CHAN_PER_LINE)
            DO_command.append(full_command)

        if len(DO_command) == 1:
            #This happens when only one DO component is used.
            # The NIDAQ task expects the array to be formatted this way, i.e. [1,2,3] instead of [ [1,2,3] ] for single-channel tasks
            DO_command=DO_command[0]

        for AO_trigger_plan in AO_trigger_plans:
            component_name  = AO_trigger_plan['component_name']
            ramp_width      = AO_trigger_plan['ramp_width']     #seconds
            delay_duration  = AO_trigger_plan['delay_duration'] #seconds
            model = self.ebit_data_model.components[component_name]
            self.AO_components_in_task.append(component_name)

            num_width_steps = int(frequency * ramp_width)
            delay_steps = int(delay_duration * frequency)

            ramp_width_array = [5] * num_width_steps #hard-coded 5V high
            delay_array = [0] * delay_steps #hard-coded 0V low

            full_command = []
            full_command.extend(delay_array)
            full_command.extend(ramp_width_array)

            wait_steps = total_samples-len(full_command)  # remaining samples until we fill out the total time
            assert wait_steps >= 0
            wait_array = [0]*wait_steps
            full_command.extend(wait_array)

            AO_task.ao_channels.add_ao_voltage_chan(f'{model.output_card}/{model.output_pin}',
                                                    min_val=model.output_min_volts,
                                                    max_val=model.output_max_volts)
            full_command = np.clip(full_command, model.output_min_volts, model.output_max_volts).tolist()
            AO_command.append(full_command)

        if len(AO_command) == 1:
            #This happens when only one AO component is used.
            # The NIDAQ task expects the array to be formatted this way, i.e. [1,2,3] instead of [ [1,2,3] ] for single-channel tasks
            AO_command=AO_command[0]

        ### Determine if AO and DO need to be synchronized
        if len(AO_command)>0:
            AO_task.timing.cfg_samp_clk_timing(frequency, sample_mode=AcquisitionType.CONTINUOUS)
            AO_task.write(AO_command, auto_start=False)

            if len(DO_command)>0:
                DO_task.timing.cfg_samp_clk_timing(frequency, source="/pcie-6738/ao/SampleClock",
                                                   sample_mode=AcquisitionType.CONTINUOUS)
                DO_task.triggers.start_trigger.cfg_dig_edge_start_trig("/pcie-6738/ao/StartTrigger")
                DO_task.write(DO_command, auto_start=False)
                DO_task.start()
            AO_task.start()
        elif len(DO_command)>0:
            DO_task.timing.cfg_samp_clk_timing(frequency, sample_mode=AcquisitionType.CONTINUOUS)
            DO_task.write(DO_command, auto_start=False)
            DO_task.start()

    def start_custom_timing_loop(self, custom_timing_plan, frequency=200000, enable_LPF=False):
        #This function takes in lists of timing loop "plans":
        #  e.g., AO_plans = [ {Drifttube2, ramp_duration, ramp_width, delay_duration, low_voltage, high_voltage}, {...}]
        #This is called whenever the timing loop was loaded from a timing csv file (i.e., not made within the GUI).
        #The goal is to synchronize all the plans together. To do this, we will use the AO clock for all plans and
        # trigger the DO plans with the AO clock.

        #Ensure a voltage ramp isn't in progress since the AO clock is needed for both ramps and timing.
        if hasattr(self, 'ramp_task'):
            print("A voltage ramp is already in progress. Voltage not changed.")
            return 2
        cycle_time = custom_timing_plan["time_s"]
        total_pts = int(frequency * cycle_time)
        AO_plans = []
        AO_trigger_plans = []
        DO_plans = []

        for plan in custom_timing_plan.keys():
            if "DO:" in plan:
                DO_plans.append(plan)
            elif "AO:" in plan:
                AO_trigger_plans.append(plan)
            elif "time_s" not in plan: #the first column of the timing csv file is "time_s", the total cycle time.
                AO_plans.append(plan)

        AO_command = []
        DO_command = []

        if len(AO_plans)+len(AO_trigger_plans) > 0:
            AO_task = nidaqmx.Task()
            self.tasks["AO_timing"] = AO_task
        if len(DO_plans) > 0:
            DO_task = nidaqmx.Task()
            self.tasks["DO_timing"] = DO_task

        for i, component_name in enumerate(AO_plans):
            model = self.ebit_data_model.components[component_name]
            self.AO_components_in_task.append(component_name)
            values = np.array(custom_timing_plan[component_name]) # [ [times], [voltages] ]
            voltages = values[1]*model.output_voltage_slope + model.output_voltage_y_intercept
            if component_name == "time_s":
                continue
            full_command = []
            if len(values[0])>1:
                times = values[0]*frequency/1000 #converts from ms to sample number
            else: #if there is only one voltage setting
                times = values[0]
            for j in range(len(times)): #[time, voltage] pairs
                if j != (len(times)-1): #if this isn't the last step in the loop...
                    vs = np.linspace(start=voltages[j], stop=voltages[j+1], num=int(times[j+1]-times[j]))
                else:
                    vs = np.linspace(start=voltages[j], stop=voltages[0], num=int(total_pts-len(full_command)))
                full_command.extend(vs)

            full_command = np.clip(full_command, model.output_min_volts, model.output_max_volts).tolist()

            AO_task.ao_channels.add_ao_voltage_chan(f'{model.output_card}/{model.output_pin}',
                                                    min_val=model.output_min_volts,
                                                    max_val=model.output_max_volts)

            if enable_LPF:
                def butter_lowpass(cutoff, fs, order=1):
                    #I use a first-order filter so the filter's time delay is constant for all frequencies
                    return butter(order, cutoff, fs=fs, btype='low', analog=False)

                def butter_lowpass_filter(data, cutoff, fs, order=1):
                    b, a = butter_lowpass(cutoff, fs, order=order)
                    y = lfilter(b, a, data)
                    return y

                # Filter requirements.
                order = 3
                fs = frequency  # sample rate, Hz
                #slew rate of the trek is 350 V/us
                cutoff = 1000#fs/2-1  # desired cutoff frequency of the filter, Hz

                #import matplotlib.pyplot as plt

                T = cycle_time  # seconds
                n = total_pts  # total number of samples

                full_command_baseline = full_command[0]
                data = np.array(full_command) - full_command_baseline #sets the first value to 0. the filter misbehaves otherwise.

                y = butter_lowpass_filter(data, cutoff, fs, order)
                y = y + full_command_baseline
                full_command = y.tolist()
            full_command = np.clip(full_command, model.output_min_volts, model.output_max_volts).tolist()

            AO_command.append(full_command)

        for i, component_name in enumerate(DO_plans):
            model = self.ebit_data_model.components[component_name]
            self.DO_components_in_task.append(component_name)
            values = custom_timing_plan[component_name]
            if component_name == "time_s":
                continue
            full_command = []
            if len(values[0])>1:
                times = np.array(values[0])*frequency/1000 #converts from ms to sample number
                voltages = values[1]
            else: #if there is only one voltage setting
                times = values[0]
                voltages = values[1]
            for j in range(len(times)): #[time, voltage] pairs
                if j != (len(times)-1): #if this isn't the last step in the loop...
                    vs = [bool(voltages[j])]*int(times[j+1]-times[j])
                else:
                    vs = [bool(voltages[j])]*int(total_pts-len(full_command))
                full_command.extend(vs)

            DO_task.do_channels.add_do_chan(f'{model.output_card}/{model.output_pin}')

            DO_command.append(full_command)

        if len(DO_command) == 1:
            #This happens when only one DO component is used.
            # The NIDAQ task expects the array to be formatted this way, i.e. [1,2,3] instead of [ [1,2,3] ] for single-channel tasks
            DO_command=DO_command[0]

        for i, component_name in enumerate(AO_trigger_plans):
            model = self.ebit_data_model.components[component_name]
            self.AO_components_in_task.append(component_name)
            values = custom_timing_plan[component_name]
            if component_name == "time_s":
                continue
            full_command = []
            if len(values[0])>1:
                times = np.array(values[0])*frequency/1000 #converts from ms to sample number
                voltages = values[1]
            else: #if there is only one voltage setting
                times = values[0]
                voltages = values[1]
            for j in range(len(times)): #[time, voltage] pairs
                if j != (len(times)-1): #if this isn't the last step in the loop...
                    vs = [5*bool(voltages[j])]*int(times[j+1]-times[j]) #outputting 5V as "True" for the trigger, 0V as "False"
                else:
                    vs = [5*bool(voltages[j])]*int(total_pts-len(full_command))
                full_command.extend(vs)

            AO_task.ao_channels.add_ao_voltage_chan(f'{model.output_card}/{model.output_pin}',
                                                    min_val=model.output_min_volts,
                                                    max_val=model.output_max_volts)

            AO_command.append(full_command)

        if len(AO_command) == 1:
            # This happens when only one AO component is used.
            # The NIDAQ task expects the array to be formatted this way, i.e. [1,2,3] instead of [ [1,2,3] ] for single-channel tasks
            AO_command = AO_command[0]

        ### Determine if AO and DO need to be synchronized
        if len(AO_command)>0:
            AO_task.timing.cfg_samp_clk_timing(frequency, sample_mode=AcquisitionType.CONTINUOUS)
            AO_task.write(AO_command, auto_start=False)

            if len(DO_command)>0:
                DO_task.timing.cfg_samp_clk_timing(frequency, source="/pcie-6738/ao/SampleClock",
                                                   sample_mode=AcquisitionType.CONTINUOUS)
                DO_task.triggers.start_trigger.cfg_dig_edge_start_trig("/pcie-6738/ao/StartTrigger")
                DO_task.write(DO_command, auto_start=False)
                DO_task.start()
            AO_task.start()
        elif len(DO_command)>0:
            DO_task.timing.cfg_samp_clk_timing(frequency, sample_mode=AcquisitionType.CONTINUOUS)
            DO_task.write(DO_command, auto_start=False)
            DO_task.start()

    def check_for_ramp(self):
        #Check if a voltage ramp is active. We do this to avoid using the AO clock with two tasks at once.
        return hasattr(self, 'ramp_task')

    def clear_all_voltages(self, turn_off_mevva_trigger_power=False):
        self.stop_timing_loop()
        if hasattr(self, 'ramp_task'):
            self.ramp_done_callback(0, 0, 0)
        for key, value in self.ebit_data_model.components.items():
            if "DO:" in key:
                self.set_digital(key, False)
            elif value.output_card and "MeVVA Trigger Power" not in key:
                with nidaqmx.Task() as task:
                    task.ao_channels.add_ao_voltage_chan(f'{value.output_card}/{value.output_pin}',
                                                         min_val=value.output_min_volts,
                                                         max_val=value.output_max_volts)
                    task.write(0)
                    task.wait_until_done()
                    self.ebit_data_model.components[key].output_voltage_value = 0
        if turn_off_mevva_trigger_power:
            with nidaqmx.Task() as task:
                key = "AO: MeVVA Trigger Power"
                value = self.ebit_data_model.components[key]
                task.ao_channels.add_ao_voltage_chan(f'{value.output_card}/{value.output_pin}',
                                                     min_val=value.output_min_volts,
                                                     max_val=value.output_max_volts)
                task.write(0)
                task.wait_until_done()
                self.ebit_data_model.components[key].output_voltage_value = 0
                print("Turning off MeVVA Trigger Power")

    def set_voltage(self, name, new_value_V, ramp_Vs=0):
        model = self.ebit_data_model.components[name]
        new_value = float(new_value_V)
        new_value = new_value * model.output_voltage_slope + model.output_voltage_y_intercept
        if new_value>model.output_max_volts:
            print(f"Caution: Voltage {new_value} exceeds maximum voltage ({model.output_max_volts}) for {name}.")
            start = self.ebit_data_model.components[name].output_voltage_value
            start_V = (start - model.output_voltage_y_intercept) / model.output_voltage_slope
            return([0, new_value, model.output_max_volts, start_V])
        elif new_value<model.output_min_volts:
            print(f"Caution: Voltage {new_value} is less than minimum voltage ({model.output_min_volts}) for {name}.")
            start = self.ebit_data_model.components[name].output_voltage_value
            start_V = (start - model.output_voltage_y_intercept) / model.output_voltage_slope
            return([0, new_value, model.output_min_volts, start_V])
        if hasattr(self, 'ramp_task'):
            print("A voltage ramp is already in progress. Voltage not changed.")
            start = self.ebit_data_model.components[name].output_voltage_value
            start_V = (start - model.output_voltage_y_intercept) / model.output_voltage_slope
            return [2, start_V]
        if (ramp_Vs!=0) and (len(self.AO_components_in_task) + len(self.DO_components_in_task)): #if there is a timing loop in progress and we want to try and ramp
            print("A timing loop is already in progress, so a ramp cannot be started. Voltage not changed.")
            start = self.ebit_data_model.components[name].output_voltage_value
            start_V = (start - model.output_voltage_y_intercept) / model.output_voltage_slope
            return [3, start_V]
        if (ramp_Vs == 0) or (new_value == self.ebit_data_model.components[name].output_voltage_value):
            with nidaqmx.Task() as task:
                task.ao_channels.add_ao_voltage_chan(f'{model.output_card}/{model.output_pin}',
                                                     min_val=model.output_min_volts,
                                                     max_val=model.output_max_volts)
                print("SETTING VOLTAGE: ", name, new_value)
                task.write(new_value)
                task.wait_until_done()
                self.ebit_data_model.components[name].output_voltage_value = float(new_value)
        else:
            self.ramp_task = nidaqmx.Task()
            self.ramp_task.ao_channels.add_ao_voltage_chan(f'{model.output_card}/{model.output_pin}',
                                                           min_val=model.output_min_volts,
                                                           max_val=model.output_max_volts)
            start = self.ebit_data_model.components[name].output_voltage_value
            start_V =  (start - model.output_voltage_y_intercept)/model.output_voltage_slope
            print("RAMPING VOLTAGE: ", name, "from", start_V, "to",new_value_V, "V at", ramp_Vs, "V/s.")
            frequency = 200000

            def butter_lowpass(cutoff, fs, order=1):
                return butter(order, cutoff, fs=fs, btype='low', analog=False)

            def butter_lowpass_filter(data, cutoff, fs, order=1):
                b, a = butter_lowpass(cutoff, fs, order=order)
                y = lfilter(b, a, data)
                return y

            ramp_duration = abs(new_value_V-start_V)/ramp_Vs  # seconds
            low_voltage = start  # already scaled
            high_voltage = new_value  # already scaled

            num_ramp_steps = int(frequency * ramp_duration)
            if num_ramp_steps > 1: #very short ramps should just be skipped
                ramp_up_array = np.linspace(low_voltage, high_voltage, num=num_ramp_steps).tolist()
            else:
                ramp_up_array = [high_voltage]
            full_command = []
            full_command.extend(ramp_up_array)

            ### Low-pass filter
            # Filter requirements.
            order = 3
            fs = frequency  # sample rate, Hz
            # slew rate of the trek is 350 V/us
            cutoff = 1000  # fs/2-1  # desired cutoff frequency of the filter, Hz

            T = ramp_duration  # seconds
            n = int(T * fs)  # total number of samples
            t = np.linspace(0, T, n, endpoint=False)

            full_command_baseline = full_command[0]
            data = np.array(
                full_command) - full_command_baseline  # sets the first value to 0. the filter misbehaves otherwise.

            y = butter_lowpass_filter(data, cutoff, fs, order)
            y = y + full_command_baseline

            full_command = y.tolist()
            full_command.extend([high_voltage]) #ends on proper voltage always


            full_command = np.clip(full_command, model.output_min_volts, model.output_max_volts).tolist()

            self.ramp_task.register_done_event(self.ramp_done_callback)
            self.ramp_task.timing.cfg_samp_clk_timing(frequency, sample_mode=AcquisitionType.FINITE, samps_per_chan=len(full_command))
            self.ramp_task.write(full_command)
            self.ramp_task.start()

            #self.task.wait_until_done()
            self.ebit_data_model.components[name].output_voltage_value = float(new_value)
        return [1]

    def ramp_done_callback(self, task_handle, status, callback_data):
        #this gets called automatically when a voltage ramp completes, or can be called early to end the ramp.
        self.ramp_task.close()
        del self.ramp_task
        print("Voltage ramp stopped.")
        return 0

    def read_all_voltages(self):
        for key, value in self.ebit_data_model.components.items():
            model = self.ebit_data_model.components[key]
            if value.input_voltage_card:
                with nidaqmx.Task() as task:
                    task.ai_channels.add_ai_voltage_chan(f'{value.input_voltage_card}/{value.input_voltage_pin}',
                                                         terminal_config=TerminalConfiguration.RSE,
                                                         min_val=-10.0, max_val=10.0)
                    data = task.read()
                    if isinstance(data, float):
                        value.input_voltage_value = data * model.voltage_slope + model.voltage_y_intercept
                    else:
                        value.input_voltage_value = None
                        print(f"Error reading {key} voltage.")

            if value.input_current_card:
                with nidaqmx.Task() as task:
                    task.ai_channels.add_ai_voltage_chan(f'{value.input_current_card}/{value.input_current_pin}',
                                                         terminal_config=TerminalConfiguration.RSE,
                                                         min_val=-10.0, max_val=10.0)
                    data = task.read()
                    if isinstance(data, float):
                        value.input_current_value = data * model.current_slope + model.current_y_intercept
                    else:
                        value.input_current_value = None
                        print(f"Error reading {key} current.")

                    print(key, value.input_current_value, data)

    def set_digital(self, name, new_value):
        model = self.ebit_data_model.components[name]
        new_value = bool(new_value)
        with nidaqmx.Task() as task:
            task.do_channels.add_do_chan(f'{model.output_card}/{model.output_pin}')
            print("SETTING DIGITAL OUTPUT: ", name, new_value)
            task.write(new_value)
            task.wait_until_done()


class SimulatedEbitVoltageController(EbitVoltageController):

    def set_voltage(self, name, new_value):
        model = self.ebit_data_model.components[name]
        new_value = float(new_value)
        new_value = new_value * model.output_voltage_slope + model.output_voltage_y_intercept

        if model.output_voltage_value != new_value:
            self.ebit_data_model.components[name].output_voltage_value = float(new_value)

    def read_all_voltages(self):
        for key, value in self.ebit_data_model.components.items():
            model = self.ebit_data_model.components[key]
            if value.input_voltage_card and value.output_voltage_value is not None:
                if value.output_voltage_value == 0:
                    value.input_voltage_value = 0
                else:
                    value.input_voltage_value = float(value.output_voltage_value) * model.voltage_slope + model.voltage_y_intercept + uniform(-.2, .2)
            if value.input_current_card and value.output_voltage_value is not None:
                if value.output_voltage_value == 0:
                    value.input_current_value = 0
                else:
                    value.input_current_value = float(value.output_voltage_value) * model.current_slope + model.current_y_intercept + uniform(-.2, .2)

    def clear_all_voltages(self):
        for key, value in self.ebit_data_model.components.items():
            if value.output_card:
                self.ebit_data_model.components[key].output_voltage_value = 0
