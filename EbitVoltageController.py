from random import uniform
from typing import Union

import numpy as np
import numpy.typing as npt

import nidaqmx
from nidaqmx import Task
from nidaqmx.constants import TerminalConfiguration, AcquisitionType, LineGrouping
from scipy.signal import butter, lfilter

from EbitDataModel import EbitDataModel

import sys
GUI_ONLY = '--gui-only'in sys.argv

import functools

def shortcircuit(disable: bool, result=None):
    """
    Decorator to either run a function normally or outright return a default value.

    disable -- if true then return result, otherwise run function normally
    result -- value to return when disable is true
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            if disable:
                return result
            return func(*args, **kwargs)
        return wrapper
    return decorator

class EbitVoltageController:
    tasks = {}
    ao_components_in_task = []
    do_components_in_task = []
    ebit_data_model: EbitDataModel
    ramp_task: Task | None = None

    def __init__(self, ebit_data_model:  EbitDataModel):
        self.ebit_data_model = ebit_data_model

    @shortcircuit(GUI_ONLY)
    def stop_timing_loop(self):
        if "ao_timing" in self.tasks:
            ao_task = self.tasks.pop("ao_timing")
            ao_task.stop()
            ao_task.close()
            for component_name in self.ao_components_in_task:
                self.set_voltage(component_name, 0)
            self.ao_components_in_task = []

        if "do_timing" in self.tasks:
            do_task = self.tasks.pop("do_timing")
            do_task.stop()
            do_task.close()
            for component_name in self.do_components_in_task:
                self.set_digital(component_name, False)
            self.do_components_in_task = []

    @shortcircuit(GUI_ONLY)
    def start_timing_loop(self, ao_plans, ao_trigger_plans, do_plans, cycle_time, frequency=200000, enable_lpf=False):
        """
        This function takes in lists of timing loop "plans":
          e.g., ao_plans = [ {Drifttube2, ramp_duration, ramp_width, delay_duration, low_voltage, high_voltage}, {...}]
        This is called whenever the timing loop was made with the GUI text boxes (i.e., not a timing csv file).
        The goal is to synchronize all the plans together. To do this, we will use the ao clock for all plans and
         trigger the do plans with the ao clock.
        enable_LPF: True/False on whether to apply a low-pass filter
        """

        # Ensure a voltage ramp isn't in progress since the ao clock is needed for both ramps and timing.
        if self.ramp_task is not None:
            print("A voltage ramp is already in progress. Voltage not changed.")
            return 2

        total_samples = int(frequency * cycle_time)

        #Create tasks for each plan
        if len(ao_plans) + len(ao_trigger_plans) > 0:
            ao_task = nidaqmx.Task()
            self.tasks["ao_timing"] = ao_task
        else:
            ao_task = None
        if len(do_plans) > 0:
            do_task = nidaqmx.Task()
            self.tasks["do_timing"] = do_task
        else:
            do_task = None

        # ao_command and do_command are lists of lists telling each ao/do port to do
        ao_command = []
        do_command = []

        for ao_plan in ao_plans:
            component_name = ao_plan['component_name']
            ramp_duration = ao_plan['ramp_duration']  #seconds
            ramp_width = ao_plan['ramp_width']  #seconds
            delay_duration = ao_plan['delay_duration']  #seconds
            low_voltage = ao_plan['low_voltage']  #volts
            high_voltage = ao_plan['high_voltage']  #volts

            #tracks which components have an active task
            self.ao_components_in_task.append(component_name)

            model = self.ebit_data_model.components[component_name]
            # Scale voltages based on their slope/y-intercept values in the CSV.
            low_voltage = low_voltage * model.output_voltage_slope + model.output_voltage_y_intercept
            high_voltage = high_voltage * model.output_voltage_slope + model.output_voltage_y_intercept

            num_ramp_steps = int(frequency * ramp_duration)
            num_width_steps = int(frequency * ramp_width)
            delay_steps = int(delay_duration * frequency)

            # delay, ramp up, hold, ramp down
            full_command = []
            full_command += [low_voltage] * delay_steps
            full_command += np.linspace(low_voltage, high_voltage, num=num_ramp_steps).tolist()
            full_command += [high_voltage] * num_width_steps
            full_command += np.linspace(high_voltage, low_voltage, num=num_ramp_steps).tolist()

            wait_steps = total_samples - len(full_command)  # remaining samples until we fill out the total time
            assert wait_steps >= 0
            wait_array = [low_voltage] * wait_steps
            full_command += wait_array

            if enable_lpf:
                full_command = butter_lowpass_filter(full_command, frequency)

            ao_task.ao_channels.add_ao_voltage_chan(f'{model.output_card}/{model.output_pin}',
                                                    min_val=model.output_min_volts,
                                                    max_val=model.output_max_volts)

            full_command = np.clip(full_command, model.output_min_volts, model.output_max_volts).tolist()
            ao_command.append(full_command)

        for do_plan in do_plans:
            component_name = do_plan['component_name']
            model = self.ebit_data_model.components[component_name]
            self.do_components_in_task.append(component_name)

            delay_steps = int(frequency * do_plan['delay_duration'])
            num_width_steps = int(frequency * do_plan['ramp_width'])

            full_command = []
            full_command += [False] * delay_steps
            full_command += [True] * num_width_steps

            wait_steps = total_samples - len(full_command)  # remaining samples until we fill out the total time
            assert wait_steps >= 0 # TODO: Check for this much earlier from the ramp_width/delay_duration
            full_command += [False] * wait_steps

            do_task.do_channels.add_do_chan(f'{model.output_card}/{model.output_pin}',
                                            line_grouping=LineGrouping.CHAN_PER_LINE)
            do_command.append(full_command)

        for ao_trigger_plan in ao_trigger_plans:
            component_name = ao_trigger_plan['component_name']

            model = self.ebit_data_model.components[component_name]
            self.ao_components_in_task.append(component_name)

            delay_steps = int(ao_trigger_plan['delay_duration'] * frequency)
            num_width_steps = int(frequency * ao_trigger_plan['ramp_width'])

            full_command = []
            full_command += [0] * delay_steps
            full_command += [5] * num_width_steps

            wait_steps = total_samples - len(full_command)  # remaining samples until we fill out the total time
            assert wait_steps >= 0
            wait_array = [0] * wait_steps
            full_command.extend(wait_array)

            ao_task.ao_channels.add_ao_voltage_chan(f'{model.output_card}/{model.output_pin}',
                                                    min_val=model.output_min_volts,
                                                    max_val=model.output_max_volts)
            full_command = np.clip(full_command, model.output_min_volts, model.output_max_volts).tolist()
            ao_command.append(full_command)

        EbitVoltageController.send_command(ao_command, ao_task, do_command, do_task, frequency)
        return None

    @shortcircuit(GUI_ONLY)
    def start_custom_timing_loop(self, custom_timing_plan, frequency=200000, enable_lpf=False):
        #This function takes in lists of timing loop "plans":
        #  e.g., ao_plans = [ {Drifttube2, ramp_duration, ramp_width, delay_duration, low_voltage, high_voltage}, {...}]
        #This is called whenever the timing loop was loaded from a timing csv file (i.e., not made within the GUI).
        #The goal is to synchronize all the plans together. To do this, we will use the ao clock for all plans and
        # trigger the do plans with the ao clock.

        #Ensure a voltage ramp isn't in progress since the ao clock is needed for both ramps and timing.
        if self.ramp_task is not None:
            print("A voltage ramp is already in progress. Voltage not changed.")
            return 2
        cycle_time = custom_timing_plan["time_s"]
        total_pts = int(frequency * cycle_time)
        ao_plans = []
        ao_trigger_plans = []
        do_plans = []

        for plan in custom_timing_plan.keys():
            if "do:" in plan:
                do_plans.append(plan)
            elif "ao:" in plan:
                ao_trigger_plans.append(plan)
            elif "time_s" not in plan:  #the first column of the timing csv file is "time_s", the total cycle time.
                ao_plans.append(plan)

        ao_command = []
        do_command = []

        if len(ao_plans) + len(ao_trigger_plans) > 0:
            ao_task = nidaqmx.Task()
            self.tasks["ao_timing"] = ao_task
        else:
            ao_task = None
        if len(do_plans) > 0:
            do_task = nidaqmx.Task()
            self.tasks["do_timing"] = do_task
        else:
            do_task = None

        for i, component_name in enumerate(ao_plans):
            model = self.ebit_data_model.components[component_name]
            self.ao_components_in_task.append(component_name)
            values = np.array(custom_timing_plan[component_name])  # [ [times], [voltages] ]
            voltages = values[1] * model.output_voltage_slope + model.output_voltage_y_intercept
            if component_name == "time_s":
                continue
            full_command = []
            if len(values[0]) > 1:
                times = values[0] * frequency / 1000  # converts from ms to sample number
            else: # if there is only one voltage setting
                times = values[0]
            for j in range(len(times)):  #[time, voltage] pairs
                if j != (len(times) - 1):  #if this isn't the last step in the loop...
                    vs = np.linspace(start=voltages[j], stop=voltages[j + 1], num=int(times[j + 1] - times[j]))
                else:
                    vs = np.linspace(start=voltages[j], stop=voltages[0], num=int(total_pts - len(full_command)))
                full_command.extend(vs)

            full_command = np.clip(full_command, model.output_min_volts, model.output_max_volts).tolist()

            ao_task.ao_channels.add_ao_voltage_chan(f'{model.output_card}/{model.output_pin}',
                                                    min_val=model.output_min_volts,
                                                    max_val=model.output_max_volts)

            if enable_lpf:
                full_command = butter_lowpass_filter(full_command, frequency)

            full_command = np.clip(full_command, model.output_min_volts, model.output_max_volts).tolist()

            ao_command.append(full_command)

        for i, component_name in enumerate(do_plans):
            model = self.ebit_data_model.components[component_name]
            self.do_components_in_task.append(component_name)
            values = custom_timing_plan[component_name]
            if component_name == "time_s":
                continue
            full_command = []
            if len(values[0]) > 1:
                times = np.array(values[0]) * frequency / 1000  #converts from ms to sample number
                voltages = values[1]
            else:  #if there is only one voltage setting
                times = values[0]
                voltages = values[1]
            for j in range(len(times)):  #[time, voltage] pairs
                if j != (len(times) - 1):  #if this isn't the last step in the loop...
                    vs = [bool(voltages[j])] * int(times[j + 1] - times[j])
                else:
                    vs = [bool(voltages[j])] * int(total_pts - len(full_command))
                full_command.extend(vs)

            do_task.do_channels.add_do_chan(f'{model.output_card}/{model.output_pin}')

            do_command.append(full_command)

        for i, component_name in enumerate(ao_trigger_plans):
            model = self.ebit_data_model.components[component_name]
            self.ao_components_in_task.append(component_name)
            values = custom_timing_plan[component_name]
            if component_name == "time_s":
                continue
            full_command = []
            if len(values[0]) > 1:
                times = np.array(values[0]) * frequency / 1000  #converts from ms to sample number
                voltages = values[1]
            else:  #if there is only one voltage setting
                times = values[0]
                voltages = values[1]
            for j in range(len(times)):  #[time, voltage] pairs
                if j != (len(times) - 1):  #if this isn't the last step in the loop...
                    vs = [5 * bool(voltages[j])] * int(
                        times[j + 1] - times[j])  #outputting 5V as "True" for the trigger, 0V as "False"
                else:
                    vs = [5 * bool(voltages[j])] * int(total_pts - len(full_command))
                full_command.extend(vs)

            ao_task.ao_channels.add_ao_voltage_chan(f'{model.output_card}/{model.output_pin}',
                                                    min_val=model.output_min_volts,
                                                    max_val=model.output_max_volts)

            ao_command.append(full_command)

        EbitVoltageController.send_command(ao_command, ao_task, do_command, do_task, frequency)
        return None

    @shortcircuit(GUI_ONLY)
    @staticmethod
    def send_command(ao_command: list[list], ao_task: Task | None, do_command: list[list], do_task: Task | None,
                     frequency: int) -> None:


        if len(ao_command) == 1:
            ao_command = ao_command[0]
        if len(do_command) == 1:
            do_command = do_command[0]

        if len(ao_command) > 0:
            ao_task.timing.cfg_samp_clk_timing(frequency, sample_mode=AcquisitionType.CONTINUOUS)
            ao_task.write(ao_command, auto_start=False)

            if len(do_command) > 0:
                do_task.timing.cfg_samp_clk_timing(frequency, source="/pcie-6738/ao/SampleClock",
                                                   sample_mode=AcquisitionType.CONTINUOUS)
                do_task.triggers.start_trigger.cfg_dig_edge_start_trig("/pcie-6738/ao/StartTrigger")
                do_task.write(do_command, auto_start=False)
                do_task.start()
            ao_task.start()

        elif len(do_command) > 0:
            do_task.timing.cfg_samp_clk_timing(frequency, sample_mode=AcquisitionType.CONTINUOUS)
            do_task.write(do_command, auto_start=False)
            do_task.start()

    @shortcircuit(GUI_ONLY)
    def clear_all_voltages(self, turn_off_mevva_trigger_power=False):

        self.stop_timing_loop()

        if self.ramp_task is not None:
            self.ramp_done_callback()

        for name, comp in self.ebit_data_model.components.items():
            if "do:" in name.lower():
                self.set_digital(name, False)
            elif comp.output_card and "MeVVA Trigger Power" not in name:
                with nidaqmx.Task() as task:
                    if comp.output_min_volts is None or comp.output_max_volts is None:
                        raise ValueError(f'Component {name} has None in output_min_volts or output_max_volts.\n'
                                         f'{comp}')
                    task.ao_channels.add_ao_voltage_chan(f'{comp.output_card}/{comp.output_pin}',
                                                         min_val=comp.output_min_volts,
                                                         max_val=comp.output_max_volts)
                    task.write(0)
                    task.wait_until_done()
                    self.ebit_data_model.components[name].output_voltage_comp = 0

        if turn_off_mevva_trigger_power:
            with nidaqmx.Task() as task:
                name = "ao: MeVVA Trigger Power"
                comp = self.ebit_data_model.components[name]
                task.ao_channels.add_ao_voltage_chan(f'{comp.output_card}/{comp.output_pin}',
                                                     min_val=comp.output_min_volts,
                                                     max_val=comp.output_max_volts)
                task.write(0)
                task.wait_until_done()
                self.ebit_data_model.components[name].output_voltage_comp = 0
                print("Turning off MeVVA Trigger Power")

    @shortcircuit(GUI_ONLY, [1])
    def set_voltage(self, name, new_value_v, ramp_vs=0):

        model = self.ebit_data_model.components[name]
        new_value = float(new_value_v)
        new_value = new_value * model.output_voltage_slope + model.output_voltage_y_intercept

        if new_value > model.output_max_volts:
            print(f"Caution: Voltage {new_value} exceeds maximum voltage ({model.output_max_volts}) for {name}.")
            start = self.ebit_data_model.components[name].output_voltage_value
            start_v = (start - model.output_voltage_y_intercept) / model.output_voltage_slope
            return [0, new_value, model.output_max_volts, start_v]
        elif new_value < model.output_min_volts:
            print(f"Caution: Voltage {new_value} is less than minimum voltage ({model.output_min_volts}) for {name}.")
            start = self.ebit_data_model.components[name].output_voltage_value
            start_v = (start - model.output_voltage_y_intercept) / model.output_voltage_slope
            return [0, new_value, model.output_min_volts, start_v]
        if self.ramp_task is not None:
            print("A voltage ramp is already in progress. Voltage not changed.")
            start = self.ebit_data_model.components[name].output_voltage_value
            start_v = (start - model.output_voltage_y_intercept) / model.output_voltage_slope
            return [2, start_v]
        if (ramp_vs != 0) and (len(self.ao_components_in_task) + len(
                self.do_components_in_task)):  #if there is a timing loop in progress and we want to try and ramp
            print("A timing loop is already in progress, so a ramp cannot be started. Voltage not changed.")
            start = self.ebit_data_model.components[name].output_voltage_value
            start_v = (start - model.output_voltage_y_intercept) / model.output_voltage_slope
            return [3, start_v]
        if (ramp_vs == 0) or (new_value == self.ebit_data_model.components[name].output_voltage_value):
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
            start_v = (start - model.output_voltage_y_intercept) / model.output_voltage_slope
            print("RAMPING VOLTAGE: ", name, "from", start_v, "to", new_value_v, "V at", ramp_vs, "V/s.")
            frequency = 200000

            ramp_duration = abs(new_value_v - start_v) / ramp_vs  # seconds
            low_voltage = start  # already scaled
            high_voltage = new_value  # already scaled

            num_ramp_steps = int(frequency * ramp_duration)
            if num_ramp_steps > 1:  #very short ramps should just be skipped
                ramp_up_array = np.linspace(low_voltage, high_voltage, num=num_ramp_steps).tolist()
            else:
                ramp_up_array = [high_voltage]

            full_command = []
            full_command += ramp_up_array

            full_command = butter_lowpass_filter(full_command, frequency).to_list()
            full_command += [high_voltage]  #ends on proper voltage always

            full_command = np.clip(full_command, model.output_min_volts, model.output_max_volts).tolist()

            self.ramp_task.register_done_event(self.ramp_done_callback)
            self.ramp_task.timing.cfg_samp_clk_timing(frequency, sample_mode=AcquisitionType.FINITE,
                                                      samps_per_chan=len(full_command))
            self.ramp_task.write(full_command)
            self.ramp_task.start()

            self.ebit_data_model.components[name].output_voltage_value = float(new_value)
        return [1]

    @shortcircuit(GUI_ONLY, 0)
    def ramp_done_callback(self):
        #this gets called automatically when a voltage ramp completes, or can be called early to end the ramp.
        self.ramp_task.close()
        self.ramp_task = None
        print("Voltage ramp stopped.")
        return 0

    @shortcircuit(GUI_ONLY)
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

    @shortcircuit(GUI_ONLY)
    def set_digital(self, name, new_value):
        model = self.ebit_data_model.components[name]
        new_value = bool(new_value)
        with nidaqmx.Task() as task:
            task.do_channels.add_do_chan(f'{model.output_card}/{model.output_pin}')
            print("SETTING DIGITAL OUTPUT: ", name, new_value)
            task.write(new_value)
            task.wait_until_done()

@shortcircuit(GUI_ONLY)
def butter_lowpass_filter(signal: Union[npt.NDArray, list], fs: float, cutoff: float = 1000, order: int = 3):
    if isinstance(signal, list):
        signal = np.array(signal)

    unfiltered = signal.copy()

    b, a = butter(order, cutoff, fs=fs, btype='low', analog=False, output='ba')
    return lfilter(b, a, unfiltered - unfiltered[0]) + unfiltered[0]


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
                    value.input_voltage_value = float(
                        value.output_voltage_value) * model.voltage_slope + model.voltage_y_intercept + uniform(-.2, .2)
            if value.input_current_card and value.output_voltage_value is not None:
                if value.output_voltage_value == 0:
                    value.input_current_value = 0
                else:
                    value.input_current_value = float(
                        value.output_voltage_value) * model.current_slope + model.current_y_intercept + uniform(-.2, .2)

    def clear_all_voltages(self):
        for key, value in self.ebit_data_model.components.items():
            if value.output_card:
                self.ebit_data_model.components[key].output_voltage_value = 0
