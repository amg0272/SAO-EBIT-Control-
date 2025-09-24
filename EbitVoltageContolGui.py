import cProfile

import configparser
import os
import sys
import numpy as np
from functools import partial
import pandas as pd
import datetime
import csv

from slack_sdk.webhook import WebhookClient
import pyqtgraph as pg #put this before pyside so that pyqtgraph uses pyqt instead of pyside (for interactive plots)
from PyQt6.QtCore import QTimer, QCoreApplication
from PyQt6.QtWidgets import (QApplication, QDialog,
                             QGridLayout, QGroupBox,
                             QLabel, QLineEdit, QMessageBox,
                             QMenu, QMenuBar, QPushButton, QVBoxLayout, QHBoxLayout, QCheckBox, QSpinBox,
                             QDoubleSpinBox, QFileDialog)
from scipy.signal import butter, lfilter
from EbitDataModel import EbitDataModel
from EbitVoltageController import SimulatedEbitVoltageController, EbitVoltageController
from LabJackTemps import TemperatureReader
import matplotlib.colors as mcolors

pg.setConfigOptions(antialias=True)
pg.setConfigOptions(useOpenGL=True)
#todo: add HV interlock?
class Dialog(QDialog):
    input_widgets = {} #text box widgets to set voltages on individual components
    timing_loop_widgets = {} #holds the widget rows (high V, low V, ramp duration, etc.) for each component that has timing capabilities
    timing_loop_start_stop_widgets = {} #start and stop buttons for the timing loop box
    timing_cycle_widgets = {} #holds the text box for the duration of the timing loop (in s) and the "Apply" button
    readonly_voltage_widgets = {} #text boxes that display the voltages that we read in from components
    readonly_current_widgets = {} #text boxes that display the currents that we read in from components
    v_plot_widgets = {} #holds all the voltage plot widgets so we can update them
    v_recent_values = {} #holds the most recent voltage (V) values for each component that monitors it.
    current_plot_widgets = {} #holds all the current plot widgets so we can update them
    current_recent_values = {} #holds the most recent Current (I) values for each component that monitors it.
    timing_loop_load_clear_widgets = {} #"Load" and "Clear" buttons to enable the custom timing loops via .csv files.
    custom_timing_plan = {} #custom timing loop .csv files are parsed into this dictionary.
    _timing_cycle_time_s = 1 #default timing cycle period, in seconds.
    logf = None #log file will go here
    log_index = int(0) #tracks how many times data is read from EBIT components, which happens once per second.
    log_period_s = int(5) #how often to record data to the log. Uses log_index to track this.
    trek_slew_rate = 330000 #350 V/us = 350,000 V/ms is the trek slew rate. I restrict it here to 330,000 V/ms for safety.
    enable_current_plots = False #True/False: whether to display plots of recent current readings
    enable_v_plots = False #True/False: whether to display plots of recent voltage readings
    enable_low_pass_filter = False #True/False: whether to apply an LPF to timing


    def __init__(self, ebit_data_model, ebit_controller):
        super().__init__()
        self.ebit_controller = ebit_controller
        self.ebit_data_model = ebit_data_model
        self.create_menu()
        self.create_grid_group_setvoltage_box()
        self.create_horizontal_group_box()
        self.create_grid_group_timingloop_box()

        main_layout = QVBoxLayout()
        main_layout.setMenuBar(self._menu_bar)
        main_layout.addWidget(self._grid_group_setvoltage_box)
        main_layout.addWidget(self._horizontalGroupBox)
        main_layout.addWidget(self._grid_group_timingloop_box)
        self.setLayout(main_layout)

        self.setWindowTitle("EBIT Voltage Monitor & Control")
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_inputs)

        config = configparser.ConfigParser()
        config.read('config.ini')
        self.slack_enabled = config['slack'].getboolean('enabled')
        self.slack = WebhookClient(config["slack"]["webhook_url"])
        self.makeLogFile()
        self.setModal(False)
        self.zero_all_voltages()
        self.ebit_controller.set_voltage("AO: MeVVA Trigger Power", 5) #turn on the circuit that sends the mevva optical trigger
        try:
            self.labjack = TemperatureReader()
        except:
            print("LabJack connection failed.")
            self.labjack = None

    def closeEvent(self, e):
        """
        Called when the user clicks the close icon in the window corner.
        """
        try:
            if self.logf:
                self.logf.close()
            self.zero_all_voltages()
            self.ebit_controller.clear_all_voltages(turn_off_mevva_trigger_power=True)
            if self.labjack:
                self.labjack.close_labjack()
        except Exception as e:
            print(e)
        QCoreApplication.quit()

    def makeLogFile(self):
        now = datetime.datetime.now()

        # Format the date and time as a string
        formatted_date_time = now.strftime("%Y-%m-%d_%Hh%Mm%Ss")

        # Create the file name using the formatted date and time
        file_name = f"ebit_log_{formatted_date_time}.csv"

        # Place file in the logs folder
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", file_name)
        # Create and open the file in write mode
        self.logf = open(path, 'a', newline='')
        self.logf_format = ['time']
        self.logf_format.extend([k for k in self.ebit_data_model.components.keys() if "O:" not in k])
        self.logf_format.extend(["Anode_uA", "Transition_mA", "Collector_mA", "High Voltage_mA", "Temperature 1 (High)", "Temperature 2 (Low)"])
        self.logwriter = csv.DictWriter(self.logf, fieldnames=self.logf_format)
        self.logwriter.writeheader()
        # Print a message indicating the file creation
        print(f"File '{path}' created.")

    def start_monitoring(self):
        """ Initiate voltage and current monitoring every 1000ms (=1s).
            Then, initiate a list of the most recent values."""
        self.timer.start(1000)
        for widget in self.readonly_current_widgets.values():
            widget.setStyleSheet("background-color: lightgreen;")
        for widget in self.readonly_voltage_widgets.values():
            widget.setStyleSheet("background-color: lightgreen;")
        self.start_monitoring_button.setEnabled(False)
        self.stop_monitoring_button.setEnabled(True)

        for key, value in self.current_recent_values.items():
            self.current_recent_values[key] = np.zeros(10)
        for key, value in self.v_recent_values.items():
            self.v_recent_values[key] = np.zeros(10)

    def stop_monitoring(self):
        """ Stop voltage and current monitoring."""
        self.timer.stop()
        for widget in self.readonly_current_widgets.values():
            widget.setStyleSheet("background-color: lightgrey;")
        for widget in self.readonly_voltage_widgets.values():
            widget.setStyleSheet("background-color: lightgrey;")
        self.start_monitoring_button.setEnabled(True)
        self.stop_monitoring_button.setEnabled(False)

    def create_horizontal_group_box(self):
        """Creates GUI buttons to control static voltages and one-time ramps (i.e., not the timing loops)."""
        self._horizontalGroupBox = QGroupBox()
        layout = QHBoxLayout()

        self.v_increment_spinbox = QDoubleSpinBox(value=5)
        self.v_increment_spinbox.setMinimum(.01)
        self.v_increment_spinbox.setMaximum(10000)
        self.v_increment_spinbox.valueChanged.connect(self.update_increment)

        self.stop_ramp_button = QPushButton("Stop Voltage Ramp")
        self.stop_ramp_button.clicked.connect(self.stop_voltage_ramp)

        self.update_all_button = QPushButton("Apply all Voltages")
        self.update_all_button.clicked.connect(self.update_all_voltages)

        self.zero_all_button = QPushButton("Zero all Voltages/Stop Ramp")
        self.zero_all_button.clicked.connect(self.zero_all_voltages)

        self.start_monitoring_button = QPushButton("Start Monitoring")
        self.start_monitoring_button.clicked.connect(self.start_monitoring)

        self.stop_monitoring_button = QPushButton("Stop Monitoring")
        self.stop_monitoring_button.clicked.connect(self.stop_monitoring)
        self.stop_monitoring_button.setEnabled(False)
        layout.addWidget(QLabel("V Increment:"))
        layout.addWidget(self.v_increment_spinbox)
        layout.addWidget(self.update_all_button)
        layout.addWidget(self.zero_all_button)
        layout.addWidget(self.stop_ramp_button)
        layout.addWidget(self.start_monitoring_button)
        layout.addWidget(self.stop_monitoring_button)
        self._horizontalGroupBox.setLayout(layout)

    def update_increment(self):
        """Called automatically whenever the V increment spinbox is changed. Changes the step size of the up/down arrows
        of the voltage spinboxes."""
        for widget in self.input_widgets.values():
            widget["field"].setSingleStep(float(self.v_increment_spinbox.value()))
            widget["ramp"].setSingleStep(float(self.v_increment_spinbox.value()))

    def stop_voltage_ramp(self):
        try:
            self.ebit_controller.ramp_done_callback(0, 0, 0)
        except:
            print("Voltage ramp not in progress.")


    def create_menu(self):
        """Creates the File-> Save/Load toolbar at the very top of the GUI."""
        self._menu_bar = QMenuBar()

        self._file_menu = QMenu("&File", self)
        self._save_preset_action = self._file_menu.addAction("S&ave Preset")
        self._load_preset_action = self._file_menu.addAction("L&oad Preset")

        self._menu_bar.addMenu(self._file_menu)
        self._load_preset_action.triggered.connect(self.load_voltages_from_file)
        self._save_preset_action.triggered.connect(self.save_voltages_to_file)

    def load_voltages_from_file(self):
        """Loads a .csv file containing static voltages and ramp values for each component"""
        dir = os.path.dirname(os.path.realpath(__file__))
        fileName, _ = QFileDialog.getOpenFileName(
            self, "Find voltage preset .csv file", dir,
            "CSV Files (*.csv);;All Files (*)")  # , options=options)
        if not fileName:
            return 0

        header = {"element", "voltage", "ramp"}
        preset_csv = pd.read_csv(fileName)

        for col in preset_csv.columns:
            if pd.isna(preset_csv[col][1]):
                preset_csv = preset_csv.drop(columns=col)

        preset_dicts = preset_csv.transpose().to_dict() #gives a dict of dicts, like {0: {'element':'Suppressor', 'voltage':0, ...}, {...}, {...} }
        for d in preset_dicts.values():
            try:
                widget = self.input_widgets[d["element"]]
                widget["field"].setValue(float(d["voltage"]))
                widget["ramp"].setValue(float(d["ramp"]))
            except Exception as e:
                print(e)
                widget = self.input_widgets[d["element"]]
                widget["field"].setValue(0.0)
                widget["ramp"].setValue(0.0)

        print("Loaded voltage presets from", fileName)

    def save_voltages_to_file(self):
        """Creates a .csv file containing static voltages and ramp values for each component"""
        now = datetime.datetime.now()
        # Format the date and time as a string
        formatted_date_time = now.strftime("%Y-%m-%d_%Hh%Mm%Ss")
        file_name = f"ebit_preset_{formatted_date_time}.csv"
        formatted_date_time = now.strftime("%Y-%m-%d_%Hh%Mm%Ss")
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "voltage presets", file_name)
        # Create and open the file in write mode
        with open(path, 'a', newline='') as presetsF:
            preset_dict_header = {"element", "voltage", "ramp"}
            presetwriter = csv.DictWriter(presetsF, fieldnames=preset_dict_header)
            presetwriter.writeheader()
            for i, (k, v) in enumerate(self.input_widgets.items()):
                preset_dict = {} #one row/element to be saved
                preset_dict["element"] = k
                preset_dict["voltage"] = float(v["field"].value())
                preset_dict["ramp"] = float(v["ramp"].value())
                presetwriter.writerow(preset_dict)
        print("Presets saved to",path)

    def create_grid_group_timingloop_box(self):
        """Creates GUI elements for the timing cycle."""
        self._grid_group_timingloop_box = QGroupBox("Timing Loop Control")

        layout = QGridLayout()
        # Top row of labels.
        layout.addWidget(QLabel("Element"), 1, 0)
        layout.addWidget(QLabel("Low Voltage"), 1, 1)
        layout.addWidget(QLabel("High Voltage"), 1, 2)
        layout.addWidget(QLabel("Ramp Duration (ms)"), 1, 3)
        layout.addWidget(QLabel("Ramp Width (ms)"), 1, 4)
        layout.addWidget(QLabel("Delay (ms)"), 1, 5)
        layout.addWidget(QLabel("Enable"), 1, 6)

        self.timing_components = {"Drifttube2":["0","300","0","5","1", "1"],
                                  "Drifttube3": ["0","450","20","200","200", "0"],
                                  "High Voltage":["4000","9700","0","7","1", "1"],
                                  "AO: MeVVA Trigger": ["0","1","0","4","1.992", "1"],
                                  "DO: Trigger 0": ["0","1","0","1","0", "0"],
                                  "DO: Trigger 1": ["0","1","0","1","0", "0"],
                                  "AO: Trigger 1": ["0","1","0","1","0", "0"],
                                  "AO: Trigger 2": ["0","1","0","1","0", "0"],
                                  "AO: Trigger 3": ["0", "1", "0", "1", "0", "0"],
                                  "AO: Trigger 4": ["0", "1", "0", "1", "0", "0"],
                                  "Anode": ["0", "450", "1", "5", "0", "0"]
                                  }
        ### timing_components is a dict with each time-able component and its corresponding default values
        # "Drifttube2" : [Low voltage = 0 V, High voltage = 5 V, ramp duration = 1 ms, ramp width = 5 ms, delay = 0 ms, enabled by default = 1]
        rowColors = ['white', 'lightsteelblue']
        for i, (component_name, defaults) in enumerate(self.timing_components.items()):
            label = QLabel(component_name)
            label.setStyleSheet(f"background-color: {rowColors[i%2]};")
            layout.addWidget(label, i+2, 0)

            low_voltage = QLineEdit(defaults[0])
            low_voltage.setStyleSheet(f"background-color: {rowColors[i%2]};")
            layout.addWidget(low_voltage, i+2, 1)

            high_voltage = QLineEdit(defaults[1])
            high_voltage.setStyleSheet(f"background-color: {rowColors[i%2]};")
            layout.addWidget(high_voltage, i+2, 2)

            ramp_duration = QLineEdit(defaults[2])
            ramp_duration.setStyleSheet(f"background-color: {rowColors[i%2]};")
            layout.addWidget(ramp_duration, i+2, 3)

            ramp_width = QLineEdit(defaults[3])
            ramp_width.setStyleSheet(f"background-color: {rowColors[i%2]};")
            layout.addWidget(ramp_width, i+2, 4)

            delay_duration = QLineEdit(defaults[4])
            delay_duration.setStyleSheet(f"background-color: {rowColors[i%2]};")
            layout.addWidget(delay_duration, i+2, 5)

            check_box = QCheckBox()
            check_box.setChecked(int(defaults[5]))
            layout.addWidget(check_box, i+2, 6)

            disable_colors = ["lightgrey", "grey"]
            if "O:" in component_name:
                output_type = "DO" #digital output
                low_voltage.setEnabled(False)
                low_voltage.setStyleSheet(f"background-color: {disable_colors[i%2]};")
                high_voltage.setEnabled(False)
                high_voltage.setStyleSheet(f"background-color: {disable_colors[i%2]};")
                ramp_duration.setEnabled(False)
                ramp_duration.setStyleSheet(f"background-color: {disable_colors[i%2]};")
            else:
                output_type = "AO" #analog output

            self.timing_loop_widgets[component_name] = {"low_voltage": low_voltage,
                                                        "high_voltage": high_voltage,
                                                        "ramp_duration": ramp_duration,
                                                        "ramp_width": ramp_width,
                                                        "delay_duration": delay_duration,
                                                        "checkbox" : check_box,
                                                        "output":output_type}

        # layout.setColumnStretch(1, 10)
        # layout.setColumnStretch(2, 20)
        start_stop_box = QGroupBox()
        start_stop_layout = QVBoxLayout()
        start_button = QPushButton("Start")
        start_button.clicked.connect(self.start_timing_loop)
        start_stop_layout.addWidget(start_button)

        stop_button = QPushButton("Stop")
        stop_button.clicked.connect(self.stop_timing_loop)
        stop_button.setEnabled(False)
        start_stop_layout.addWidget(stop_button)
        self.timing_loop_start_stop_widgets = {"start_button":start_button, "stop_button":stop_button}
        start_stop_box.setLayout(start_stop_layout)
        layout.addWidget(start_stop_box, 2, 7, 3, 1)

        load_clear_box = QGroupBox()
        load_clear_layout = QVBoxLayout()
        load_button = QPushButton("Load")
        load_button.clicked.connect(self.handle_choose_file_for_custom_timing)
        load_clear_layout.addWidget(load_button)

        clear_button = QPushButton("Clear")
        clear_button.clicked.connect(self.clear_custom_timing_loop)
        clear_button.setEnabled(False)
        load_clear_layout.addWidget(clear_button)
        self.timing_loop_load_clear_widgets = {"load_button":load_button, "clear_button":clear_button}
        load_clear_box.setLayout(load_clear_layout)
        layout.addWidget(load_clear_box, 5, 7, 3, 1)

        timing_group_box = QGroupBox()
        timing_layout = QGridLayout()

        timing_layout.addWidget(QLabel("Cycle Time (s)"), 1,0)
        timing_cycle_time_box = QLineEdit("1")
        timing_layout.addWidget(timing_cycle_time_box, 2,0)

        timing_cycle_button = QPushButton("Apply")
        timing_cycle_button.clicked.connect(self.update_timing_cycle_value)
        self._timing_cycle_time_s = 1
        timing_layout.addWidget(timing_cycle_button, 3,0)

        timing_redraw_button = QPushButton("Update plot")
        timing_redraw_button.clicked.connect(self.update_timing_cycle_plot)
        #timing_layout.addWidget(timing_redraw_button, 4,0)

        self.timing_voltage_plot = pg.PlotWidget()
        self.timing_voltage_plot.addLegend(offset=[-1,1], verSpacing=-5, horSpacing=20)
        timing_layout.addWidget(self.timing_voltage_plot, 1,1, -1, -1)
        timing_layout.setColumnStretch(1, 2)
        timing_layout.setRowStretch(1, 2)
        timing_layout.setRowMinimumHeight(1, 120)

        self.update_timing_cycle_plot()

        self.timing_cycle_widgets = {"timing_cycle_time_box":timing_cycle_time_box, "timing_cycle_button":timing_cycle_button, "timing_redraw_button":timing_redraw_button}
            #timing_voltage_plot.plot(this_data)
        timing_layout.setColumnStretch(1,1)
        timing_group_box.setLayout(timing_layout)
        layout.addWidget(timing_group_box, i+3, 0, -1, -1)
        self._grid_group_timingloop_box.setLayout(layout)

    def update_timing_cycle_value(self):
        """Sets the total cycle time to what is in the text box and redraws the timing plot."""
        old_timing_value_s = self._timing_cycle_time_s
        widgets = self.timing_cycle_widgets
        try:
            new_timing_value_s = float(widgets["timing_cycle_time_box"].text())
            self._timing_cycle_time_s = new_timing_value_s
            try:
                if not self.update_timing_cycle_plot():
                    print("Error updating plot.")
            except Exception as e:
                print("Error updating plot:", e)
        except Exception:
            print("error setting new timing value")

    def update_timing_cycle_plot(self):
        """Redraws the timing plot at the bottom of the screen."""
        self.timing_voltage_plot.clear()
        legend = self.timing_voltage_plot.addLegend(offset=[-1,-1], verSpacing=-5, horSpacing=20)

        frequency = 200000 #the NI card runs at 200,000 Hz
        total_pts = int(frequency * self._timing_cycle_time_s)

        def butter_lowpass(cutoff, fs, order=1):
            return butter(order, cutoff, fs=fs, btype='low', analog=False)

        def butter_lowpass_filter(data, cutoff, fs, order=1):
            b, a = butter_lowpass(cutoff, fs, order=order)
            y = lfilter(b, a, data)
            return y
        numPlotted = 0 #used to resize the legend
        for i, component_name in enumerate(self.timing_components.keys()):
            widgets = self.timing_loop_widgets[component_name]
            if widgets["checkbox"].isChecked():
                ramp_duration =   float(widgets["ramp_duration"].text())/1000 #seconds
                ramp_width =            float(widgets["ramp_width"].text())/1000 #seconds
                delay_duration =        float(widgets["delay_duration"].text())/1000 #seconds
                low_voltage =           float(widgets["low_voltage"].text())
                high_voltage =          float(widgets["high_voltage"].text())
                cycle_time =            float(self._timing_cycle_time_s)
                numPlotted += 1
                if cycle_time < (2*ramp_duration + ramp_width + delay_duration):
                    msg = QMessageBox()
                    msg.setIcon(QMessageBox.Icon.Warning)
                    msg.setText(f"Timing sequence of {component_name} exceeds the {cycle_time*1000} ms cycle time ({2*ramp_duration + ramp_width + delay_duration} ms).")
                    msg.exec()
                    self._timing_cycle_time_s = 1
                    return 0
                num_ramp_steps = int(frequency * ramp_duration)
                num_width_steps = int(frequency * ramp_width)
                delay_steps = int(delay_duration * frequency)
                ramp_up_array = np.linspace(low_voltage, high_voltage, num=num_ramp_steps)
                ramp_width_array = [high_voltage] * num_width_steps
                ramp_down_array = np.linspace(high_voltage, low_voltage, num=num_ramp_steps)
                delay_array = [low_voltage] * delay_steps

                full_command = []
                full_command.extend(delay_array)
                full_command.extend(ramp_up_array)
                full_command.extend(ramp_width_array)
                full_command.extend(ramp_down_array)
                full_command.extend([low_voltage] * (total_pts - len(full_command)))

                if "O:" not in component_name and self.enable_low_pass_filter:
                    ### Low-pass filter
                    # Filter requirements.
                    order = 3
                    fs = frequency  # sample rate, Hz
                    # slew rate of the trek is 350 V/us
                    cutoff = 1000  # fs/2-1  # desired cutoff frequency of the filter, Hz

                    T = cycle_time  # seconds
                    n = total_pts  # total number of samples
                    t = np.linspace(0, T, n, endpoint=False)

                    full_command_baseline = full_command[0]
                    data = np.array(full_command) - full_command_baseline  # sets the first value to 0. the filter misbehaves otherwise.

                    y = butter_lowpass_filter(data, cutoff, fs, order)
                    y = y + full_command_baseline
                    full_command = y.tolist()
                v_range = (max(full_command)-min(full_command)) if ((max(full_command)-min(full_command))!=0.0) else max(full_command) #if there is only one voltage for the loop, we want to avoid a divide by 0 error
                reduced_ramp_voltages = (np.array(full_command) - min(full_command)) / v_range
                xs = np.linspace(0, int(self._timing_cycle_time_s*1000), num=int(self._timing_cycle_time_s*frequency), endpoint=False)
                self.timing_voltage_plot.plot(xs, reduced_ramp_voltages-1.5*numPlotted, name = component_name, pen={'color':pg.intColor(i), 'width':2})
        #legend.setScale(1.5-(0.07*numPlotted))
        legend.setColumnCount(int(np.ma.ceil(numPlotted/6)))
        self.timing_voltage_plot.setXRange(0, int(self._timing_cycle_time_s*1000))
        self.timing_voltage_plot.setYRange(-1.5*(numPlotted)-0.5, 1.1)
        self.timing_voltage_plot.setLimits(xMin=-1, xMax=int(self._timing_cycle_time_s*1000))
        self.timing_voltage_plot.setMouseEnabled(y=False)
        return(1)

    def update_timing_cycle_plot_custom(self):
        """Plots timing sequences loaded from a .csv file."""
        self.timing_voltage_plot.clear()
        legend = self.timing_voltage_plot.addLegend(offset=[-1,-1], verSpacing=-5, horSpacing=20)
        cycle_time = self.custom_timing_plan["time_s"]

        frequency = 200000#1000 #each point represents one ms for plotting purposes only
        total_pts = int(frequency * cycle_time)

        def butter_lowpass(cutoff, fs, order=1):
            # I use a first-order filter so the filter's time delay is constant for all frequencies
            return butter(order, cutoff, fs=fs, btype='low', analog=False)

        def butter_lowpass_filter(data, cutoff, fs, order=1):
            b, a = butter_lowpass(cutoff, fs, order=order)
            y = lfilter(b, a, data)
            return y

        numPlotted = 0  # used to resize the legend

        for i, (component_name, values)  in enumerate(self.custom_timing_plan.items()):
            if component_name == "time_s":
                continue
            if cycle_time*1000 < (max(values[0])):
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Icon.Warning)
                msg.setText(f"Timing sequence of {component_name} exceeds the {cycle_time*1000} ms cycle time ({max(values[0])} ms).")
                msg.exec()
                return 0
            full_command = []
            if len(values[0])>1:
                times = np.array(values[0])*frequency/1000 #converts from ms to sample number
                voltages = values[1]
            else: #if there is only one voltage setting
                times = values[0]
                voltages = values[1]
            if "O:" not in component_name: #Analog out signals only
                for j in range(len(times)): #[time, voltage] pairs
                    if j != (len(times)-1): #if this isn't the last step in the loop...
                        vs = np.linspace(start=voltages[j], stop=voltages[j+1], num=int(times[j+1]-times[j]))
                        dt = (times[j+1] - times[j])/frequency*1000 #converts back from sample number to ms
                        if dt<=0.0 or not len(vs):
                            msg = QMessageBox()
                            msg.setIcon(QMessageBox.Icon.Warning)
                            msg.setText(
                                f"There are repeated times in the timing sequence. Times must increment at each step.")
                            self.clear_custom_timing_loop()
                            self.update_timing_cycle_plot()
                            msg.exec()
                            return 0
                        dv = abs((vs[0] - vs[-1]))
                        if dv/dt >= self.trek_slew_rate and component_name == "High Voltage":
                            msg = QMessageBox()
                            msg.setIcon(QMessageBox.Icon.Warning)
                            msg.setText(
                                f"High Voltage slew rate of {dv/dt} V/ms exceeds the maximum of {self.trek_slew_rate} V/ms.")
                            self.clear_custom_timing_loop()
                            self.update_timing_cycle_plot()
                            msg.exec()
                            return 0
                    else:
                        vs = np.linspace(start=voltages[j], stop=voltages[0], num=int(total_pts-len(full_command)))
                    full_command.extend(vs)
                if self.enable_low_pass_filter:
                    ### Low-pass filter
                    # Filter requirements.
                    order = 3
                    fs = frequency  # sample rate, Hz
                    # slew rate of the trek is 350 V/us, or 350,000 V/ms
                    cutoff = 1000   # desired cutoff frequency of the filter, Hz. 1000 is arbitrary.
                    full_command_baseline = full_command[0]
                    data = np.array(full_command) - full_command_baseline  # sets the first value to 0. the filter misbehaves otherwise.
                    y = butter_lowpass_filter(data, cutoff, fs, order)
                    y = y + full_command_baseline
                    full_command = y.tolist()
            else:
                for j in range(len(times)):  # [time, voltage] pairs
                    if j != (len(times) - 1):  # if this isn't the last step in the loop...
                        vs = [int(voltages[j])] *int(times[j+1]-times[j])
                    else:
                        vs = [int(voltages[j])] *int(total_pts-len(full_command))
                    full_command.extend(vs)

            numPlotted += 1
            reduced_ramp_voltages = (np.array(full_command) - min(full_command)) / (max(full_command)-min(full_command))
            xs = np.linspace(0, int(cycle_time*1000), num=total_pts, endpoint=False)
            self.timing_voltage_plot.plot(xs, reduced_ramp_voltages - 1.5 * numPlotted, name=component_name, pen={'color':pg.intColor(i), 'width':2})
        legend.setColumnCount(int(np.ma.ceil(numPlotted / 6)))
        self.timing_voltage_plot.setXRange(0, cycle_time*1000)
        self.timing_voltage_plot.setYRange(-1.5*(numPlotted), 1.1)
        self.timing_voltage_plot.setLimits(xMin=-1, xMax=cycle_time*1000)
        self.timing_voltage_plot.setMouseEnabled(y=False)
        self.timing_cycle_widgets["timing_cycle_time_box"].setText(str(self.custom_timing_plan["time_s"]))
        return 1

    def start_timing_loop(self):
        """Sends timing information entered into the GUI timing loop boxes to EbitVoltageController, where the voltages
        are scaled and converted to the PCIe clock frequency."""
        if self.ebit_controller.ramp_task is not None:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Icon.Information)
            msg.setText(
                "Caution: A voltage ramp is already in progress. To cancel the in-progress ramp, use the Stop Voltage Ramp button.")
            msg.exec()
            return -1
        AO_plans = []
        AO_trigger_plans = []
        DO_plans = []

        for component_name, widgets in self.timing_loop_widgets.items():
            #check if enabled
            if widgets["checkbox"].isChecked():
                #if enabled, get ramp plan
                component_ramp_plan = {"component_name": component_name,
                                        "ramp_duration":float(widgets["ramp_duration"].text()) / 1000,
                                        "ramp_width":float(widgets["ramp_width"].text()) / 1000,
                                        "delay_duration":float(widgets["delay_duration"].text()) / 1000,
                                        "low_voltage":float(widgets["low_voltage"].text()),
                                        "high_voltage":float(widgets["high_voltage"].text())}
                #ensure ramp isn't longer than cycle time
                ramp_plan_duration = 2*component_ramp_plan["ramp_duration"] + component_ramp_plan["ramp_width"]+ component_ramp_plan["delay_duration"]
                if self._timing_cycle_time_s < ramp_plan_duration:
                    msg = QMessageBox()
                    msg.setIcon(QMessageBox.Icon.Warning)
                    msg.setText(f"Timing sequence of {component_name} exceeds the {self._timing_cycle_time_s} s cycle time ({ramp_plan_duration} s).")
                    msg.exec()
                    return -1
                if "DO:" in component_name:
                    DO_plans.append(component_ramp_plan)
                elif "AO:" in component_name:
                    AO_trigger_plans.append(component_ramp_plan)
                else:
                    AO_plans.append(component_ramp_plan)

        # Disable buttons after we know all plans are not too long
        for component_name, widgets in self.timing_loop_widgets.items():
                        # check if enabled
            if widgets["checkbox"].isChecked():
                if "O:" not in component_name:
                    self.input_widgets[component_name]["button"].setEnabled(False)
                    self.input_widgets[component_name]["field"].setEnabled(False)
                    self.input_widgets[component_name]["ramp"].setEnabled(False)

                widgets["ramp_duration"].setStyleSheet("background-color: lightgreen;")
                widgets["ramp_duration"].setReadOnly(True)
                widgets["ramp_width"].setStyleSheet("background-color: lightgreen;")
                widgets["ramp_width"].setReadOnly(True)
                widgets["delay_duration"].setStyleSheet("background-color: lightgreen;")
                widgets["delay_duration"].setReadOnly(True)
                widgets["low_voltage"].setStyleSheet("background-color: lightgreen;")
                widgets["low_voltage"].setReadOnly(True)
                widgets["high_voltage"].setStyleSheet("background-color: lightgreen;")
                widgets["high_voltage"].setReadOnly(True)
                widgets["checkbox"].setEnabled(False)

        if (len(AO_plans)+len(DO_plans)+len(AO_trigger_plans)) > 0:
            self.update_timing_cycle_plot()
            
            pr = cProfile.Profile()

            pr.runcall(self.ebit_controller.start_timing_loop, AO_plans, AO_trigger_plans, DO_plans, self._timing_cycle_time_s, enable_lpf=self.enable_low_pass_filter)

            pr.dump_stats('stats.txt')

            #self.ebit_controller.start_timing_loop(AO_plans, AO_trigger_plans, DO_plans, self._timing_cycle_time_s, enable_lpf=self.enable_low_pass_filter)
            self.timing_loop_start_stop_widgets["start_button"].setEnabled(False)
            self.timing_loop_start_stop_widgets["stop_button"].setEnabled(True)
            self.timing_cycle_widgets["timing_cycle_time_box"].setEnabled(False)
            self.timing_cycle_widgets["timing_cycle_time_box"].setText(str(self._timing_cycle_time_s))
            self.timing_cycle_widgets["timing_cycle_button"].setEnabled(False)

            self.timing_loop_load_clear_widgets["clear_button"].setEnabled(False)
            self.timing_loop_load_clear_widgets["load_button"].setEnabled(False)
        else:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Icon.Information)
            msg.setText("Use the checkboxes to enable at least one element for the timing loop.")
            msg.exec()
            return -1

    def stop_timing_loop(self):
        self.ebit_controller.stop_timing_loop()
        rowColors = ['white', 'lightsteelblue']
        disableColors = ['lightgrey', 'grey']
        for i, (component_name, widgets) in enumerate(self.timing_loop_widgets.items()):
            if "O:" not in component_name:
                self.input_widgets[component_name]["button"].setEnabled(True)
                self.input_widgets[component_name]["field"].setEnabled(True)
                self.input_widgets[component_name]["ramp"].setEnabled(True)

            widgets["ramp_duration"].setStyleSheet(f"background-color: {rowColors[i%2]};")
            widgets["ramp_duration"].setReadOnly(False)
            widgets["ramp_width"].setStyleSheet(f"background-color: {rowColors[i%2]};")
            widgets["ramp_width"].setReadOnly(False)
            widgets["delay_duration"].setStyleSheet(f"background-color: {rowColors[i%2]};")
            widgets["delay_duration"].setReadOnly(False)
            widgets["low_voltage"].setStyleSheet(f"background-color: {rowColors[i%2]};")
            widgets["low_voltage"].setReadOnly(False)
            widgets["high_voltage"].setStyleSheet(f"background-color: {rowColors[i%2]};")
            widgets["high_voltage"].setReadOnly(False)
            widgets["checkbox"].setEnabled(True)

            if "O:" in component_name:
                widgets["low_voltage"].setStyleSheet(f"background-color: {disableColors[i % 2]}")
                widgets["high_voltage"].setStyleSheet(f"background-color: {disableColors[i % 2]}")
                widgets["ramp_duration"].setStyleSheet(f"background-color: {disableColors[i % 2]}")

        self.timing_loop_start_stop_widgets["start_button"].setEnabled(True)
        self.timing_loop_start_stop_widgets["stop_button"].setEnabled(False)
        self.timing_cycle_widgets["timing_cycle_time_box"].setEnabled(True)
        self.timing_cycle_widgets["timing_cycle_button"].setEnabled(True)
        self.timing_loop_load_clear_widgets["clear_button"].setEnabled(False)
        self.timing_loop_load_clear_widgets["load_button"].setEnabled(True)

    def create_grid_group_setvoltage_box(self):
        """Create GUI elements to handle static voltages and ramps."""
        self._grid_group_setvoltage_box = QGroupBox("Manual Voltage Control")
        layout = QGridLayout()

        # Top row of labels.
        layout.addWidget(QLabel("Element"), 1, 0)
        layout.addWidget(QLabel("Output Voltage (Volts)"), 1, 1)
        layout.addWidget(QLabel("Ramp (V/s)"), 1, 3)
        layout.addWidget(QLabel("Input Voltage (Volts)"), 1, 4)
        layout.addWidget(QLabel("Input Voltage vs Time"), 1, 5)
        layout.addWidget(QLabel("Input Current (mA)"), 1, 6)
        layout.addWidget(QLabel("Input Current vs Time"), 1, 7)

        for i, (k, v) in enumerate(self.ebit_data_model.components.items()):
            if "DO:" in k or "AI:" in k or "AO:" in k: #don't do this for digital output or analog input entries in io_config.csv
                continue
            label = QLabel(k)
            layout.addWidget(label, i + 2, 0)
            if v.output_card:
                self.input_widgets[k] = {"button": QPushButton("Apply"),
                                         "field": QDoubleSpinBox(value = v.output_default_value),
                                         "ramp": QDoubleSpinBox()}
                self.input_widgets[k]["field"].setRange(-999999, 999999)
                self.input_widgets[k]["field"].setSingleStep(5)
                self.input_widgets[k]["ramp"].setRange(0, 999999)
                self.input_widgets[k]["ramp"].setSingleStep(5)



                self.input_widgets[k]["button"].clicked.connect(partial(self.update_voltage, k))
                layout.addWidget(self.input_widgets[k]["field"], i + 2, 1)
                layout.addWidget(self.input_widgets[k]["button"], i + 2, 2)
                layout.addWidget(self.input_widgets[k]["ramp"], i + 2, 3)
                if v.output_card == "pcie-6738" and v.output_pin == "AO0":
                    self.input_widgets[k]["button"].setEnabled(False)
                    self.input_widgets[k]["field"].setReadOnly(True)
                    self.input_widgets[k]["ramp"].setReadOnly(True)
                    self.input_widgets[k]["field"].setStyleSheet("background-color: red;")
                    self.input_widgets[k]["ramp"].setStyleSheet("background-color: red;")
                    self.input_widgets[k]["field"].setText("A0 not allowed")

            if v.input_voltage_card:
                input_voltage_editor = QLineEdit(str(v.input_voltage_value))
                input_voltage_editor.setReadOnly(True)
                input_voltage_editor.setStyleSheet("background-color: lightgrey;")
                layout.addWidget(input_voltage_editor, i + 2, 4)
                self.readonly_voltage_widgets[k] = input_voltage_editor

                if self.enable_v_plots:
                    ### Voltage plots
                    input_v_plot = pg.PlotWidget(name=f"input_v_plot_{k}")
                    if k != 'Suppressor':  ### Link all X axes together so that the times are always synced between plots
                        input_v_plot.setXLink(self.v_plot_widgets['Suppressor'])

                    input_v_plot.enableAutoRange()
                    input_v_plot.showAxes((1,0,0,1), (1,0,0,0))

                    input_v_plot.setMouseEnabled(y=False, x=False)
                    #input_v_plot.hideAxis("bottom")
                    layout.addWidget(input_v_plot, i + 2, 5)
                    self.v_plot_widgets[k] = input_v_plot
                self.v_recent_values[k] = np.zeros(10)

            if v.input_current_card:
                input_current_editor = QLineEdit(str(v.input_current_value))
                input_current_editor.setReadOnly(True)
                input_current_editor.setStyleSheet("background-color: lightgrey;")
                layout.addWidget(input_current_editor, i + 2, 6)
                self.readonly_current_widgets[k] = input_current_editor

                if self.enable_current_plots:
                    ### Current plots
                    input_current_plot = pg.PlotWidget(name=f"input_current_plot_{k}")
                    input_current_plot.setMouseEnabled(y=False, x=False)
                    input_current_plot.showAxes((1,0,0,1), (1,0,0,0))
                    if k != 'Anode': ### Link all X axes together so that the times are always synced between plots
                        input_current_plot.setXLink(self.current_plot_widgets['Anode'])
                    layout.addWidget(input_current_plot, i + 2, 7)
                    self.current_plot_widgets[k] = input_current_plot
                self.current_recent_values[k] = np.zeros(10)
        layout.setColumnStretch(i+2,1)
        self._grid_group_setvoltage_box.setLayout(layout)
        #print(self.current_plot_widgets)

    def zero_all_voltages(self):
        self.timing_loop_start_stop_widgets["stop_button"].click()
        self.ebit_controller.clear_all_voltages()
        for key, value in self.input_widgets.items():
            value["field"].setValue(0.0)

    def update_voltage(self, name):
        """Set a single static voltage or initiate a ramp."""
        ret = self.ebit_controller.set_voltage(name, self.input_widgets[name]["field"].value(),
                                               ramp_vs= float(self.input_widgets[name]["ramp"].value()))
        #ret is [1] or [0, new_value, model.output_max_volts, start_V (voltage before setting the new value)]
        if not ret[0]: #if the set voltage is too high or too low:
            #make a message box with the error
            msg = QMessageBox(self)
            msg.setModal(False)
            msg.setIcon(QMessageBox.Icon.Critical)
            msg.setText(f"Caution: Voltage {ret[1]} exceeds {name} voltage limit of ({ret[2]}).")
            msg.setInformativeText("The voltage was not changed.")
            msg.show()
            #set the text box value back to what it was
            self.input_widgets[name]["field"].setValue(ret[3])
        elif ret[0]==2: #if there is already a ramp in progress
            msg = QMessageBox(self)
            msg.setModal(False)
            msg.setIcon(QMessageBox.Icon.Critical)
            msg.setText(f"Caution: A voltage ramp is already in progress. To cancel the in-progress ramp, use the Cancel Voltage Ramp button.")
            msg.setInformativeText("The voltage was not changed.")
            msg.show()
            self.input_widgets[name]["field"].setValue(ret[1])
        elif ret[0]==3: #if there is a timing loop in progress and we attempted to start a ramp
            msg = QMessageBox(self)
            msg.setModal(False)
            msg.setIcon(QMessageBox.Icon.Critical)
            msg.setText(f"Caution: A timing loop is already in progress, so a ramp cannot be started.")
            msg.setInformativeText("The voltage was not changed.")
            msg.show()
            self.input_widgets[name]["field"].setValue(ret[1])

    def update_all_voltages(self):
        for key, value in self.input_widgets.items():
            self.update_voltage(key)

    def update_inputs(self):
        """Read in EBIT voltages and currents."""
        self.ebit_controller.read_all_voltages()
        v_log_dict = {"time":datetime.datetime.now().strftime("%H:%M:%S")}
        for key, field in self.readonly_current_widgets.items():
            component = self.ebit_data_model.components[key]
            value = component.input_current_value
            value = round(value, 5) if value is not None else value
            field.setText(str(value))
            v_log_key = next(name for name in self.logf_format if name.startswith(key) and name.endswith("A")) #gets the header name for the log dictionary, to account for different units (e.g., uA or mA).
            v_log_dict[v_log_key] = value

            self.current_recent_values[key] = np.append(self.current_recent_values[key][1:], value)

        for key, field in self.readonly_voltage_widgets.items():
            component = self.ebit_data_model.components[key]
            value = component.input_voltage_value
            value = round(value, 5) if value is not None else value
            v_log_dict[key] = value
            field.setText(str(value))
            self.v_recent_values[key] = np.append(self.v_recent_values[key][1:], value)

            # Check for power threshold.
            if component.power_threshold and component.input_current_value and component.input_voltage_value:
                power = component.input_current_value * component.input_voltage_value
                if power > component.power_threshold:
                    self.readonly_current_widgets[key].setStyleSheet("background-color: red;")
                    self.readonly_voltage_widgets[key].setStyleSheet("background-color: red;")

                    # Set the voltage to zero for safety.
                    #self.timing_loop_start_stop_widgets["stop_button"].click()
                    #self.ebit_controller.stop_timing_loop()
                    self.ebit_controller.set_voltage(key, 0)
                    self.ebit_controller.set_voltage("Transition", 9) #Amy noticed the transition can act strangely when the anode trips #todo: fix transition calibration (io_config.csv). right now, 9V is the lowest allowed voltage.
                    self.input_widgets[key]["field"].setValue(0.0)
                    self.input_widgets["Transition"]["field"].setValue(9.0)

                    msg_text = f"{key} voltage set to zero for power safety!"
                    more_info = f"Threshold of {component.power_threshold} W reached!\nCurrent = {component.input_current_value} mA\nVoltage = {component.input_voltage_value} V"

                    # Send slack message
                    if self.slack_enabled:
                        self.slack.send(text=f"{msg_text}\n{more_info}")

                    # Issue a popup.
                    try:
                        self.msg.close()
                    except:
                        pass
                    self.msg = QMessageBox(self)
                    self.msg.setModal(False)
                    self.msg.setIcon(QMessageBox.Icon.Critical)
                    self.msg.setText(msg_text)
                    self.msg.setInformativeText(more_info)
                    self.msg.show()

                else:
                    self.readonly_current_widgets[key].setStyleSheet("background-color: lightgreen;")
                    self.readonly_voltage_widgets[key].setStyleSheet("background-color: lightgreen;")
        self.update_current_v_plots()
        if not self.log_index%self.log_period_s: #we only want to log every self.log_period_s seconds, % is the modulus operator
            #Manually add the temperature readings to the dict for logging.
            # They aren't here by default because they don't have a text box in the GUI like the components do.
            if self.labjack:
                try:
                    temperature_voltages = self.labjack.read_temperature_voltages()
                except Exception as e:
                    print("Failed to read temperatures:", e)
                    temperature_voltages = [0.0,0.0]
                #We use the lakeshore 218's analog out for temperature monitoring.
                # As of 1/14/2025, Grant set the following scaling settings: analog 1: 0V@0K, 10V@25K. analog 2: 0V@0K, 10V@100K
                Temperature1 = min(temperature_voltages[0]*(100./10.), 100.)
                Temperature2 = min(temperature_voltages[1]*(25./10.), 100.)
                v_log_dict["Temperature 1 (High)"] = round(Temperature1, 5) if Temperature1 is not None else Temperature1
                v_log_dict["Temperature 2 (Low)"] = round(Temperature2, 5) if Temperature2 is not None else Temperature2

            try:
                #Manually add static drift tube voltages based on what we have inputted.
                # NOTE: This cannot track ramps or timing cycles!
                dt1 = self.ebit_data_model.components["Drifttube1"].output_voltage_value
                dt2 = self.ebit_data_model.components["Drifttube2"].output_voltage_value
                dt3 = self.ebit_data_model.components["Drifttube3"].output_voltage_value

                v_log_dict["Drifttube1"] = round(dt1, 5) if dt1 is not None else dt1
                v_log_dict["Drifttube2"] = round(dt2, 5) if dt2 is not None else dt2
                v_log_dict["Drifttube3"] = round(dt3, 5) if dt3 is not None else dt3
            except:
                print("Failed to get drift tube voltages.")

            self.logwriter.writerow(v_log_dict)
        self.log_index += 1

    def update_current_v_plots(self):
        #Replot voltage and current plots with the newest values
        self.clear_current_v_plots()
        for j, (key,vplot) in enumerate(self.v_plot_widgets.items()):
            v_vals = self.v_recent_values[key]
            v_vals = np.array([v if v is not None else 0 for v in v_vals]) #set None values to 0 for plotting
            vplot.plot(v_vals, pen=pg.intColor(j))
            if key in self.current_plot_widgets:
                i_vals = self.current_recent_values[key]
                self.current_plot_widgets[key].plot(i_vals, pen=pg.intColor(j))


    def clear_current_v_plots(self):
        for key, plot in self.current_plot_widgets.items():
            plot.clear()
        for key, plot in self.v_plot_widgets.items():
            plot.clear()

    def handle_choose_file_for_custom_timing(self):
        #opens a popup window to select a custom timing .csv file
        dir = os.path.dirname(os.path.realpath(__file__))
        fileName, _ = QFileDialog.getOpenFileName(
            self, "Find timing .csv file", dir,
            "CSV Files (*.csv);;All Files (*)")  # , options=options)
        if not fileName:
            return 0
        timing_csv = pd.read_csv(fileName)
        cycle_time_s = timing_csv["Cycle Time (s)"][0]

        for col in timing_csv.columns:
            if pd.isna(timing_csv[col][1]):
                timing_csv = timing_csv.drop(columns=col)

        time_suffix = " Time (ms)"

        self.custom_timing_plan = {"time_s":cycle_time_s}
        for component_name in self.timing_components.keys():
            if component_name not in timing_csv.columns:
                continue
            times_ms = timing_csv[component_name + time_suffix]
            times_ms = [t for t in times_ms if not pd.isna(t)] #remove NaN values
            voltages = timing_csv[component_name]
            voltages = [v for v in voltages if not pd.isna(v)] #remove NaN values
            self.custom_timing_plan[component_name] = [times_ms, voltages]
        if self.update_timing_cycle_plot_custom():
            if (len(self.custom_timing_plan)) > 1: #Time (ms) is always included
                self.timing_cycle_widgets["timing_cycle_time_box"].setEnabled(False)
                #self.timing_cycle_widgets["timing_cycle_time_box"].setText(str(self._timing_cycle_time_s))
                self.timing_cycle_widgets["timing_cycle_button"].setEnabled(False)
                self.timing_cycle_widgets["timing_redraw_button"].setEnabled(False)

                for component_name, widgets in self.timing_loop_widgets.items():
                    widgets["ramp_duration"].setStyleSheet("background-color: gray;")
                    widgets["ramp_duration"].setReadOnly(True)
                    widgets["ramp_width"].setStyleSheet("background-color: gray;")
                    widgets["ramp_width"].setReadOnly(True)
                    widgets["delay_duration"].setStyleSheet("background-color: gray;")
                    widgets["delay_duration"].setReadOnly(True)
                    widgets["low_voltage"].setStyleSheet("background-color: gray;")
                    widgets["low_voltage"].setReadOnly(True)
                    widgets["high_voltage"].setStyleSheet("background-color: gray;")
                    widgets["high_voltage"].setReadOnly(True)
                    widgets["checkbox"].setEnabled(False)

                start_button = self.timing_loop_start_stop_widgets["start_button"]
                stop_button = self.timing_loop_start_stop_widgets["stop_button"]
                start_button.clicked.disconnect(self.start_timing_loop)
                start_button.clicked.connect(self.start_timing_loop_custom)
                stop_button.clicked.disconnect(self.stop_timing_loop)
                stop_button.clicked.connect(self.stop_timing_loop_custom)

                self.timing_loop_load_clear_widgets["load_button"].setEnabled(False)
                self.timing_loop_load_clear_widgets["clear_button"].setEnabled(True)
            else:
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Icon.Information)
                msg.setText("Enable at least one element for the timing loop.")
                msg.exec()

    def start_timing_loop_custom(self):
        if self.ebit_controller.ramp_task is not None:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Icon.Information)
            msg.setText(
                "Caution: A voltage ramp is already in progress. To cancel the in-progress ramp, use the Cancel Voltage Ramp button.")
            msg.exec()
            return -1
        self.ebit_controller.start_custom_timing_loop(self.custom_timing_plan,
                                                      enable_lpf=self.enable_low_pass_filter)
        for component_name in self.custom_timing_plan.keys():
            if component_name != "time_s" and "O:" not in component_name and "AI:" not in component_name:
                self.input_widgets[component_name]["button"].setEnabled(False)
                self.input_widgets[component_name]["field"].setEnabled(False)
                self.input_widgets[component_name]["ramp"].setEnabled(False)
        self.timing_loop_load_clear_widgets["load_button"].setEnabled(False)
        self.timing_loop_load_clear_widgets["clear_button"].setEnabled(False)
        self.timing_loop_start_stop_widgets["stop_button"].setEnabled(True)
        self.timing_loop_start_stop_widgets["start_button"].setEnabled(False)


    def stop_timing_loop_custom(self):
        self.ebit_controller.stop_timing_loop()
        self.timing_loop_start_stop_widgets["stop_button"].setEnabled(False)
        self.timing_loop_start_stop_widgets["start_button"].setEnabled(True)
        self.timing_loop_load_clear_widgets["load_button"].setEnabled(False)
        self.timing_loop_load_clear_widgets["clear_button"].setEnabled(True)

    def clear_custom_timing_loop(self):
        self.timing_cycle_widgets["timing_cycle_time_box"].setEnabled(True)
        self.timing_cycle_widgets["timing_cycle_time_box"].setText(str(self._timing_cycle_time_s))
        self.timing_cycle_widgets["timing_cycle_button"].setEnabled(True)
        self.timing_cycle_widgets["timing_redraw_button"].setEnabled(True)

        rowColors = ['white', 'lightsteelblue']
        disableColors = ['lightgrey', 'grey']
        for i, (component_name, widgets) in enumerate(self.timing_loop_widgets.items()):

            if "O:" not in component_name: #Analog Out (AO:) and Digital Out (DO:) components are excluded
                self.input_widgets[component_name]["button"].setEnabled(True)
                self.input_widgets[component_name]["field"].setEnabled(True)
                self.input_widgets[component_name]["ramp"].setEnabled(True)

            widgets["ramp_duration"].setStyleSheet(f"background-color: {rowColors[i%2]};")
            widgets["ramp_duration"].setReadOnly(False)
            widgets["ramp_width"].setStyleSheet(f"background-color: {rowColors[i%2]};")
            widgets["ramp_width"].setReadOnly(False)
            widgets["delay_duration"].setStyleSheet(f"background-color: {rowColors[i%2]};")
            widgets["delay_duration"].setReadOnly(False)
            widgets["low_voltage"].setStyleSheet(f"background-color: {rowColors[i%2]};")
            widgets["low_voltage"].setReadOnly(False)
            widgets["high_voltage"].setStyleSheet(f"background-color: {rowColors[i%2]};")
            widgets["high_voltage"].setReadOnly(False)
            widgets["checkbox"].setEnabled(True)

            if "O:" in component_name:
                widgets["low_voltage"].setStyleSheet(f"background-color: {disableColors[i%2]}")
                widgets["high_voltage"].setStyleSheet(f"background-color: {disableColors[i%2]}")
                widgets["ramp_duration"].setStyleSheet(f"background-color: {disableColors[i%2]}")

        self.timing_loop_start_stop_widgets["start_button"].setEnabled(True)
        self.timing_loop_start_stop_widgets["stop_button"].setEnabled(False)
        self.timing_cycle_widgets["timing_cycle_time_box"].setEnabled(True)
        self.timing_cycle_widgets["timing_cycle_button"].setEnabled(True)
        self.timing_cycle_widgets["timing_cycle_button"].click() #reset the plot to the regular timing stuff
        self.timing_cycle_widgets["timing_cycle_button"].click() #have to click it twice for some reason

        self.timing_loop_load_clear_widgets["load_button"].setEnabled(True)
        self.timing_loop_load_clear_widgets["clear_button"].setEnabled(False)

        start_button = self.timing_loop_start_stop_widgets["start_button"]
        stop_button = self.timing_loop_start_stop_widgets["stop_button"]
        try:
            start_button.clicked.disconnect(self.start_timing_loop_custom)
            stop_button.clicked.disconnect(self.stop_timing_loop_custom)
            start_button.clicked.connect(self.start_timing_loop)
            stop_button.clicked.connect(self.stop_timing_loop)
        except TypeError:
            pass
            #A TypeError is thrown when you try to disconnect a signal that wasn't connected.
            # This occurs when you try to load a custom timing csv with invalid times (e.g., repeated timestamps), which will call this function.



# LIGHT_COLORS_NAMES = [
#     'lightcoral', 'lightsalmon', 'lightgoldenrodyellow', 'lightgreen',
#     'lightskyblue', 'lightsteelblue', 'lightgray', 'whitesmoke',
#     'aliceblue', 'beige', 'mintcream', 'seashell',
#     'lavenderblush', 'palevioletred'
# ] #names of light matplotlib colors
# LIGHT_COLORS = [mcolors.to_hex(c) for c in LIGHT_COLORS_NAMES]

if __name__ == '__main__':
    data_model = EbitDataModel()
    controller = EbitVoltageController(data_model)
    app = QApplication(sys.argv)
    app.setStyleSheet("QLineEdit{font-size: 12pt;} QLabel{font-size: 12pt;} QDoubleSpinBox{font-size: 12pt;}")
    dialog = Dialog(data_model, controller)
    sys.exit(dialog.exec())
