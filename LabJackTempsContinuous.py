"""
Opens a connnection from the LabJack to the lab computer via USB.
Reads two Analog In signals for the two EBIT temperature readings on the temperature monitor.
"""
import os
import sys
import numpy as np
import configparser
import datetime
import csv
import pyqtgraph as pg
from PyQt6.QtCore import QTimer, QCoreApplication
from PyQt6.QtWidgets import (QApplication, QDialog,
                             QGridLayout, QGroupBox,
                             QLabel, QLineEdit, QMessageBox,
                             QMenu, QMenuBar, QPushButton, QVBoxLayout, QHBoxLayout, QCheckBox, QSpinBox,
                             QDoubleSpinBox, QFileDialog, QLCDNumber)
from labjack import ljm
from slack_sdk.webhook import WebhookClient

class ContinuousTemperatureReader(QDialog):
    def __init__(self):
        super().__init__()
        config = configparser.ConfigParser()
        config.read('config.ini')
        self.time = 1 #data recording time elapsed, in seconds
        self.slack_enabled = True #send slack alerts for high temperatures or not
        self.slack = WebhookClient(config["slack"]["webhook_url"])
        self.ports = {"Temperature 1 (High)":"AIN1", "Temperature 2 (Low)":"AIN2"}
        self.handle = self.open_labjack()
        self.create_path_box()
        self.create_buttons()
        self.create_time_elapsed_box()
        self.create_t_plot_box()

        self.timeElapsedTimer = QTimer()
        self.timeElapsedTimer.timeout.connect(self.update_log_file)
        self.oneSecondTimer = QTimer() #to track time elapsed
        self.oneSecondTimer.timeout.connect(self.tick)
        self.oneSecondTimer.setInterval(1000)

        #the plot will store the last self.temperaturePlotLength values in chronological order
        self.temperaturePlotLength = 1000
        self.dataForTempPlot = {'T1':np.zeros(self.temperaturePlotLength), 'T2':np.zeros(self.temperaturePlotLength), 'time':np.zeros(self.temperaturePlotLength)}
        #np.append(self.dataForTempPlot[key][1:], value)

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.pathGroupBox)
        main_layout.addWidget(self._horizGroupBox)
        main_layout.addWidget(self.timeElapsedGroupBox)
        main_layout.addWidget(self.tempPlotBox)
        self.setLayout(main_layout)

        self.t1_limit = 44.3
        self.t2_limit = 4.1

        self.setWindowTitle("EBIT Temperature Monitor")

    def open_labjack(self):
        try:
            handle = ljm.openS("T7", "ANY", "ANY")  # T7 device, Any connection, Any identifier
            #info = ljm.getHandleInfo(handle)
            print("LabJack connected.")
            return handle
        except Exception as E:
            print(E)
            print("Labjack connection failed. Check the USB cable from the Labjack to the EBIT PC and restart the temperature monitor.")

    def read_temperature_voltages(self):
        voltages = ljm.eReadNames(self.handle, len(self.ports), list(self.ports.values()))
        return voltages

    def voltages_to_temperature(self, voltages):
        Temperature1 = min(voltages[0] * (100. / 10.), 100.)
        Temperature2 = min(voltages[1] * (25. / 10.), 100.)
        return [Temperature1, Temperature2]

    def read_temperature(self):
        return self.voltages_to_temperature(self.read_temperature_voltages())

    def makeLogFile(self):
        now = datetime.datetime.now()

        # Format the date and time as a string
        formatted_date_time = now.strftime("%Y-%m-%d_%Hh%Mm%Ss")

        # Create the file name using the formatted date and time
        file_name = f"ebit_log_{formatted_date_time}.csv"

        # Place file in the logs folder
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "continuous_t_logs", file_name)
        # Create and open the file in write mode
        print(path)
        self.logf = open(path, 'a', newline='')
        self.logf_format = ['time',"Temperature 1 (High)", "Temperature 2 (Low)"]
        self.logwriter = csv.DictWriter(self.logf, fieldnames=self.logf_format)
        self.logwriter.writeheader()
        # Print a message indicating the file creation
        print(f"File '{path}' created.")
        self.make_log_file_button.setEnabled(False)
        self.start_monitoring_button.setEnabled(True)
        self.path_label.setText(path)

    def close_labjack(self):
        print("Terminating LabJack connection.\n")
        ljm.close(self.handle)

    def update_log_file(self):
        T_log_dict = {"time":datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
        try:
            temperatures = self.read_temperature()
        except Exception as e:
            print("Failed to read temperatures:", e)
            temperatures = [0.0, 0.0]
        # We use the lakeshore 218's analog out for temperature monitoring.
        # As of 1/14/2025, Grant set the following scaling settings: analog 1: 0V@0K, 10V@25K. analog 2: 0V@0K, 10V@100K
        Temperature1 = min(temperatures[0], 100.)
        Temperature2 = min(temperatures[1], 100.)
        T_log_dict["Temperature 1 (High)"] = round(Temperature1, 5) if Temperature1 is not None else Temperature1
        T_log_dict["Temperature 2 (Low)"] = round(Temperature2, 5) if Temperature2 is not None else Temperature2
        self.logwriter.writerow(T_log_dict)
        self.logf.flush()
        self.dataForTempPlot["T1"] = np.append(self.dataForTempPlot["T1"][1:], Temperature1)
        self.dataForTempPlot["T2"] = np.append(self.dataForTempPlot["T2"][1:], Temperature2)
        self.dataForTempPlot["time"] = np.append(self.dataForTempPlot["time"][1:], datetime.datetime.timestamp(datetime.datetime.now()))
        #todo: change measurement frequency when temperature exceeds limit
        if Temperature1 >= self.t1_limit:
            t1_brush = pen={'color':'red', 'width':1}
            self.send_slack({"T1":Temperature1, "T2":Temperature2})
        else:
            t1_brush = pen={'color':'white', 'width':1}
        if Temperature2 >= self.t2_limit:
            t2_brush = pen={'color':'red', 'width':1}
            self.send_slack({"T1": Temperature1, "T2": Temperature2})
        else:
            t2_brush = pen={'color':'white', 'width':1}
        ###Clear plots before replotting
        self.t1_plot.clear()
        self.t2_plot.clear()
        self.t1_plot.plot(self.dataForTempPlot["time"][self.dataForTempPlot["time"]!=0], self.dataForTempPlot["T1"][self.dataForTempPlot["T1"]!=0], pen=t1_brush)
        self.t2_plot.plot(self.dataForTempPlot["time"][self.dataForTempPlot["time"]!=0], self.dataForTempPlot["T2"][self.dataForTempPlot["T2"]!=0], pen=t2_brush)
        self.latest_t_label.setText(f'T1: {Temperature1:03.3f}   T2: {Temperature2:03.3f}')

    def create_path_box(self):
        self.pathGroupBox=QGroupBox()
        layout=QHBoxLayout()
        layout.addWidget(QLabel("Log file path:"))

        self.path_label = QLabel("")
        self.make_log_file_button = QPushButton("Make Log File")
        self.make_log_file_button.clicked.connect(self.makeLogFile)

        layout.addWidget(self.path_label)
        layout.addWidget(self.make_log_file_button)
        self.pathGroupBox.setLayout(layout)

    def send_slack(self, temp_dict):
        if self.slack_enabled:
            self.slack.send(text=f"EBIT temperature limit(s) exceeded.\n40K Stage: {temp_dict['T1']:.2f}K (limit: {self.t1_limit:.2f}K)\n4K Stage: {temp_dict['T2']:.3f}K (limit: {self.t2_limit:.3f}K)")

    def create_buttons(self):
        """Creates GUI buttons to control static voltages and one-time ramps (i.e., not the timing loops)."""
        self._horizGroupBox = QGroupBox()
        layout = QHBoxLayout()

        self.t_increment_spinbox = QDoubleSpinBox(value=5)
        self.t_increment_spinbox.setMinimum(.1)
        self.t_increment_spinbox.setMaximum(10000)

        self.start_monitoring_button = QPushButton("Start Temperature Monitoring")
        self.start_monitoring_button.clicked.connect(self.start_monitoring)
        self.start_monitoring_button.setEnabled(False)

        self.stop_monitoring_button = QPushButton("Stop Temperature Monitoring")
        self.stop_monitoring_button.clicked.connect(self.stop_monitoring)
        self.stop_monitoring_button.setEnabled(False)
        layout.addWidget(QLabel("Time between measurements (s):"))
        layout.addWidget(self.t_increment_spinbox)
        layout.addWidget(self.start_monitoring_button)
        layout.addWidget(self.stop_monitoring_button)
        self._horizGroupBox.setLayout(layout)

    def create_time_elapsed_box(self):
        self.timeElapsedGroupBox = QGroupBox()
        layout = QHBoxLayout()
        self.timeElapsedLabel = QLabel()
        self.update_timer()
        self.latest_t_label = QLabel()

        layout.addWidget(QLabel("Time Elapsed: "))
        layout.addWidget(self.timeElapsedLabel)
        layout.addWidget(QLabel("Last Measurement:"))
        layout.addWidget(self.latest_t_label)
        self.timeElapsedGroupBox.setLayout(layout)

    def create_t_plot_box(self):
        self.tempPlotBox = QGroupBox()
        layout = QVBoxLayout()
        self.t1_plot = pg.PlotWidget()
        self.t1_plot.setLabel(axis='left', text='<span style="color: yellow; font-size: 16pt">T1 (40K) </span>')
        self.t1_plot.setLabel(axis='bottom', text='<span style="color: yellow; font-size: 16pt">Time (s) </span>')

        self.t2_plot = pg.PlotWidget()
        self.t2_plot.setLabel(axis='bottom', text='<span style="color: yellow; font-size: 16pt">Time (s) </span>')
            #axis='left', text='T2 (4K) Temperature', units='K', size='14pt')
        self.t2_plot.setLabel(axis='left', text='<span style="color: yellow; font-size: 16pt">T2 (4K) </span>')
        self.t2_plot.setXLink(self.t1_plot)
        axis1 = pg.DateAxisItem()
        self.t1_plot.setAxisItems({'bottom': axis1})
        axis2 = pg.DateAxisItem()
        self.t2_plot.setAxisItems({'bottom': axis2})
        layout.addWidget(self.t1_plot)
        layout.addWidget(self.t2_plot)
        self.tempPlotBox.setLayout(layout)


    def tick(self):
        self.time += 1
        self.update_timer()

    def update_timer(self):
        self.timeElapsedLabel.setText(f"{self.time // (3600 * 24) :02d}:{(self.time // 3600)%24:02d}:{(self.time // 60)%60:02d}:{self.time % 60:02d}")
        #self.lcdTimer.display("%d:%05.2f" % (self.time // 60, self.time % 60))

    def start_monitoring(self):
        period_s = float(self.t_increment_spinbox.value())
        self.disable_buttons()
        self.timeElapsedTimer.setInterval(int(period_s*1000))
        self.oneSecondTimer.start()
        self.timeElapsedTimer.start()

    def stop_monitoring(self):
        self.enable_buttons()
        self.oneSecondTimer.stop()
        self.timeElapsedTimer.stop()

    def disable_buttons(self):
        self.t_increment_spinbox.setEnabled(False)
        self.start_monitoring_button.setEnabled(False)
        self.stop_monitoring_button.setEnabled(True)

    def enable_buttons(self):
        self.t_increment_spinbox.setEnabled(True)
        self.start_monitoring_button.setEnabled(True)
        self.stop_monitoring_button.setEnabled(False)

    def closeEvent(self, e):
        print("Closing temperature monitor.")
        """
        Called when the user clicks the close icon in the window corner.
        """
        try:
            self.close_labjack()
            self.logf.close()
        except Exception as e:
            print(e)
        QCoreApplication.quit()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyleSheet("QLineEdit{font-size: 12pt;} QLabel{font-size: 12pt;} QDoubleSpinBox{font-size: 12pt;}")
    tr = ContinuousTemperatureReader()
    sys.exit(tr.exec())


