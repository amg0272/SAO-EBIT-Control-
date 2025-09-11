"""
Opens a connnection from the LabJack to the lab computer via USB.
Reads two Analog In signals for the two EBIT temperature readings on the temperature monitor.
"""
from labjack import ljm

class TemperatureReader:
    def __init__(self):
        self.ports = {"Temperature 1 (High)":"AIN1", "Temperature 2 (Low)":"AIN2"}
        self.handle = self.open_labjack()

    def open_labjack(self):
        handle = ljm.openS("T7", "ANY", "ANY")  # T7 device, Any connection, Any identifier
        #info = ljm.getHandleInfo(handle)
        print("LabJack connected.")
        return handle

    def read_temperature_voltages(self):
        voltages = ljm.eReadNames(self.handle, len(self.ports), list(self.ports.values()))

        # print("\nResults: ")
        # for i in range(numFrames):
        #     print("    Name - %s, value : %f" % (names[i], results[i]))

        return voltages

    def close_labjack(self):
        # Close handle
        print("Terminating LabJack connection.\n")
        ljm.close(self.handle)



if __name__ == '__main__':
    tr = TemperatureReader()
    temps = tr.read_temperature_voltages()
    print(temps)
    tr.close_labjack()
