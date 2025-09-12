import nidaqmx.system

system = nidaqmx.system.System.local()

print(system.driver_version)

# This is comment.
for device in system.devices:
    print([i.name for i in device.ci_physical_chans])
    print([i.name for i in device.ao_physical_chans])
    print([i.name for i in device.ai_physical_chans])
    print(device.co_max_timebase)