import nidaqmx



task = nidaqmx.Task()  
task_2 = nidaqmx.Task()

# task.di_channels.add_di_chan("Dev1/PFI12")
# task_2.di_channels.add_di_chan("Dev1/PFI0")
# #task_2.timing.samp_clk_active_edge(10280)


task.co_channels.add_co_pulse_chan_freq("Dev1/ctr0")

task.timing.cfg_implicit_timing()


task.triggers.start_trigger.cfg_dig_edge_start_trig("Dev2/PFI0")

###Read is for input devices
# t = task.read(10)
# print t

task.close()