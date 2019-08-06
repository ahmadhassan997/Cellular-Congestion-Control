import matplotlib
import matplotlib.pyplot as plt

# Data for plotting
time, rsrp, sinr = [], [], []
for line in open('./../DlRsrpSinrStats.txt', 'r'):
  values = [s for s in line.split()]
  time.append(float(values[0]))
  rsrp.append(float(values[4]))
  sinr.append(float(values[5]))

time2, Rx = [], []
for line in open('./../udp-put.dat', 'r'):
  values = [s for s in line.split()]
  time2.append(float(values[0]))
  Rx.append(float(values[4]))

# fig, axs = plt.subplots(3 , sharex = True)
# # fig.tight_layout()
# axs[2].set_xlabel('Time (s)')
# # fig.suptitle('Downlink Cellular Condidtions')
# axs[0].set_title('Signal-To-Noise Ratio (dB)')
# axs[0].plot(time, sinr)
# axs[1].set_title('Referenced Signal Received Power (dBm)')
# axs[1].plot(time, rsrp , 'tab:red')
# axs[2].set_title('# of Received Bytes on Server')
# axs[2].plot(time2, Rx, 'tab:green' )
# plt.show()

