import matplotlib.pyplot as plt
import rosbag
import numpy as np

diffError = []
tuneError = []
laserError = []
laserDiffError= []
laserTuneError = []

""" Test linii """

diff = rosbag.Bag('../Dobre bagi/diffTL.bag')
tune = rosbag.Bag('../Dobre bagi/tuneTL.bag')
laser_no_odom = rosbag.Bag('../Dobre bagi/laserTL_no_odom.bag')
laser_diff = rosbag.Bag('../Dobre bagi/laserTL_odom_diff.bag')
laser_tune = rosbag.Bag('../Dobre bagi/laserTL_odom_tune.bag')


for topic, msg, t in diff.read_messages(topics = ['/errors']):
    diffError.append(msg.data*100)

for topic, msg, t in tune.read_messages(topics = ['/errors']):
    tuneError.append(msg.data*100)

for topic, msg, t in laser_no_odom.read_messages(topics = ['/errors']):
    laserError.append(msg.data*100)

for topic, msg, t in laser_diff.read_messages(topics = ['/errors']):
    laserDiffError.append(msg.data*100)

for topic, msg, t in laser_tune.read_messages(topics = ['/errors']):
    laserTuneError.append(msg.data*100)

diff.close()
tune.close()
laser_no_odom.close()
laser_diff.close()
laser_tune.close()

plt.figure(1)
plt.plot(np.linspace(0, 20, len(laserTuneError)), laserTuneError, label='tune_controller+laser_scan_matcher')
#plt.plot(np.linspace(0, 20, len(tuneError)), tuneError, label='tune_controller')
plt.plot(np.linspace(0, 20, len(laserError)), laserError, label='laser_scan_matcher')
plt.plot(np.linspace(0, 20, len(laserDiffError)), laserDiffError, label='diff_drive_controller+laser_scan_matcher')
plt.plot(np.linspace(0, 20, len(diffError)), diffError, label='diff_drive_controller')
plt.xlabel('Czas [s]')
plt.ylabel('Blad (odleglosc) [cm]')
plt.title('Porownanie bledow dla testu linii')
plt.legend(loc="upper left")
plt.show()


""" Test obrotu """

diff = rosbag.Bag('../Dobre bagi/diffTO.bag')
tune = rosbag.Bag('../Dobre bagi/tuneTL.bag')
laser_no_odom = rosbag.Bag('../Dobre bagi/laserTO_no_odom.bag')
laser_diff = rosbag.Bag('../Dobre bagi/laserTO_odom_diff.bag')
laser_tune = rosbag.Bag('../Dobre bagi/laserTO_odom_tune.bag')


for topic, msg, t in diff.read_messages(topics = ['/errors']):
    diffError.append(msg.data*100)

for topic, msg, t in tune.read_messages(topics = ['/errors']):
    tuneError.append(msg.data*100)

for topic, msg, t in laser_no_odom.read_messages(topics = ['/errors']):
    laserError.append(msg.data*100)

for topic, msg, t in laser_diff.read_messages(topics = ['/errors']):
    laserDiffError.append(msg.data*100)

for topic, msg, t in laser_tune.read_messages(topics = ['/errors']):
    laserTuneError.append(msg.data*100)

diff.close()
tune.close()
laser_no_odom.close()
laser_diff.close()
laser_tune.close()

for i in range(len(diffError)):
    if diffError[i] > 2.0:
        diffError[i] = diffError[i-1]
for i in range(len(tuneError)):
    if tuneError[i] > 6.0:
        tuneError[i] = tuneError[i-1]
for i in range(len(laserError)):
    if laserError[i] > 2.0:
        laserError[i] = laserError[i-1]
for i in range(len(laserDiffError)):
    if laserDiffError[i] > 2.0:
        laserDiffError[i] = laserDiffError[i-1]
for i in range(len(laserTuneError)):
    if laserTuneError[i] > 2.0:
        laserTuneError[i] = laserTuneError[i-1]

plt.figure(2)
plt.plot(np.linspace(0, 40, len(laserTuneError)), laserTuneError, label='tune_controller+laser_scan_matcher')
#plt.plot(np.linspace(0, 40, len(tuneError)), tuneError, label='tune_controller')
plt.plot(np.linspace(0, 40, len(laserError)), laserError, label='laser_scan_matcher')
plt.plot(np.linspace(0, 40, len(laserDiffError)), laserDiffError, label='diff_drive_controller+laser_scan_matcher')
plt.plot(np.linspace(0, 40, len(diffError)), diffError, label='diff_drive_controller')
plt.xlabel('Czas [s]')
plt.ylabel('Blad (obrot) [rad]')
plt.title('Porownanie bledow dla testu obrotu')
plt.legend(loc="upper left")
plt.show()



""" Test kwadratu """

diff = rosbag.Bag('../Dobre bagi/diffTK.bag')
tune = rosbag.Bag('../Dobre bagi/tuneTK.bag')
laser_no_odom = rosbag.Bag('../Dobre bagi/laserTK_no_odom.bag')
laser_diff = rosbag.Bag('../Dobre bagi/laserTK_odom_diff.bag')
laser_tune = rosbag.Bag('../Dobre bagi/laserTK_odom_tune.bag')


for topic, msg, t in diff.read_messages(topics = ['/errors']):
    diffError.append(msg.data*100)

for topic, msg, t in tune.read_messages(topics = ['/errors']):
    tuneError.append(msg.data*100)

for topic, msg, t in laser_no_odom.read_messages(topics = ['/errors']):
    laserError.append(msg.data*100)

for topic, msg, t in laser_diff.read_messages(topics = ['/errors']):
    laserDiffError.append(msg.data*100)

for topic, msg, t in laser_tune.read_messages(topics = ['/errors']):
    laserTuneError.append(msg.data*100)

diff.close()
tune.close()
laser_no_odom.close()
laser_diff.close()
laser_tune.close()


plt.figure(1)
plt.plot(np.linspace(0, 50, len(laserTuneError)), laserTuneError, label='tune_controller+laser_scan_matcher')
#plt.plot(np.linspace(0, 50, len(tuneError)), tuneError, label='tune_controller')
plt.plot(np.linspace(0, 50, len(laserError)), laserError, label='laser_scan_matcher')
plt.plot(np.linspace(0, 50, len(laserDiffError)), laserDiffError, label='diff_drive_controller+laser_scan_matcher')
plt.plot(np.linspace(0, 50, len(diffError)), diffError, label='diff_drive_controller')
plt.xlabel('Czas [s]')
plt.ylabel('Blad (odleglosc) [cm]')
plt.title('Porownanie bledow dla testu kwadratu')
plt.legend(loc="upper left")
plt.show()