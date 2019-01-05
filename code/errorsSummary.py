import matplotlib.pyplot as plt
import rosbag
import numpy as np
import math

diffError = []
tuneError = []
laserError = []
laserDiffError= []
laserTuneError = []

""" Test linii """

diff = rosbag.Bag('../diffTL.bag')
tune = rosbag.Bag('../tuneTL.bag')
laser_no_odom = rosbag.Bag('../laserTL_no_odom.bag')
laser_diff = rosbag.Bag('../laserTL_odom_diff.bag')
laser_tune = rosbag.Bag('../laserTL_odom_tune.bag')


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

plt.figure(1, figsize=(10, 7), dpi=100)
#plt.plot(np.linspace(0, 20, len(laserTuneError)), laserTuneError, label='tune_controller+laser_scan_matcher')
plt.plot(np.linspace(0, 20, len(tuneError)), tuneError, label='tune_controller')
#plt.plot(np.linspace(0, 20, len(laserError)), laserError, label='laser_scan_matcher')
#plt.plot(np.linspace(0, 20, len(laserDiffError)), laserDiffError, label='diff_drive_controller+laser_scan_matcher')
#plt.plot(np.linspace(0, 20, len(diffError)), diffError, label='diff_drive_controller')
plt.xlabel('Czas [s]')
plt.ylabel('Blad (odleglosc) [cm]')
plt.title('Porownanie bledow dla testu linii')
plt.legend(loc="upper left")
plt.savefig('../wykresy2/tuneTL_blad.png', dpi=600)
#plt.show()

print 'Test linii'
print 'Diff: ', diffError[-1], '\nTune: ', tuneError[-1], '\nLaser: ',laserError[-1], \
    '\nLaserDiff: ', laserDiffError[-1], '\nLaserTune: ', laserTuneError[-1]

""" Test obrotu """
diffError = []
tuneError = []
laserError = []
laserDiffError= []
laserTuneError = []

diff = rosbag.Bag('../diffTO.bag')
tune = rosbag.Bag('../tuneTO.bag')
laser_no_odom = rosbag.Bag('../laserTO_no_odom.bag')
laser_diff = rosbag.Bag('../laserTO_odom_diff.bag')
laser_tune = rosbag.Bag('../laserTO_odom_tune.bag')


for topic, msg, t in diff.read_messages(topics = ['/errors']):
    diffError.append(msg.data)

for topic, msg, t in tune.read_messages(topics = ['/errors']):
    tuneError.append(msg.data)

for topic, msg, t in laser_no_odom.read_messages(topics = ['/errors']):
    laserError.append(msg.data)

for topic, msg, t in laser_diff.read_messages(topics = ['/errors']):
    laserDiffError.append(msg.data)

for topic, msg, t in laser_tune.read_messages(topics = ['/errors']):
    laserTuneError.append(msg.data)

diff.close()
tune.close()
laser_no_odom.close()
laser_diff.close()
laser_tune.close()

for i in range(len(diffError)):
    diffError[i] += -2*math.pi if diffError[i] > math.pi else (2*math.pi if diffError[i] < -math.pi else 0)
    if diffError[i] < 0:
        diffError[i] = -diffError[i]
for i in range(len(tuneError)):
    tuneError[i] += -2*math.pi if tuneError[i] > math.pi else (2*math.pi if tuneError[i] < -math.pi else 0)
    if tuneError[i] < 0:
        tuneError[i] = -tuneError[i]
for i in range(len(laserError)):
    laserError[i] += -2*math.pi if laserError[i] > math.pi else (2*math.pi if laserError[i] < -math.pi else 0)
    if laserError[i] < 0:
        laserError[i] = -laserError[i]
for i in range(len(laserDiffError)):
    laserDiffError[i] += -2*math.pi if laserDiffError[i] > math.pi else (2*math.pi if laserDiffError[i] < -math.pi else 0)
    if laserDiffError[i] < 0:
        laserDiffError[i] = -laserDiffError[i]
for i in range(len(laserTuneError)):
    laserTuneError[i] += -2*math.pi if laserTuneError[i] > math.pi else (2*math.pi if laserTuneError[i] < -math.pi else 0)
    if laserTuneError[i] < 0:
        laserTuneError[i] = -laserTuneError[i]


plt.figure(2, figsize=(10, 7), dpi=100)
#plt.plot(np.linspace(0, 40, len(laserTuneError)), laserTuneError, label='tune_controller+laser_scan_matcher')
plt.plot(np.linspace(0, 40, len(tuneError)), tuneError, label='tune_controller')
#plt.plot(np.linspace(0, 40, len(laserError)), laserError, label='laser_scan_matcher')
#plt.plot(np.linspace(0, 40, len(laserDiffError)), laserDiffError, label='diff_drive_controller+laser_scan_matcher')
#plt.plot(np.linspace(0, 40, len(diffError)), diffError, label='diff_drive_controller')
plt.xlabel('Czas [s]')
plt.ylabel('Blad (obrot) [rad]')
plt.title('Porownanie bledow dla testu obrotu')
plt.legend(loc="upper left")
plt.savefig('../wykresy2/tuneTO_blad.png', dpi=600)
#plt.show()

print 'Test obrotu'
print 'Diff: ', diffError[-1], '\nTune: ', tuneError[-1], '\nLaser: ',laserError[-1], \
    '\nLaserDiff: ', laserDiffError[-1], '\nLaserTune: ', laserTuneError[-1]

""" Test kwadratu """
diffError = []
tuneError = []
laserError = []
laserDiffError= []
laserTuneError = []

diff = rosbag.Bag('../diffTK.bag')
tune = rosbag.Bag('../tuneTK.bag')
laser_no_odom = rosbag.Bag('../laserTK_no_odom.bag')
laser_diff = rosbag.Bag('../laserTK_odom_diff.bag')
laser_tune = rosbag.Bag('../laserTK_odom_tune.bag')


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


plt.figure(3, figsize=(10, 7), dpi=100)
#plt.plot(np.linspace(0, 50, len(laserTuneError)), laserTuneError, label='tune_controller+laser_scan_matcher')
plt.plot(np.linspace(0, 50, len(tuneError)), tuneError, label='tune_controller')
#plt.plot(np.linspace(0, 50, len(laserError)), laserError, label='laser_scan_matcher')
#plt.plot(np.linspace(0, 50, len(laserDiffError)), laserDiffError, label='diff_drive_controller+laser_scan_matcher')
#plt.plot(np.linspace(0, 50, len(diffError)), diffError, label='diff_drive_controller')
plt.xlabel('Czas [s]')
plt.ylabel('Blad (odleglosc) [cm]')
plt.title('Porownanie bledow dla testu kwadratu')
plt.legend(loc="upper left")
plt.savefig('../wykresy2/tuneTK_blad.png', dpi=600)
plt.show()

print 'Test linii'
print 'Diff: ', diffError[-1], '\nTune: ', tuneError[-1], '\nLaser: ',laserError[-1], \
    '\nLaserDiff: ', laserDiffError[-1], '\nLaserTune: ', laserTuneError[-1]