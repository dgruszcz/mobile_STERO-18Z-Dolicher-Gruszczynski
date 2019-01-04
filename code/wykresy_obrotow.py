import matplotlib.pyplot as plt
import rosbag
import numpy as np
import math


bag = rosbag.Bag('../tuneTO.bag')
print bag.get_message_count()

kat_gazebo = []
kat_sensor = []
blad = []
time = []
totalTime = []

# zmienne pomocnicze
last_gz = 0
last_sen = 0
iterator_gz = 0
iterator_sen = 0

for topic, msg, t in bag.read_messages(topics = ['/enkoder', '/errors', '/gazebo_odom']):
    if topic == '/enkoder':
        if iterator_sen == 0:
            current_sen = msg.theta
            kat_sensor.append(current_sen)
            last_sen = current_sen
        else:
            current_sen = msg.theta
            kat_sensor.append(kat_sensor[iterator_sen-1] + math.fabs(math.fabs(current_sen) - math.fabs(last_sen)))
            last_sen = current_sen
        iterator_sen = iterator_sen + 1
    elif topic == '/errors':
        blad.append(msg.data)
        time.append(t.secs + 0.000000001 * t.nsecs)
    else:
        if iterator_gz == 0:
            current_gz = msg.theta
            kat_gazebo.append(current_gz)
            last_gz = current_gz
        else:
            current_gz = msg.theta
            kat_gazebo.append(kat_gazebo[iterator_gz-1] + math.fabs(math.fabs(current_gz) - math.fabs(last_gz)))
            last_gz = current_gz
        iterator_gz = iterator_gz + 1

bag.close()

for i in range(len(time)):
    time[len(time)-1-i] = time[len(time)-1-i] - time[0]

for i in range(len(blad)):
    if blad[i] > 2.0:
        blad[i] = blad[i-1]


moveDuration = 2*math.pi / 0.57  # liczba sekund
samplesCount = moveDuration/0.020    # wezly publikuja z czestotliwoscia 50Hz

kat_zadany = np.linspace(0, 2*math.pi, int(samplesCount))
kat_zadany = np.hstack((np.zeros(len(time)-int(samplesCount)), kat_zadany))

plt.figure(1)
plt.plot(time, kat_gazebo, label='Obrot rzeczywisty', lw=2.0)
plt.plot(time, kat_sensor, label='Obrot zmierzony', lw=2.0)
plt.plot(time, kat_zadany, label='Obrot zadany', ls='--')
plt.xlabel('Czas [s]')
plt.ylabel('Kat [rad]')
plt.title('Sterowanie ze sprzezeniem (obrot o 360 stopni)\nlaser_scan_matcher+tune_controller')
plt.legend(loc='upper left')
plt.show()


""" Blad: """

for i in range(len(time)):
    time[len(time)-1-i] = time[len(time)-1-i] - time[0]

plt.figure(2)
plt.plot(time, blad)
plt.xlabel('Czas [s]')
plt.ylabel('Blad [rad]')
plt.title('Sterowanie ze sprzezeniem (obrot o 360 stopni)\nlaser_scan_matcher+tune_controller')
plt.show()