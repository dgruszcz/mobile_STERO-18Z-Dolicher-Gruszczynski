import matplotlib.pyplot as plt
import rosbag
import numpy as np
import math

name = 'tuneTO'
name2 = "tune_controller"
bag = rosbag.Bag('../dontAskMeWhy/'+name+'.bag')
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
    blad[i] += -2*math.pi if blad[i] > math.pi else (2*math.pi if blad[i] < -math.pi else 0)
    if blad[i] < 0:
        blad[i] = -blad[i]


moveDuration = 2*math.pi / 0.565  # liczba sekund
samplesCount = moveDuration/0.020    # wezly publikuja z czestotliwoscia 50Hz

kat_zadany1 = np.zeros(55)
kat_zadany2 = np.linspace(0, 2*math.pi, int(samplesCount))
kat_zadany3 = np.ones(len(kat_gazebo) - int(samplesCount) - len(kat_zadany1))*2*math.pi
kat_zadany = np.hstack((kat_zadany1, kat_zadany2, kat_zadany3))

plt.figure(1, figsize=(10, 6), dpi=100)
plt.plot(time, kat_gazebo, label='Obrot rzeczywisty', lw=2.0)
plt.plot(time, kat_sensor, label='Obrot zmierzony', lw=2.0)
plt.plot(time, kat_zadany, label='Obrot zadany', ls='--')
plt.xlabel('Czas [s]')
plt.ylabel('Kat [rad]')
plt.title('Sterowanie ze sprzezeniem (obrot o 360 stopni)\n'+name2)
plt.legend(loc='upper left')
#plt.savefig('../wykresy2/'+name+'.png', dpi=600)
plt.show()


# """ Blad: """
#
ref = np.array(kat_gazebo)
real = np.array(kat_sensor)
blad = np.fabs((real-ref))

plt.figure(2, figsize=(10, 7), dpi=100)
plt.plot(np.linspace(0, 40, len(blad)), blad, label=name2)
plt.xlabel('Czas [s]')
plt.ylabel('Blad [rad]')
plt.title('Porownanie bledow dla testu obrotu')
plt.savefig('../wykresy2/'+name+'_blad.png', dpi=600)
plt.legend(loc="upper left")
plt.show()