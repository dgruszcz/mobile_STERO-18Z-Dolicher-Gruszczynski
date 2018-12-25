import matplotlib.pyplot as plt
import rosbag
import numpy as np
import math

def getRotZ(q):
    siny_cosp = +2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z* q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


bag = rosbag.Bag('../test_obrotu_diff_drive_controller.bag')
print bag.get_message_count()
print bag.get_end_time()
print bag.get_start_time()

kat_gazebo = []
kat_sensor = []
time = []

# zmienne pomocnicze
last_gz = 0
last_sen = 0
iterator_gz = 0
iterator_sen = 0

for topic, msg, t in bag.read_messages(topics = ['/elektron/mobile_base_controller/odom', '/gazebo_odom']):
    if topic == '/elektron/mobile_base_controller/odom':
        if iterator_sen == 0:
            current_sen = getRotZ(msg.pose.pose.orientation)
            kat_sensor.append(current_sen)
            last_sen = current_sen
        else:
            current_sen = getRotZ(msg.pose.pose.orientation)
            kat_sensor.append(kat_sensor[iterator_sen-1] + math.fabs(math.fabs(current_sen) - math.fabs(last_sen)))
            last_sen = current_sen
        iterator_sen = iterator_sen + 1
    else:
        if iterator_gz == 0:
            current_gz = getRotZ(msg.pose.pose.orientation)
            kat_gazebo.append(current_gz)
            last_gz = current_gz
        else:
            current_gz = getRotZ(msg.pose.pose.orientation)
            kat_gazebo.append(kat_gazebo[iterator_gz-1] + math.fabs(math.fabs(current_gz) - math.fabs(last_gz)))
            last_gz = current_gz
        iterator_gz = iterator_gz + 1
        time.append(t)


""" UWAGA - diff_drive_controller nadaje z 2x wieksza czestotliwoscia!
    Dla tune_controllera ponizszy kod nalezy zakomentowac."""
for i in range(len(kat_gazebo)-1):
    kat_sensor[i] = kat_sensor[2*i]

count = len(kat_gazebo)-2
timeFromStart = []
for i in range(count):
    timeFromStart.append(time[i].secs + 0.000000001*time[i].nsecs - time[0].secs - 0.000000001*time[0].nsecs)

moveDuration = 2*math.pi / 0.3  # liczba sekund
samplesCount = moveDuration/0.020    # wezly publikuja z czestotliwoscia 50Hz


startOfRotation = 83  # Numer probki w ktorej rozpoczal sie obrot
kat_zadany = np.hstack((np.zeros(startOfRotation), np.linspace(0, 2*math.pi, int(samplesCount))))
kat_zadany = np.hstack((kat_zadany, 2*math.pi*np.ones(len(kat_gazebo) - len(kat_zadany))))

plt.figure(1)
plt.plot(timeFromStart, kat_gazebo[:count], label='Obrot rzeczywisty', lw=3.0)
plt.plot(timeFromStart, kat_sensor[:count], label='Obrot zmierzony', lw=2.0)
plt.plot(timeFromStart, kat_zadany[:count], label='Obrot zadany', ls='--')
plt.xlabel('Czas [s]')
plt.ylabel('Kat [rad]')
plt.title('Sterowanie ze sprzezeniem - diff_drive_controller\n(obrot o 360 stopni)')
plt.legend(loc='upper left')
plt.show()


kat_gazebo = np.array(kat_gazebo)
kat_sensor = np.array(kat_sensor)


blad = np.fabs(kat_gazebo[:count] - kat_sensor[:count])


plt.figure(2)
plt.plot(timeFromStart, blad)
plt.xlabel('Czas [s]')
plt.ylabel('Blad [rad]')
plt.title('Sterowanie ze sprzezeniem - diff_drive_controller\n(obrot o 360 stopni)')
plt.show()