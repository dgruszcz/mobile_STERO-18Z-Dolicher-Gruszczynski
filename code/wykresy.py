import matplotlib.pyplot as plt
import rosbag
import numpy as np

test_kwadratu = True
bag = rosbag.Bag('../test_kwadratu_tune_controller.bag')
print bag.get_message_count()
print bag.get_end_time()
print bag.get_start_time()
x_gazebo = []
y_gazebo = []
x_sensor = []
y_sensor = []
time = []
for topic, msg, t in bag.read_messages(topics = ['/elektron/mobile_base_controller/odom', '/gazebo_odom']):
    if topic == '/elektron/mobile_base_controller/odom':
        x_sensor.append(msg.pose.pose.position.x)
        y_sensor.append(msg.pose.pose.position.y)
    else:
        x_gazebo.append(msg.pose.pose.position.x)
        y_gazebo.append(msg.pose.pose.position.y)
        time.append(t)


plt.figure(1)
plt.plot(x_gazebo, y_gazebo, label='Polozenie rzeczywiste', lw=2.0)
plt.plot(x_sensor, y_sensor, label='Polozenie zmierzone')
if test_kwadratu:
    plt.plot([0, 1, 1, 0, 0], [0, 0, 1, 1, 0], label='Trajektoria zadana', ls='--')
    plt.title('Sterowanie ze sprzezeniem - diff_drive_controller\n(ruch po kwadracie)')
else:
    plt.plot([0, 1], [0, 0], label='Trajektoria zadana', ls='--')
    plt.title('Sterowanie ze sprzezeniem - diff_drive_controller\n(ruch po linii)')
plt.xlabel('Polozenie w osi X')
plt.ylabel('Polozenie w osi Y')

plt.legend(loc='upper right')
plt.show()

count = len(x_gazebo)-2
x_gazebo = np.array(x_gazebo)
y_gazebo = np.array(y_gazebo)
x_sensor = np.array(x_sensor)
y_sensor = np.array(y_sensor)

""" UWAGA - diff_drive_controller nadaje z 2x wieksza czestotliwoscia!
    Dla tune_controllera ponizszy kod nalezy zakomentowac"""
# for i in range(len(x_gazebo)-1):
#     x_sensor[i] = x_sensor[2*i]
#     y_sensor[i] = y_sensor[2*i]


blad = np.sqrt((y_gazebo[:count] - y_sensor[:count])**2 + (x_gazebo[:count] - x_sensor[:count])**2)
timeFromStart = []
for i in range(count):
    timeFromStart.append(time[i].secs + 0.000000001*time[i].nsecs - time[0].secs - 0.000000001*time[0].nsecs)

plt.figure(2)
plt.plot(timeFromStart, blad)
plt.xlabel('Czas [s]')
plt.ylabel('Blad (odleglosc) [m]')
if test_kwadratu:
    plt.title('Sterowanie ze sprzezeniem - diff_drive_controller\n(ruch po kwadracie)')
else:
    plt.title('Sterowanie ze sprzezeniem - diff_drive_controller\n(ruch po linii)')
plt.show()