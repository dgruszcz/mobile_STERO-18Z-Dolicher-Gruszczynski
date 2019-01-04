import matplotlib.pyplot as plt
import rosbag

test_kwadratu = False
bag = rosbag.Bag('../tuneTL.bag')
print bag.get_message_count()
x_gazebo = []
y_gazebo = []
x_sensor = []
y_sensor = []
blad = []
time = []
for topic, msg, t in bag.read_messages(topics = ['/enkoder', '/errors', '/gazebo_odom']):
    if topic == '/enkoder':
        x_sensor.append(msg.x)
        y_sensor.append(msg.y)
    elif topic == '/errors':
        blad.append(msg.data*100)
        time.append(t.secs + 0.000000001*t.nsecs)
    else:
        x_gazebo.append(msg.x)
        y_gazebo.append(msg.y)

bag.close()

""" Tajektoria: """
plt.figure(1)
plt.plot(x_gazebo, y_gazebo, label='Polozenie rzeczywiste', lw=2.0)
plt.plot(x_sensor, y_sensor, label='Polozenie zmierzone')
if test_kwadratu:
    plt.plot([0, 1, 1, 0, 0], [0, 0, 1, 1, 0], label='Trajektoria zadana', ls='--')
    plt.title('Sterowanie ze sprzezeniem (ruch po kwadracie)\nlaser_scan_matcher+tune_controller')
else:
    plt.plot([0, 1], [0, 0], label='Trajektoria zadana', ls='--')
    plt.title('Sterowanie ze sprzezeniem (ruch po linii)\nlaser_scan_matcher+tune_controller')
plt.xlabel('Polozenie w osi X')
plt.ylabel('Polozenie w osi Y')

plt.legend(loc='lower right')
plt.show()


""" Blad: """
for i in range(len(time)):
    time[len(time)-1-i] = time[len(time)-1-i] - time[0]
plt.figure(2)
plt.plot(time, blad)
plt.xlabel('Czas [s]')
plt.ylabel('Blad (odleglosc) [cm]')
if test_kwadratu:
    plt.title('Sterowanie ze sprzezeniem (ruch po kwadracie)\nlaser_scan_matcher+tune_controller')
else:
    plt.title('Sterowanie ze sprzezeniem (ruch po linii)\nlaser_scan_matcher+diff_drive_controller')
plt.show()