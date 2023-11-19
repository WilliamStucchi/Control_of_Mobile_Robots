import sys
import rosbag
import matplotlib.pyplot as plt

# Read data from bag
bag = rosbag.Bag(sys.argv[1])

# State published by the simulator
vehicleState_time = []
vehicleState_x = []
vehicleState_y = []
vehicleState_theta = []
vehicleState_act_x = []
vehicleState_act_y = []
vehicleState_act_theta = []
vehicleState_linearvelocity = []
vehicleState_angularvelocity = []

for topic, msg, t in bag.read_messages():
    if topic == "/state":
        vehicleState_time.append(msg.data[0])
        vehicleState_x.append(msg.data[1])
        vehicleState_y.append(msg.data[2])
        vehicleState_theta.append(msg.data[3])
        vehicleState_act_x.append(msg.data[4])
        vehicleState_act_y.append(msg.data[5])
        vehicleState_act_theta.append(msg.data[6])
        vehicleState_linearvelocity.append(msg.data[7])
        vehicleState_angularvelocity.append(msg.data[8])

bag.close()

# Plot data
plt.figure(1)
plt.title('Trajectory')
plt.plot(vehicleState_x,vehicleState_y, 'r--', label='Desired')
plt.plot(vehicleState_x[0],vehicleState_y[0],'ro')
plt.plot(vehicleState_x[len(vehicleState_x)-1],vehicleState_y[len(vehicleState_x)-1],'rx')
plt.plot(vehicleState_act_x,vehicleState_act_y, 'b-', label='Actual')
plt.plot(vehicleState_act_x[0],vehicleState_act_y[0],'bo')
plt.plot(vehicleState_act_x[len(vehicleState_act_x)-1],vehicleState_act_y[len(vehicleState_act_x)-1],'bx')
plt.legend()
plt.xlabel("x [m]")
plt.ylabel("y [m]")

plt.figure(2)
plt.title('Robot commands')
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_linearvelocity, 'b-')
plt.xlabel("Time [s]")
plt.ylabel("Linear velocity [m/s]")
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_angularvelocity, 'b-')
plt.xlabel("Time [s]")
plt.ylabel("Angular velocity [rad/s]")

plt.figure(3)
plt.title('Robot state')
plt.subplot(311)
plt.plot(vehicleState_time,vehicleState_x, 'r--', label='Desired')
plt.plot(vehicleState_time,vehicleState_act_x, 'b-', label='Actual')
plt.xlabel("Time [s]")
plt.ylabel("x [m]")
plt.legend()
plt.subplot(312)
plt.plot(vehicleState_time,vehicleState_y, 'r--', label='Desired')
plt.plot(vehicleState_time,vehicleState_act_y, 'b-', label='Actual')
plt.xlabel("Time [s]")
plt.ylabel("y [m]")
plt.legend()
plt.subplot(313)
plt.plot(vehicleState_time,vehicleState_theta, 'r--', label='Desired')
plt.plot(vehicleState_time,vehicleState_act_theta, 'b-', label='Actual')
plt.xlabel("Time [s]")
plt.ylabel("theta [rad]")
plt.xlabel("Time [s]")
plt.ylabel("theta [rad]")
plt.legend()

plt.show()

