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
vehicleState_yawrate = []
vehicleState_vy = []
vehicleState_ay = []
vehicleState_sideslip = []
vehicleState_slipFront = []
vehicleState_slipRear = []
vehicleState_forceFront = []
vehicleState_forceRear = []
vehicleState_velocity = []
vehicleState_steer = []

for topic, msg, t in bag.read_messages():
    if topic == "/car_state":
        vehicleState_time.append(msg.data[0])
        vehicleState_x.append(msg.data[1])
        vehicleState_y.append(msg.data[2])
        vehicleState_theta.append(msg.data[3])
        vehicleState_yawrate.append(msg.data[4])
        vehicleState_vy.append(msg.data[5])
        vehicleState_ay.append(msg.data[6])
        vehicleState_sideslip.append(msg.data[7])
        vehicleState_slipFront.append(msg.data[8])
        vehicleState_slipRear.append(msg.data[9])
        vehicleState_forceFront.append(msg.data[10])
        vehicleState_forceRear.append(msg.data[11])
        vehicleState_velocity.append(msg.data[12])
        vehicleState_steer.append(msg.data[13])

bag.close()

# Plot data
plt.figure(1)
plt.plot(vehicleState_x,vehicleState_y)
plt.plot(vehicleState_x[0],vehicleState_y[0],'ro')
plt.plot(vehicleState_x[len(vehicleState_x)-1],vehicleState_y[len(vehicleState_x)-1],'rx')
plt.xlabel("x [m]")
plt.ylabel("y [m]")

plt.figure(2)
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_steer)
plt.xlabel("Time [s]")
plt.ylabel("Steer act [rad]")
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_velocity)
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m/s]")

plt.figure(3)
plt.subplot(311)
plt.plot(vehicleState_time,vehicleState_x)
plt.xlabel("Time [s]")
plt.ylabel("x [m]")
plt.subplot(312)
plt.plot(vehicleState_time,vehicleState_y)
plt.xlabel("Time [s]")
plt.ylabel("y [m]")
plt.subplot(313)
plt.plot(vehicleState_time,vehicleState_theta)
plt.xlabel("Time [s]")
plt.ylabel("theta [rad]")

plt.figure(4)
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_forceFront)
plt.xlabel("Time [s]")
plt.ylabel("Front lateral force [N]")
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_forceRear)
plt.xlabel("Time [s]")
plt.ylabel("Rear lateral force [N]")

plt.show()

