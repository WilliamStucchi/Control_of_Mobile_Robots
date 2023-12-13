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
vehicleState_linearvelocity = []
vehicleState_angularvelocity = []

# State published by the controller
controllerState_time = []
controllerState_xref = []
controllerState_yref = []
controllerState_xPref = []
controllerState_yPref = []
controllerState_xP = []
controllerState_yP = []
controllerState_vPx = []
controllerState_vPy = []
controllerState_linearvelocity = []
controllerState_angularvelocity = []
controllerState_xPerr = []
controllerState_yPerr = []

for topic, msg, t in bag.read_messages():
    if topic == "/state":
        vehicleState_time.append(msg.data[0])
        vehicleState_x.append(msg.data[1])
        vehicleState_y.append(msg.data[2])
        vehicleState_theta.append(msg.data[3])
        vehicleState_linearvelocity.append(msg.data[4])
        vehicleState_angularvelocity.append(msg.data[5])

    if topic == "/controller_state":
        controllerState_time.append(msg.data[0])
        controllerState_xref.append(msg.data[1])
        controllerState_yref.append(msg.data[2])
        controllerState_xPref.append(msg.data[3])
        controllerState_yPref.append(msg.data[4])
        controllerState_xP.append(msg.data[5])
        controllerState_yP.append(msg.data[6])
        controllerState_vPx.append(msg.data[7])
        controllerState_vPy.append(msg.data[8])
        controllerState_linearvelocity.append(msg.data[9])
        controllerState_angularvelocity.append(msg.data[10])
        controllerState_xPerr.append(msg.data[3]-msg.data[5])
        controllerState_yPerr.append(msg.data[4]-msg.data[6])

bag.close()

# Plot data
plt.figure(1)
plt.plot(vehicleState_x,vehicleState_y, 'b' ,label="robot controlled")
plt.plot(vehicleState_x[0],vehicleState_y[0],'ro')
plt.plot(vehicleState_x[len(vehicleState_x)-1],vehicleState_y[len(vehicleState_x)-1],'rx')
plt.plot(controllerState_xref,controllerState_yref,'r--', label="desired")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.legend(loc="best")

plt.figure(2)
plt.subplot(211)
plt.plot(vehicleState_time,vehicleState_linearvelocity, 'b')
plt.xlabel("Time [s]")
plt.ylabel("Linear velocity [m/s]")
plt.subplot(212)
plt.plot(vehicleState_time,vehicleState_angularvelocity, 'b')
plt.xlabel("Time [s]")
plt.ylabel("Angular velocity [rad/s]")

plt.figure(3)
plt.subplot(211)
plt.plot(controllerState_time,controllerState_xPerr, 'b')
plt.xlabel("Time [s]")
plt.ylabel("x position error [m]")
plt.subplot(212)
plt.plot(controllerState_time,controllerState_yPerr, 'b')
plt.xlabel("Time [s]")
plt.ylabel("y position error [m]")

plt.show()

