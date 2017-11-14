#!/usr/bin/env python
from learningfuncs import *
import matplotlib.pyplot as plt

# roscore
# rosrun fw_wrapper srv_wrapper
# rosrun eecs301_grp_c asnX.py


# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")

    # Sensor setup
    IR1 = 6 # left
    IR2 = 3 # right
    DMS = 3 # front
    SENSORS = (IR1, IR2, DMS)

    # control loop running at X Hz
    r = rospy.Rate(10000) # 10000hz

    # select mode to run
    print "Select one:"
    print "0. check sensors"
    print "1. testing mode"
    selection = raw_input(">>> ")

    if selection == "0":
        f = False

        while not rospy.is_shutdown():
            if f:
                moveMotor(6, 5)
                f = False
            else:
                moveMotor(6, 0)
                f = True
            viewSensors(SENSORS);

        sys.exit()

    elif selection == "1":
        print fastSweep(SENSORS)

    else:
        print "Invalid selection '" + selection + "' Quiting..."
        sys.exit()