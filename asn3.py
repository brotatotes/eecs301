#!/usr/bin/env python
from asnfuncs import *
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
    ir1 = 6
    dms = 3

    sensors = {"ir1":ir1, "dms": dms}

    top_motor = 9
    bot_motor = 1

    motors = {"top_motor": top_motor, "bot_motor":bot_motor}

    data_file = "data.csv"

    context = {"sensors": sensors, "motors": motors, "data_file": data_file, "max_n": 10}

    asn3 = Asn(context)

    # control loop running at X Hz
    r = rospy.Rate(10000) # 10000hz

    # select mode to run
    print "Select one:"
    print "0. check sensors"
    print "1. testing mode"
    print "2. data collection"
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
            viewSensors(sensors);

        sys.exit()

    elif selection == "1":
        pass

    elif selection == "2":
        pass

    else:
        print "Invalid selection '" + selection + "' Quiting..."
        sys.exit()