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
    ir1 = 3
    dms = 4

    sensors = {"ir1":ir1, "dms": dms}

    top_motor = 9
    bot_motor = 1

    motors = {"top_motor": top_motor, "bot_motor":bot_motor}

    data_file = "data.csv"

    train_test_sizes = {"test_size": 0.2, "train_size": 0.8}

    context = {"sensors": sensors, "motors": motors, "data_file": data_file, "max_n": 9, "train_test_sizes": train_test_sizes}

    asn3 = Asn3(context)

    # control loop running at X Hz
    r = rospy.Rate(10000) # 10000hz

    # select mode to run
    print "Select one:"
    print "0. check sensors"
    print "1. test mode"
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
        asn3.collect_datum(512)

    elif selection == "2":
        while True:
            asn3.collect_data()

    else:
        print "Invalid selection '" + selection + "' Quiting..."
        sys.exit()
