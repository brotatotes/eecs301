#!/usr/bin/env python
from asnfuncs import *
from learningfuncs import *
import matplotlib.pyplot as plt
import numpy as np

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

    data_files = {"data1": "data1.csv", "data2": "data2.csv"}

    train_test_sizes = {"test_size": 0.2, "train_size": 0.8}

    context = {"sensors": sensors, "motors": motors, "data_files": data_files, "max_n": 9, "train_test_sizes": train_test_sizes}

    asn3 = Asn3(context)
    asn3l = Asn3Learner(context)

    # control loop running at X Hz
    r = rospy.Rate(10000) # 10000hz

    # select mode to run
    print "Select one:"
    print "0. check sensors"
    print "1. test mode"
    print "2. data collection"
    print "3. calibrate"
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

    elif selection == "3":
        orientation_adc = 514
        a = asn3.collect_datum(orientation_adc)
        b = asn3.collect_datum(orientation_adc)
        c = [np.mean((a[i], b[i])) for i in range(len(a))]
        asn3l.calibrate(c, orientation_adc)


    else:
        print "Invalid selection '" + selection + "' Quiting..."
        sys.exit()
