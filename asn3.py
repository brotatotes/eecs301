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

    data_files = {"data1": "data1.csv", "data2": "data2.csv", "data3": "data3.csv"}

    train_test_sizes = {"test_size": 0.2, "train_size": 0.8}

    context = {"sensors": sensors, "motors": motors, "data_files": data_files, "max_n": 9, "train_test_sizes": train_test_sizes}

    asn3 = Asn3(context)

    # control loop running at X Hz
    r = rospy.Rate(10000) # 10000hz


    setMotorMode(bot_motor, 0)

    # select mode to run
    print "Select one:"
    print "0. check sensors"
    print "1. test mode"
    print "2. data collection"
    print "3. asn3 orientation demo"
    print "4. asn3 PID demo"
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
        # orientation_adc = 514
        # a = asn3.collect_datum(orientation_adc)
        # b = asn3.collect_datum(orientation_adc)
        # c = [np.mean((a[i], b[i])) for i in range(len(a))]
        raw_input("First, I need to calibrate. This will take a few seconds. Hit Enter to confirm.")
        
        print "Calibrating..."
        asn3.calibrate()
        print "Done!"

        while True:
            setMotorMode(bot_motor, 1)
            raw_input("Turn robot to a desired orientation. Hit Enter when done.")

            orientation_adc = getMotorPositionCommand(bot_motor)
            print "You've turned it to", (orientation_adc -359) / 3.41, "degrees."

            if orientation_adc < 359 or orientation_adc > 665:
                print "This is out of range, I'll adjust it."
                setMotorMode(bot_motor, 0)
                if orientation_adc < 359:
                    setMotorTargetPositionCommand(bot_motor, 359)
                else:
                    setMotorTargetPositionCommand(bot_motor, 665)

                time.sleep(0.5)

                orientation_adc = getMotorPositionCommand(bot_motor)
                print "Adjusted to", (orientation_adc -359) / 3.41, "degrees."


            raw_input("Now I'm going to use machine learning to figure out my orientation. Hit Enter to begin.")

            res = 0
            while res < 359 or res > 665:
                vals = asn3.sweep()
                print vals
                res = asn3.learner.compute_result(vals)
                if res < 359 or res > 665:
                    print "Result out of range. Re-scanning."
            print "I think I'm at", (res - 359) / 3.41, "degrees and I am actually at", (getMotorPositionCommand(bot_motor) - 359) / 3.41
            print

    elif selection == "4":
        raw_input("First, I need to calibrate. This will take a few seconds. Hit Enter to confirm.")
        
        print "Calibrating..."
        asn3.calibrate()
        print "Done!"

        setMotorMode(bot_motor, 1)
        raw_input("Turn robot to a desired orientation. Hit Enter when done.")

        orientation_adc = getMotorPositionCommand(bot_motor)
        print "You've turned it to", orientation_adc, "adc."

        if orientation_adc < 359 or orientation_adc > 665:
            print "This is out of range, I'll adjust it."
            setMotorMode(bot_motor, 0)
            if orientation_adc < 359:
                setMotorTargetPositionCommand(bot_motor, 359)
            else:
                setMotorTargetPositionCommand(bot_motor, 665)

            time.sleep(0.5)

            orientation_adc = getMotorPositionCommand(bot_motor)
            print "Adjusted to", (orientation_adc -359) / 3.41, "degrees."

        angle = input("What is your target angle (0-90)? ")
        if angle < 0 or angle > 90:
            print "This is out of range, I'll adjust it."
            if angle < 0:
                angle = 0
            else:
                angle = 90

            print "Adjusted to", angle, "degrees"

        orientation_adc = int((angle * 3.41) + 359)
        print "You selected", angle, "degrees, which is", (orientation_adc -359) / 3.41, "degrees."

        prev = None
        serrs = 0
        a = 1
        b = 0
        c = 0

        while True:
            adc = 0
            while adc < 359 or adc > 665:
                vals = asn3.sweep()
                print vals
                adc = asn3.learner.compute_result(vals)
                if adc < 359 or adc > 665:
                    print "Result out of range. Re-scanning."
            print "guess:", (adc -359) / 3.41
            print "actual:", (getMotorPositionCommand(bot_motor) - 459) / 3.41

            err = orientation_adc - adc
            if prev == None:
                prev = err
            diff = err - prev
            prev = err

            serrs += err

            effort = a * err + b * diff + c * serrs

            print "effort:", effort
            print

            setMotorMode(bot_motor, 0)
            setMotorTargetPositionCommand(bot_motor, int(adc + effort))

    else:
        print "Invalid selection '" + selection + "' Quiting..."
        sys.exit()
