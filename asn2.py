#!/usr/bin/env python
import time
from asnfuncs import *
from map import *

# roscore
# rosrun fw_wrapper srv_wrapper
# rosrun eecs301_grp_c asnX.py


# Main function
if __name__ == "__main__":
    rospy.init_node('example_node', anonymous=True)
    rospy.loginfo("Starting Group X Control Node...")

    # load states
    states = pickle.load( open("states.p", "rb"))
    print("States loaded:", states.keys())

    stopDrive()

    # Sensor setup
    IR1 = 2 # left
    IR2 = 3 # right
    DMS = 3 # front
    SENSORS = (IR1, IR2, DMS)

    # IR 1 is port 19,
    # IR 2 is port 2

    # control loop running at X Hz
    r = rospy.Rate(1000) # 1000hz

    # select mode to run
    print("Select one:")
    print("0. asn2")
    print("1. capture mode")
    print("2. check sensors")

    selection = raw_input(">>> ")

    # setup for each  mode
    if selection == "1":
        releaseMotors()
        s = raw_input("What is the name of this state? ")
        captureState(s)
        print(s, "saved")
        engageMotors()
        sys.exit()

    elif selection == "2":
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

    elif selection == "0" or selection == "":
        # setup
        heading = input("Select heading:\n1. North\n2. East\n3. South\n4. West\n(Select 1-4): ")
        if not heading in [1,2,3,4]:
            print("Failed. Please input a number 1 - 4")
            print("Quitting...")
            sys.exit()

        elif heading in [1,3]:
            s = 'Y'
        else:
            s = 'X'

        switch(s)
        STATE = s

        print("Enter start coordinates (0-7):")
        x = input("Enter i coordinate: ")
        y = input("Enter j coordinate: ")
        if x < 0 or x > 7 or y < 0 or y > 7:
            print("Coordinates out of range.")
            print("Quitting...")
            sys.exit()

        start = (x,y)

        print("Enter end coordinates (0-7):")
        x = input("Enter i coordinate: ")
        y = input("Enter j coordinate: ")
        if x < 0 or x > 7 or y < 0 or y > 7:
            print("Coordinates out of range.")
            print("Quitting...")
            sys.exit()

        end = (x,y)

        m EECSMap()
        print("Setup done!")
        
        
        raw_input("Ready to go? ")
        STATE = findAndDrivePath(m, (0,6), (1,6), STATE)

    else:
        print("Invalid selection '" + selection + "' Quiting...")
        sys.exit()




    m = EECSMap()
    
    # STATE = drive('s', STATE)
    # STATE = drive('s', STATE)
    # STATE = drive('s', STATE)
    # STATE = drive('s', STATE)

    # setMotorMode(9,0)
    # setMotorTargetPositionCommand(9,0)

    # while not rospy.is_shutdown():



        # sleep to enforce loop rate
        # r.sleep()

        # xToY()
        # raw_input("start yToX? ")
        # yToX()
        # raw_input("start xToY? ")

        # drive(1)
        # raw_input()
