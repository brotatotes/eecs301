#!/usr/bin/env python
from asnfuncs import *
from map import *
import time

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
    DMS = 1 # front
    SENSORS = (IR1, IR2, DMS)

    # IR 1 is port 1
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

        print("Creating map...")
        MAP = EECSMap()
        MAP.printObstacleMap()

        switch('x')

        raw_input("loop? ")

    else:
        print("Invalid selection '" + selection + "' Quiting...")
        sys.exit()


    for i in range(1,9):
        moveMotor(i, 0)


    start = time.time()


    xToY()
    drive(0, 'y')
    drive(0, 'y')
    drive(0, 'y')

    # while not rospy.is_shutdown() or time.time() - start > 5000:
        


        # sleep to enforce loop rate
        # r.sleep()

        # xToY()
        # raw_input()
        # yToX()
        # raw_input()

        # drive(1)
        # raw_input()

        


        
    
        

