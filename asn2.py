#!/usr/bin/env python
import time
from asnfuncs import *
from map import *
import matplotlib.pyplot as plt

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
    IR1 = 6 # left
    IR2 = 3 # right
    DMS = 3 # front
    SENSORS = (IR1, IR2, DMS)

    # IR 1 is port 19,
    # IR 2 is port 2

    # control loop running at X Hz
    r = rospy.Rate(10000) # 10000hz

    # select mode to run
    print("Select one:")
    print("0. asn2")
    print("1. capture mode")
    print("2. check sensors")
    print("3. testing mode")
    print("4. Wander mode")
    print("5. adjust test")    
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
        starth = input("Select start heading:\n1. North\n2. East\n3. South\n4. West\n(Select 1-4): ")
        print "You selected", starth
        if not starth in [1,2,3,4]:
            print "Failed. Please input a number 1 - 4"
            print "Quitting..."
            sys.exit()

        elif starth in [1,3]:
            s = 'Y'
        else:
            s = 'X'

        switch(s)
        STATE = s

        endh = input("Select end heading:\n1. North\n2. East\n3. South\n4. West\n(Select 1-4): ")
        print "You selected", endh
        if not endh in [1,2,3,4]:
            print "Failed. Please input a number 1 - 4"
            print "Quitting..."
            sys.exit()

        elif endh in [1,3]:
            e = 'Y'
        else:
            e = 'X'

        print "Enter start coordinates (0-7):"
        x = input("Enter i coordinate: ")
        print "You selected", x        
        y = input("Enter j coordinate: ")
        print "You selected", y

        print "start position = (" + str(x) + ", " + str(y) + ")\n"

        if x < 0 or x > 7 or y < 0 or y > 7:
            print "Coordinates out of range."
            print "Quitting..."
            sys.exit()

        start = (x,y)

        print "Enter end coordinates (0-7):"
        x = input("Enter i coordinate: ")
        print "You selected", x
        y = input("Enter j coordinate: ")
        print "You selected", y

        print "end position = (" + str(x) + ", " + str(y) + ")\n"

        if x < 0 or x > 7 or y < 0 or y > 7:
            print "Coordinates out of range."
            print "Quitting..."
            sys.exit()

        end = (x,y)

        m = EECSMap()
        print "Setup done!"
        
        
        raw_input("Ready to go? ")
        STATE = findAndDrivePath(m, start, end, STATE)
        
        if e == 'X':
            yToX()
        else:
            xToY()

    elif selection == "3":
        # temp=[]
        # setMotorTargetSpeed(9,1023)
        # setMotorTargetPositionCommand(9,400)
        # time.sleep(2)
        # for i in range(0,1024,20):
        #     setMotorTargetPositionCommand(9,i)
        #     # time.sleep(1)
        #     temp.append((getMotorPositionCommand(9), getSensorValue(DMS), getSensorValue(IR1)))
        #     # time.sleep(1)

        # # setMotorTargetPositionCommand(9,1023)
        # # time.sleep(0.1)
        # # temp.append((getMotorPositionCommand(9), getSensorValue(DMS)))

        # for t in temp:
        #     print t
        

        # plt.scatter([adc_to_deg(1,t[0]) for t in temp], [adc_to_cm(0, t[1]) for t in temp], c='b', label='dms')
        # plt.scatter([adc_to_deg(1,t[0]) - 170 if adc_to_deg(1,t[0]) > 0 else adc_to_deg(1,t[0]) + 190 for t in temp], [adc_to_cm(1, t[2]) for t in temp], c='r', label='ir1')

        avgs = sweepSensors(SENSORS)
        plt.scatter(avgs.keys(), avgs.values())
        plt.show()
        # while True:
        #     viewSensors(SENSORS)
        print(detectWalls(avgs))


    elif selection == "4":
        starth = input("Select start heading:\n1. North\n2. East\n3. South\n4. West\n(Select 1-4): ")
        print "You selected", starth
        if not starth in [1,2,3,4]:
            print "Failed. Please input a number 1 - 4"
            print "Quitting..."
            sys.exit()

        elif starth in [1,3]:
            s = 'Y'
        else:
            s = 'X'

        switch(s)
        STATE = s

        print "Enter start coordinates (0-7):"
        x = input("Enter i coordinate: ")
        print "You selected", x        
        y = input("Enter j coordinate: ")
        print "You selected", y

        print "start position = (" + str(x) + ", " + str(y) + ")\n"

        if x < 0 or x > 7 or y < 0 or y > 7:
            print "Coordinates out of range."
            print "Quitting..."
            sys.exit()

        raw_input("Ready to go? ")

        start = (x,y)
        STATE = wander(SENSORS,STATE, start)

    elif selection == "5":
        state = 'X'
        switch(state)
        drive('w', state)
        drive('w', state)
        drive('w', state)
        drive('w', state)

                

    else:
        print("Invalid selection '" + selection + "' Quiting...")
        sys.exit()




    
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
