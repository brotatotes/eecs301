#!/usr/bin/env python
from asnfuncs import *

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
    
    # select mode to run
    print("Select one:")
    print("1. capture mode")
    print("2. asn0")
    print("3. asn1")
    print("4. test")
    selection = raw_input(">>> ")
    
    asn0 = asn1 = test = False
    
    # setup for each  mode
    if selection == "1":
        releaseMotors()
        s = raw_input("What is the name of this state? ")
        captureState(s)
        print(s, "saved")
        engageMotors()
        sys.exit()

    elif selection == "2":
        asn0 = True
        irs = [0] * 5
        threshold = 50
        step = 0
        objectSeen = False
        direction = "clockwise"

    elif selection == "3":
        asn1 = True

        nvals = 5

        LEFT_WALL = RIGHT_WALL = REACT = False

        action = raw_input("Select Action:\n1. Follow wall on left.\n2. Follow wall on right.\n3. Reactive control.\n>>>")
        if action == "1":
            LEFT_WALL = True
        elif action == "2":
            RIGHT_WALL = True
        elif action == "3":
            REACT = True
        else:
            print("Invalid selection '" + action + "' Quiting...")
            sys.exit()

        leftVals = [0] * nvals
        rightVals = [0] * nvals
        frontVals = [0] * nvals

        go_to_default()

        raw_input("enter loop? ")

    elif selection == "4": # testing area
        test = True

        raw_input("loop? ")

    else:
        print("Invalid selection '" + selection + "' Quiting...")
        sys.exit()
    
    # Sensor setup
    IR1 = 2 # left
    IR2 = 3 # right
    DMS = 1 # front
    SENSORS = (IR1, IR2, DMS)

    # IR 1 is port 1
    # IR 2 is port 2

    # control loop running at X Hz
    r = rospy.Rate(1000) # 1000hz
    go_to_default()
    f = False

    go_to_default()
    
    while not rospy.is_shutdown():
        
        # loop for each mode
        if asn0:
            irs[:len(irs)-1] = irs[1:]
            irs[-1] = getSensorValue(IR1)
            if all(map(lambda x: x > threshold, irs)):
                doBehavior1()

            else:
                if getSensorValue(DMS) > 1000:
                    direction = "clockwise" if direction != "clockwise" else "counterclockwise"       
            
                step += 1
                turn(step, direction)

        elif asn1:
            leftVals = [adc_to_cm(1, getSensorValue(IR1)) for _ in range(nvals)]
            rightVals = [adc_to_cm(2, getSensorValue(IR2)) for _ in range(nvals)]
            frontVals = [adc_to_cm(0, getSensorValue(DMS)) for _ in range(nvals)]

            if not (LEFT_WALL or RIGHT_WALL or REACT):
                LEFT_WALL = True
            # figure out left_wall or right_wall or react:


            if RIGHT_WALL:
                go_to_default()
                ## follow left wall
                # actionPlan = detectObstacles((leftVals, rightVals, frontVals))

                print("leftVals:", leftVals)
                print("rightVals:", rightVals)
                print("frontVals:", frontVals)
                # print("actionPlan:", actionPlan)
                # print 

                if toofar(rightVals, 20):    
                    fineright()
                    forward()
                elif tooclose(rightVals, 13):
                    fineleft()
                    forward()
                else:
                    forward()

            elif LEFT_WALL:
                go_to_default()

                # actionPlan = detectObstacles((leftVals, rightVals, frontVals))

                print("leftVals:", leftVals)
                print("rightVals:", rightVals)
                print("frontVals:", frontVals)
                # print("actionPlan:", actionPlan)
                # print 

                if toofar(leftVals, 20):    
                    fineleft()
                    rightforward()
                elif tooclose(leftVals, 12):
                    fineright()
                    rightforward()
                else:
                    rightforward()

            elif REACT:
                go_to_default()
                actionPlan = detectObstacles((leftVals, rightVals, frontVals))

                print("leftVals:", leftVals)
                print("rightVals:", rightVals)
                print("frontVals:", frontVals)
                print("actionPlan:", actionPlan)
                print 

                if actionPlan == "180":
                    turn180()
                elif actionPlan == "R90":
                    turn90()
                elif actionPlan == "L90":
                    turn90("counterclockwise")
                else:
                    forward()


        elif test:
            # viewSensors(SENSORS)
            if f:
                moveMotor(6, 5)
                f = False
            else:
                moveMotor(6, 0)
                f = True
            # for port in range(1,7):
            #     print(port, getSensorValue(port))
            viewSensors(SENSORS);

            # turn180()
            # raw_input()
            
        # sleep to enforce loop rate
        # r.sleep()
        

        
    
        

