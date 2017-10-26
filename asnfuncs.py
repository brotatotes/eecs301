#!/usr/bin/env python
import roslib, rospy, pickle, sys, math, time
from fw_wrapper.srv import *
from map import *

# -----------SERVICE DEFINITION-----------
# allcmd REQUEST DATA
# ---------
# string command_type
# int8 device_id
# int16 target_val
# int8 n_dev
# int8[] dev_ids
# int16[] target_vals

# allcmd RESPONSE DATA
# ---------
# int16 val
# --------END SERVICE DEFINITION----------

# ----------COMMAND TYPE LIST-------------
# GetMotorTargetPosition
# GetMotorCurrentPosition
# GetIsMotorMoving
# GetSensorValue
# GetMotorWheelSpeed
# SetMotorTargetPosition
# SetMotorTargetSpeed
# SetMotorTargetPositionsSync
# SetMotorMode
# SetMotorWheelSpeed

# wrapper function to call service to set a motor mode
# 0 = set target positions, 1 = set wheel moving
def setMotorMode(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorMode', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get motor wheel speed
def getMotorWheelSpeed(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorWheelSpeed', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor wheel speed
def setMotorWheelSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorWheelSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set motor target speed
def setMotorTargetSpeed(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetSpeed', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to SetMotorTargetPositionsSync
def setMotorTargetPositionsSync(n, motor_ids, target_vals):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetPositionsSync', 0, 0, n, motor_ids, target_vals)
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def setWheelSpeedSync(n, motor_ids, target_vals):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetWheelSpeedSync', 0, 0, n, motor_ids, target_vals)
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


# wrapper function to call service to get sensor value
def getSensorValue(port):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetSensorValue', port, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to set a motor target position
def setMotorTargetPositionCommand(motor_id, target_val):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('SetMotorTargetPosition', motor_id, target_val, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to get a motor's current position
def getMotorPositionCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetMotorCurrentPosition', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# wrapper function to call service to check if a motor is currently moving
def getIsMotorMovingCommand(motor_id):
    rospy.wait_for_service('allcmd')
    try:
        send_command = rospy.ServiceProxy('allcmd', allcmd)
        resp1 = send_command('GetIsMotorMoving', motor_id, 0, 0, [0], [0])
        return resp1.val
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#################################### Motor Handling ####################################

def moveMotor(motor_id, deg):
    # thighs: top down counter-clockwise
    # legs: down, up
    motor_ranges = {1:(160,720), 2:(280,850), 3:(280,850), 4:(160,720),
                    5:(160,820), 6:(860,200), 7:(860,200), 8:(160,820)}

    if not motor_id in range(1,9):
        raise Exception("Unrecognized Motor ID.")

    adc = deg_to_adc(motor_id, deg)

    r = motor_ranges[motor_id]

    current_pos = getMotorPositionCommand(motor_id)

    if adc >= 0:
        if r[0] < r[1]:
            d = min(adc, r[1])
        else:
            d = max(adc, r[1])
    else:
        if r[0] < r[1]:
            d = max(adc, r[0])
        else:
            d = min(adc, r[0])

    print("Attempting to move from", current_pos, "to", d, "within", r)

    return setMotorTargetPositionCommand(motor_id, d)

    print("Motor move completed.")

def deg_to_adc(motor_id, deg):
    if motor_id in (5,8):
        return 3.41 * deg + 205
    elif motor_id in (6,7):
        return -3.41 * deg + 818
    elif motor_id in (1,2,3,4):
        return 3.41 * deg + 512
    else:
        raise Exception("Unrecognized Motor ID.")

def adc_to_deg(motor_id, adc):
    if motor_id in (5,8):
        return (adc - 205) / 3.41
    elif motor_id in (6,7):
        return (adc - 818) / -3.41
    elif motor_id in (1,2,3,4):
        return (adc - 512) / 3.41
    else:
        raise Exception("Unrecognized Motor ID.")

def deg_to_adc_l(l):
    if len(l) != 8:
        raise Exception("Need 8 entries")
    return [deg_to_adc(i+1, d) for i,d in enumerate(l)]

def adc_to_deg_l(l):
    if len(l) != 8:
        raise Exception("Need 8 entries")
    return [adc_to_deg(i+1, d) for i,d in enumerate(l)]

def releaseMotors():
    for i in range(1,9) + range(11,15):
        setMotorMode(i,1)

def engageMotors():
    for i in range(1,9) + range(11,15):
        setMotorMode(i,0)

#################################### Wheel Handling #####################################

def drive(direction, state):
    # we don't know what 0 direction means yet
    for i in range(11,15):
        setMotorMode(i, 1)
    d = direction.lower()
    if d == "n":
        if state == 'X':
            xToY()
            state = 'Y'
        speeds = [1023, 2047, 1023, 2047]
        setWheelSpeedSync(4, range(11,15), speeds)
        time.sleep(1.728)
    elif d == "s":
        if state == 'X':
            xToY()
            state = 'Y'
        speeds = [2047, 1023, 2047, 1023]
        setWheelSpeedSync(4, range(11,15), speeds)
        time.sleep(1.69)
    elif d == "w":
        if state == 'Y':
            yToX()
            state = 'X'
        speeds = [1023, 1023, 2047, 2047]
        setWheelSpeedSync(4, range(11,15), speeds)
        time.sleep(1.73)
    elif d == "e":
        if state == 'Y':
            yToX()
            state = 'X'
        speeds = [2047, 2047, 1023, 1023]
        setWheelSpeedSync(4, range(11,15), speeds)
        time.sleep(1.7)


    stopDrive()

    return state

def drivePath(path, state):
    mapping = ['', 'n', 'e', 's', 'w']
    for p in path:
        state = drive(mapping[p], state)


def wheelTurn():
    speeds = [1023] * 4

    for i in range(11,15):
        setMotorMode(i, 1)

    setWheelSpeedSync(4, range(11,15), speeds)
    time.sleep(2)
    stopDrive()

def xToY():

    # raw_input()
    moveMotor(7, 15)
    moveMotor(3, 90)
    moveMotor(7, 0)

    # raw_input()
    moveMotor(6, 15)
    moveMotor(2, 90)
    moveMotor(6, 0)

    # raw_input()
    moveMotor(8, 15)
    moveMotor(4, -90)
    # raw_input()

    moveMotor(5, 15)
    moveMotor(1, -90)
    moveMotor(5, 0)
    moveMotor(8, 0)



def yToX():
    moveMotor(6, 15)
    moveMotor(2, -2)
    moveMotor(6, 0)

    moveMotor(7, 15)
    moveMotor(3, -2)
    moveMotor(7, 0)

    moveMotor(5, 15)
    moveMotor(1, 0)
    moveMotor(5, 0)

    moveMotor(8, 15)
    moveMotor(4, 0)
    moveMotor(8, 0)

def switch(state = "X"):

    if state == "X":
        targets = [0,-2,-2,0]+[0] * 4
    else:
        targets = [-90, 90, 90, -90] + [0] *4

    for i in range(1,9):
        # setMotorWheelSpeed(i, 1000)
        # moveMotor(i, targets[i-1])
        # time.sleep(1)
        targets[i-1] = deg_to_adc(i, targets[i-1])

    setMotorTargetPositionsSync(8, range(1,9), targets)



def stopDrive():
    setWheelSpeedSync(4, range(11,15), [0] * 4)

##################################### Map Handling ######################################
def getNeighborCoord(i, j, dir):
    if dir == DIRECTION.North:
        return (i-1,j)
    elif dir == DIRECTION.South:
        return (i+1,j)
    elif dir == DIRECTION.West:
        return (i, j-1)
    elif dir == DIRECTION.East:
        return (i, j+1)

    raise Exception("Invalid direction given.")

def inRange(i, j):
    return (i >= 0 and i <= 7 and j >= 0 and j <= 7)

def buildCostMap(m, target):
    xSize, ySize = m.getCostmapSize(True), m.getCostmapSize(False)
    xRange, yRange = range(xSize), range(ySize)

    if not (target[0] in xRange and target[1] in yRange):
        raise Exception("target postion out of range.")

    # reset cost map to infinity
    for i in xrange(8):
        for j in xrange(8):
            m.setCost(i,j,float("inf"))

    # initialize target cost to 0.
    m.setCost(target[0], target[1], 0)
    visited = set([(target[0], target[1])])
    queue = [target]
    while queue:
        tile = queue.pop(0)
        for direc in range(1,5):
            newTile = getNeighborCoord(tile[0], tile[1], direc)
            if inRange(newTile[0], newTile[1]) and not newTile in visited and m.getNeighborObstacle(tile[0], tile[1], direc) == 0:
                m.setNeighborCost(tile[0], tile[1], direc, min(m.getCost(tile[0], tile[1]) + 1, m.getNeighborCost(tile[0], tile[1], direc)))
                queue.append(newTile)
                visited.add((tile[0], tile[1]))

def newMap():
    m = EECSMap()
    m.clearObstacleMap()
    return m

def findAndDrivePath(m, start, target, state):
    paths = findPath(m, start, target)
    m.printObstacleMap()
    m.printCostMap()
    print("Generated paths:", paths)
    # raw_input("drive? ")
    # start = time.time()
    state = drivePath(paths, state)
    return state

    # print((time.time() - start))

def findPath(eecsmap, start, target):
    xSize, ySize = eecsmap.getCostmapSize(True), eecsmap.getCostmapSize(False)
    xRange, yRange = range(xSize), range(ySize)

    if not (start[0] in xRange and start[1] in yRange):
        raise Exception("target postion out of range.")

    if not (target[0] in xRange and target[1] in yRange):
        raise Exception("target postion out of range.")

    buildCostMap(eecsmap, target)

    path = []
    curr = start
    while curr != target:
        nexts = []
        for direc in range(1,5):
            potential = getNeighborCoord(curr[0], curr[1], direc)
            if inRange(potential[0], potential[1]) and eecsmap.getNeighborObstacle(curr[0], curr[1], direc) == 0:
                nexts.append((potential, direc))
        best = min(nexts, key=lambda n: eecsmap.getCost(n[0][0], n[0][1]))
        # best should never be worse than curr
        path.append(best[1])
        curr = best[0]

    return path

def updateWalls(eecsmap, walls):
    if walls["north"]:
        eecsmap.setObstacle(start[0], start[1], Direction.North)

    if walls["south"]:
        eecsmap.setObstacle(start[0], start[1], Direction.South)

    if walls["west"]:
        eecsmap.setObstacle(start[0], start[1], Direction.West)

    if walls["east"]:
        eecsmap.setObstacle(start[0], start[1], Direction.East)


def wander(sensors, state, start = (0,0)):
    m = EECSMap()
    m.clearObstacleMap()
    m.clearCostMap()

    curr = start

    visited = set(start)

    targets = set()

    # initialize targets
    for direc in range(1,5):
        potential = getNeighborCoord(start[0], start[1], direc)
        if inRange(start[0], start[1]):
            targets.add(potential)    

    while targets:

        

        # find closest
        best_cost = float("inf")
        best_target = None

        # grab all from targets 
        for t in targets:
            buildCostMap(m, t)
            cost = m.getCost(start[0], start[1])
            if cost > best_cost:
                best_cost = cost
                best_target = t

        if not best_target:
            return

        # go to closest
        if curr != start:
            path = findPath(m, start, best_target)
            state = drive(path, state)

        # update curr
        curr = best_target
       

        # remove that from targets


        

        # update targets

        # update visited

        # sweep, update walls
        walls = detectWalls(sensors)
        updateWalls(m, walls)





#################################### Sensor Handling #####################################
def get_sensor_consts(sensor):
    if sensor == 1:
        a, b = -7.4689, -0.1898
    elif sensor == 2:
        a, b = -7.5156, -0.1777
    else:
        a, b = -7.9882, -0.0505
    return a,b

def adc_to_cm(sensor, adc):
    """
    sensor == 1 is ir1
    sensor == 2 is ir2
    sensor == 0 is dms

    ir works up to ~ 25 cm
    dms works ~ 7cm to 60 cm
    """
    if adc <= 0:
        return float("inf")

    a, b = get_sensor_consts(sensor)

    return (math.log(adc) + a) / (b)

def cm_to_adc(sensor, cm):
    """
    sensor == 1 is ir1
    sensor == 2 is ir2
    sensor == 0 is dms

    ir works up to ~ 25 cm
    dms works ~ 7cm to 60 cm
    """
    a, b = get_sensor_consts(sensor)

    return math.exp(b * cm - a)

def viewSensors(sensors):
    """
    assumes sensors are the ports [IR1, IR2, DMS]
    """
    ir1, ir2, dms = sensors
    adc = map(getSensorValue, sensors)
    vals = [-1]*3
    for i,p in enumerate(sensors):
        vals[i] = adc_to_cm(i+1, adc[i])
    print "SENSOR DATA"
    print "IR1: %d adc\nIR2: %d adc\nDMS: %d adc" % (adc[0], adc[1], adc[2])
    print "IR1: %.2fcm\nIR2: %.2fcm\nDMS: %.2fcm\n" % (vals[0], vals[1], vals[2])

def sweepSensors(sensors):
    """
    assumes sensors are the ports [IR1, IR2, DMS]
    """
    ir1, ir2, dms = sensors

    res = 40

    irdata = {}
    dmsdata = {}

    setMotorTargetSpeed(9,1023)
    setMotorTargetPositionCommand(9,0)
    time.sleep(1)

    for i in range(0, 1024, res):
        setMotorTargetPositionCommand(9,i)

        deg = int(adc_to_deg(1, getMotorPositionCommand(9)))
        dmsdata[deg] = adc_to_cm(0, getSensorValue(dms))

        ir = min(adc_to_cm(1, getSensorValue(ir1)), 60)
        if deg > 0:
            deg -= 170
        else:
            deg += 190

            if deg > 180:
                deg = deg - 360

        irdata[deg] = ir
    
    # average ir and deg
    avg = {}
    for k in (set(irdata.keys()) | set(dmsdata.keys())):
        if not k in irdata:
            avg[k] = dmsdata[k]
        elif not k in dmsdata:
            avg[k] = irdata[k]
        else:
            avg[k] = (dmsdata[k] + irdata[k]) / 2.0

    return avg

def detectWalls(sensors):
    avgs = sweepSensors(sensors)
    threshold = 40

    getVals = lambda start, end: [avgs[deg] for deg in range(start,end+1) if deg in avgs]
    north = getVals(-180, -165) + getVals(165, 180)
    south = getVals(-15, 15)    
    east = getVals(75, 105)
    west = getVals (-105, -75)

    return {"north": sum(north) / len(north) < threshold, "south": sum(south) / len(south) < threshold, "east": sum(east) / len(east) < threshold, "west": sum(west) / len(west) < threshold}

    

#################################### State Management ####################################

def go_to_default():
    go_to_state("default")


def go_to_state(statename):
    if type(statename) is str:
        states = pickle.load( open("states.p", "rb"))
        state = states[statename]
    elif type(statename) is list:
        state = statename
    # print("Attempting", statename)
    setMotorTargetPositionsSync(len(state), range(1,9), state)
    # print("Completed", statename)

def captureState(name):
    states = pickle.load( open("states.p", "rb"))
    states[name] = [getMotorPositionCommand(i) for i in range(1,9)]
    pickle.dump(states, open("states.p", "wb"))

def doBehavior1():
    behaviors = ["default", "crouch", "lean", "wiggle1", "wiggle2", "wiggle1", "wiggle2", "wiggle1", "wiggle2", "halfup", "default"]
    doBehavior(behaviors)

def doBehavior(behaviors):
    for b in behaviors:
        go_to_state(b)

def viewStates():
    states = pickle.load(open("states.p", "rb"))
    for k in states.keys():
        degs = [-1] * 8
        for i in range(8):
            degs[i] = "%.2f" % adc_to_deg(i+1, states[k][i])
        print k, "\t\t", degs

#################################### Macro Actions ####################################

def fineleft():
    b = []
    for i in range(1,9):
        b.append("left"+str(i))
    b.append("default")
    doBehavior(b)

def fineright():
    b = []
    for i in range(1,9):
        b.append("right"+str(i))
    b.append("default")
    doBehavior(b)

def turn(i, direction="clockwise"):
    if direction == "clockwise":
        behaviors = ["lift1", "lift2"] # ["lift1", "swing1", "lift2"]
    else:
        behaviors = ["lift3", "lift4"] # ["lift3", "swing3", "lift4"]
    doBehavior([behaviors[i%len(behaviors)]])

def turn90(direction="clockwise"):
    if direction == "clockwise":
        b = ["lift2", "lift1"] * 4
        doBehavior(b)
        go_to_default()
    else:
        b = ["lift4", "lift3"] * 4 + ["lift4"]
        doBehavior(b)
        go_to_default()

def turn180(direction="clockwise"):
    if direction == "clockwise":
        b = ["lift2", "lift1"] * 9
        doBehavior(b)
        go_to_default()
    else:
        b = ["lift4", "lift3"] * 9 + ["lift4"]
        doBehavior(b)
        go_to_default()

def forward(cycles = 1):
    states = pickle.load( open("states.p", "rb"))
    b = [states["forward1"], states["forward2"]] * cycles
    doBehavior(b)

def rightforward(cycles = 1):
    states = pickle.load( open("states.p", "rb"))
    b = [states["forward2"], states["forward1"]] * cycles
    doBehavior(b)

def backward(cycles = 1):
    states = pickle.load( open("states.p", "rb"))
    b = [states["backward1"], states["backward2"]] * cycles
    doBehavior(b)

def strafeleft(cycles = 1):
    states = pickle.load( open("states.p", "rb"))
    b = [states["strafeleft1"], states["strafeleft2"]] * cycles
    doBehavior(b)

def straferight(cycles = 1):
    states = pickle.load( open("states.p", "rb"))
    b = [states["straferight1"], states["straferight2"]] * cycles
    doBehavior(b)
