#from serial_master import serial_parser
#from serial_lidar_thread import lidarRead
#from pprint import pprint
#import Queue, thread
import time, math
import serial, time, os, sys
from math import pi
import RPi.GPIO as GPIO # always needed with RPi.GPIO

### serial init

debug = True
com_port_lidar  = '/dev/ttyAMA0'#'/dev/cu.usbmodem1411' ##change the COM port if required, check in Arduino IDE
com_port_master  = '/dev/ttyACM0'

#define crucial sonar ranges
k_Sens_Max_Dist = 5000
k_Sens_Mid_Dist = 2000
k_Sens_Min_Dist = 700 # 1000
k_Sens_Stop_Dist = 400 # 500

#define turn angles
turn180 = pi
turn90 = pi/2
turn45 = pi/4
turn30 = pi/6

prevCount1 = 0
prevCount2 = 0

# #define num of sensors
# numSensFront = 4
# numSensFrontL = 2
# numSensFrontR = 2
# numSensBack = 4
# numSensBackL = 2
# numSensBackR = 2

clear = True
goHome = False
# go =True
previousTime = 0

servo1 = 6
dir1 = 13
servo2 = 26
dir2 =19
encoder1a = 20
encoder1b = 21
encoder2a = 16
encoder2b = 12
encoderScaleFactor = 0.31919/1200.0

#robot variables
homeX = 0.0
homeY = 0.0
robotHeading = 0.0
robotPositionX = homeX
robotPositionY = homeY


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)  # choose BCM pinout
GPIO.setup(dir1, GPIO.OUT, initial=1)#direction bit1
GPIO.setup(servo1, GPIO.OUT)#servo1
GPIO.setup(dir2, GPIO.OUT, initial=0)#direction bit2
GPIO.setup(servo2, GPIO.OUT)#servo2
leftMotor = GPIO.PWM(servo1, 50) #set up servo 1 at 50Hz
rightMotor = GPIO.PWM(servo2, 50) #set up servo 2 at 50Hz
GPIO.setup(encoder1a,GPIO.IN) #initialize encoder GPIOs
GPIO.setup(encoder1b,GPIO.IN)
GPIO.setup(encoder2a, GPIO.IN)
GPIO.setup(encoder2b, GPIO.IN)

countw1 = 0 #initialize encoder counts
countw2 = 0

countForPositionw1 = 0
countForPositionw2 = 0

COUNTSPERREV = 1200.0
WHEELDIA = .1016 #0.073 meters
ticksToMeters = (WHEELDIA *  math.pi)/COUNTSPERREV #meters
metersToTicks = COUNTSPERREV/( math.pi * WHEELDIA) #ticks
ticksFullTurn = COUNTSPERREV * 4.8

waypoints = [[1.0,0.0], [1.0, 1.0], [0.0, 1.0], [0.0,0.0]] # meters
waypointCounter = 0
interruptWaypoint = []

def wheel1a(channel):
  if GPIO.input(encoder1a) == 1:
    if GPIO.input(encoder1b) == 0:
      count1(1,-1)
    else: count1(1,1)
  elif GPIO.input(encoder1a) == 0:
    if GPIO.input(encoder1b) == 0:
      count1(1,1)
    else: count1(1,-1)

def wheel1b(channel):
  if GPIO.input(encoder1b) == 1:
    if GPIO.input(encoder1a) == 0:
      count1(1,1)
    else: count1(1,-1)
  elif GPIO.input(encoder1b) == 0:
    if GPIO.input(encoder1a) == 0:
      count1(1,-1)
    else: count1(1,1)
def wheel2a(channel):
  if GPIO.input(encoder2a) == 1:
    if GPIO.input(encoder2b) == 0:
      count1(2,-1)
    else: count1(2,1)
  elif GPIO.input(encoder2a) == 0:
    if GPIO.input(encoder2b) == 0:
      count1(2,1)
    else: count1(2,-1)

def wheel2b(channel):
  if GPIO.input(encoder2b) == 1:
    if GPIO.input(encoder2a) == 0:
      count1(2,1)
    else: count1(2,-1)
  elif GPIO.input(encoder2b) == 0:
    if GPIO.input(encoder2a) == 0:
      count1(2,-1)
    else: count1(2,1)

GPIO.add_event_detect(encoder1a, GPIO.BOTH, callback = wheel1a)
GPIO.add_event_detect(encoder1b, GPIO.BOTH, callback = wheel1b)
GPIO.add_event_detect(encoder2a, GPIO.BOTH, callback = wheel2a)
GPIO.add_event_detect(encoder2b, GPIO.BOTH, callback = wheel2b)

def cleanup():
  leftMotor.stop()
  rightMotor.stop()
  GPIO.cleanup()

def count1(wheel,num):

  global countw1, countw2, countForPositionw1, countForPositionw2
  if wheel == 1:
    countw1 = countw1 + num
    countForPositionw1 = countForPositionw1 + num

  elif wheel == 2:
    countw2 = countw2 + num
    countForPositionw2 = countForPositionw2 + num

#----------------------------------------------------------------------------------------------------------------------------------------------------------

def setMotors(leftDir, rightDir, leftDuty, rightDuty): #leftDuty, rightDuty
  if leftDuty > 100:
    leftDuty = 100
  elif leftDuty < 0:
    leftDuty = 0
  if rightDuty > 100:
    rightDuty = 100
  elif rightDuty < 0:
    rightDuty = 0

  if leftDir == 1:
    GPIO.output(dir1,GPIO.HIGH)
  elif leftDir == -1:
    GPIO.output(dir1,GPIO.LOW)
  if rightDir == 1:
    GPIO.output(dir2,GPIO.HIGH)
  elif rightDir == -1:
    GPIO.output(dir2,GPIO.LOW)

  leftMotor.ChangeDutyCycle(leftDuty)
  rightMotor.ChangeDutyCycle(rightDuty)

def turnBot(rads,dir,duty):
  #print "yo"
  global prevCount1, prevCount2, countw1, countw2, robotHeading, countForPositionw1, countForPositionw2
  countw1, countw2 = 0,0

  ticks = (rads/(2*math.pi)* ticksFullTurn)

  targetw1 = countw1 + dir*ticks
  targetw2 = countw2 + -1*dir*ticks
  setMotors(dir,(-1*dir), duty,duty)
  print "LEFT",dir*countw1, dir*targetw1, "RIGHT",-1*dir*countw2, -1*dir*targetw2
  while dir*countw1 < dir*targetw1 or -1*dir*countw2 < -1*dir*targetw2: # or = when both pass we stop
    time.sleep(.01)
    kp=1.0/5.0
    e1 = (-1*dir*countw2)-dir*countw1
    e2 = dir*countw1-(-1*dir*countw2)
    Lduty = duty+e1*kp
    Rduty = duty+e2*kp
    #print "LEFT",dir*countw1, dir*targetw1, e1, "RIGHT",-1*dir*countw2, -1*dir*targetw2, e2
    #print countw1,countw2, Lduty,Rduty,e1
    if dir == 1:
      print "turning " + str(rads) + " clockwise"
    if dir == -1:
      print "turning " + str(rads) + " counterclockwise"
    setMotors(dir,(-1*dir), Lduty,Rduty)
  robotHeading += rads*dir
  driveBot(1,0)

  countw1, countw2, countForPositionw1, countForPositionw2, prevCount1, prevCount2 = 0,0,0,0,0,0





def driveBot(dir, duty):
  #global countw1, countw2, robotPositionX, robotPositionY, currentHeading
  #countw1, countw2 = 0,0
  #targetw1 = countw1 + dir * dist * metersToTicks
  #targetw2 = countw2 + dir * dist * metersToTicks
  #setMotors(dir,(dir), duty,duty)
  #while dir*countw1 < dir*targetw1 or dir*countw2 < dir*targetw2: # or = when both pass we stop
    #time.sleep(.01)

  kp=1.0/100
  e1 = (dir*countw2)-(dir*countw1)
  e2 = (dir*countw1)-(dir*countw2)

  Lduty = duty+e1*kp
  Rduty = duty+e2*kp
  # print countw1, countw2 #Lduty,Rduty,e1,e2,

  #print "LEFT",dir*countw1,"RIGHT",-1*dir*countw2
  #print "Meters:",ticksToMeters*countw1, Lduty,Rduty,e1
  if duty == 0:
    print "Stopped"
  else:
    print "Driving" #(countw1,countw2)
  setMotors(dir,(dir), Lduty,Rduty)
  updatePos()


##find minimum value in list and return
def minimum_value(x):
  ##set to maximum range to avoid edge case of first element being -1, because then if statement will not execute
  min_val = 99999
  for i in x[1:]:
    if i < min_val and i >= 0:
      min_val = i
  return min_val

def calculateHeading(currentHeading, currentX, currentY, newX, newY):#current heading must always be positive and less than 2 pi
  xdifference = newX-currentX
  ydifference = newY-currentY
  angle = math.atan2(ydifference, xdifference)
  if angle < 0:
    angle = 2*math.pi +angle
  angle = angle - currentHeading
  if angle > math.pi:
    angle = angle - 2*math.pi
  if angle < -1* math.pi:
    angle = angle + 2*math.pi
  return angle

def updatePos():
  global prevCount1,prevCount2,robotHeading, robotPositionX, robotPositionY#,t1,t2 #countForPositionw1, countForPositionw2,
  counts1 = countw1 - prevCount1
  counts2 = countw2 - prevCount2
  averageTicks = (counts1 + counts2)/2.0
  displacement = averageTicks*encoderScaleFactor
  robotPositionX += math.cos(robotHeading) * displacement
  robotPositionY += math.sin(robotHeading) * displacement
  print countw1,countw2,robotPositionX,robotPositionY,prevCount1 +counts1,prevCount2+counts2
  # print countForPositionw1, countForPositionw2,robotPositionX,robotPositionY
  # countForPositionw1, countForPositionw2 = 0,0
  prevCount1 = countw1
  prevCount2 = countw2

def goToWayPoint(waypoint):
  global waypointCounter, waypoints, interruptWaypoint, robotPositionX, robotPositionY, homeX, homeY
  if len(interruptWaypoint) != 0:
    waypointX = interruptWaypoint[0][0]
    waypointY = interruptWaypoint[0][1]
  else:
    waypointX = waypoint[0]
    waypointY = waypoint[1]

  angle = calculateHeading(robotHeading, robotPositionX, robotPositionY, waypointX, waypointY)
  # if abs(angle - pi) < 0.0001:
  #   angle = 0
  if waypointCounter == 0:
    totalDistance = math.sqrt((homeX - waypointX)**2 + (homeY - waypointY)**2)
  else:
    totalDistance = math.sqrt((waypoints[waypointCounter-1][0] - waypointX)**2 + (waypoints[waypointCounter-1][1] - waypointY)**2)
  distance = math.sqrt((robotPositionX - waypointX)**2 + (robotPositionY - waypointY)**2)
  # print countw1,countw2
  # turn bot
  if abs(angle) > 15.0*(pi/180.0): #maybe change this to a have a small tolerance
    if angle < 0:
      direction = -1
    else: direction = 1
    turnBot(abs(angle), direction, 40)

  #print waypointCounter
  #print waypointCounter

  if distance > 0.05:
    if (totalDistance - distance) < 0.1: # beginning speed up slowly
      driveBot(1,15+300*(((totalDistance-distance))))
    elif distance < 0.1:# end slow down
      driveBot(1,15+300*(distance))
    else: # middle go steady
      driveBot(1,15+300*(((0.1))))
  else:
    driveBot(1,0)
    if len(interruptWaypoint) != 0:
      interruptWaypoint = []
    else:
      if waypointCounter != len(waypoints)-1:
        waypointCounter +=1
      else:
        waypointCounter = 0



def goToInterruptWaypoint(angle, direction):
  global robotPositionX, robotPositionY, robotHeading, interruptWaypoint
  interruptWaypoint.append((robotPositionX + math.cos(robotHeading + angle*direction), robotPositionY + math.sin(robotHeading + angle*direction)))
  goToWayPoint(interruptWaypoint[0]) #dont need to specify this

#/*Obstacle avoidance algorithm*/
def collisionAvoid(quad1, quad2, quad3, quad4):
 # quad 1, quad2, quad3, quad 4
 # \\\\\\\\\ |||||||| /////////
 #         [robot_here]
  global clear, previousTime, waypoints, waypointCounter
  current_time = time.time()

  #define the front sensors
  m_SensC_FrontLL = quad1
  m_SensC_FrontLC = quad2
  m_SensC_FrontRC = quad3
  m_SensC_FrontRR = quad4

  #define total distances per edge of robot
  frontTotL = m_SensC_FrontLL + m_SensC_FrontLC
  frontTotR = m_SensC_FrontRR + m_SensC_FrontRC
  frontTot = frontTotL + frontTotR

  #tolerance = 700

  if (clear and (m_SensC_FrontRR <= k_Sens_Min_Dist or m_SensC_FrontRC <= k_Sens_Min_Dist or m_SensC_FrontLL <= k_Sens_Min_Dist or m_SensC_FrontLC <= k_Sens_Min_Dist)):
    previousTime = time.time()
    clear = False
    driveBot(1,0)
  elif not (m_SensC_FrontRR <= k_Sens_Min_Dist or m_SensC_FrontRC <= k_Sens_Min_Dist or m_SensC_FrontLL <= k_Sens_Min_Dist or m_SensC_FrontLC <= k_Sens_Min_Dist):
    clear = True
    goToWayPoint(waypoints[waypointCounter])

  #Only care if our sensors are sensing any objects within range
  #Otherwise leave speed and turn angle unchanged and allow robot to continue as is
  if ((m_SensC_FrontRR <= k_Sens_Min_Dist or m_SensC_FrontRC <= k_Sens_Min_Dist or m_SensC_FrontLL <= k_Sens_Min_Dist or m_SensC_FrontLC <= k_Sens_Min_Dist) and (previousTime <= time.time() - 3)):
    clear = True
    print (quad1, quad2, quad3, quad4)
    #want to make a sharp turn on the spot if any of the sensors are within critical distance
    #A tolerance is used when comparing left sensors vs right sensors
    #This is necessary for rare cases where all sensors are within critical distance, in this case robot will reverse
    if (m_SensC_FrontRR <= k_Sens_Stop_Dist or m_SensC_FrontRC <= k_Sens_Stop_Dist or m_SensC_FrontLL <= k_Sens_Stop_Dist or m_SensC_FrontLC <= k_Sens_Stop_Dist):
      driveBot(1,0)
      if (frontTotL <= frontTotR):
        if (m_SensC_FrontLC <= k_Sens_Stop_Dist and m_SensC_FrontRC <= k_Sens_Stop_Dist):
          goToInterruptWaypoint(turn180, 1)#turnBot(turn180, 1, 30)
          #right
        elif (m_SensC_FrontLL <= k_Sens_Stop_Dist and m_SensC_FrontLC <= k_Sens_Stop_Dist):
          goToInterruptWaypoint(turn90,1)#turnBot(turn90, 1, 30) #right
        elif (m_SensC_FrontLC <= k_Sens_Stop_Dist):
          goToInterruptWaypoint(turn45,1)#turnBot(turn45, 1, 30)
        elif (m_SensC_FrontLL <= k_Sens_Stop_Dist):
          goToInterruptWaypoint(turn30,1)#turnBot(turn30, 1, 30)
      else: # (frontTotL + tolerance <= frontTotR):
        if (m_SensC_FrontLC <= k_Sens_Stop_Dist and m_SensC_FrontRC <= k_Sens_Stop_Dist):
          goToInterruptWaypoint(turn180,-1)#turnBot(turn180, -1, 30)
        elif (m_SensC_FrontRR <= k_Sens_Stop_Dist and m_SensC_FrontRC <= k_Sens_Stop_Dist):
          goToInterruptWaypoint(turn90,-1)#turnBot(turn90, -1, 30)
        elif (m_SensC_FrontRC <= k_Sens_Stop_Dist):
          goToInterruptWaypoint(turn45,-1)#turnBot(turn45, -1, 30)
        elif (m_SensC_FrontRR <= k_Sens_Stop_Dist):
          goToInterruptWaypoint(turn30, -1)#turnBot(turn30, -1, 30)
     #None of the sensors are within critical distance, but within range of object
     #Must decrease speed and gradually adjust direction
    elif not (m_SensC_FrontRR <= k_Sens_Stop_Dist or m_SensC_FrontRC <= k_Sens_Stop_Dist or m_SensC_FrontLL <= k_Sens_Stop_Dist or m_SensC_FrontLC <= k_Sens_Stop_Dist):
      driveBot(1,0)
      if (frontTotL  <= frontTotR):
        if (m_SensC_FrontLC <= k_Sens_Min_Dist and m_SensC_FrontRC <= k_Sens_Min_Dist):
          goToInterruptWaypoint(turn90, 1)#turnBot(turn90, 1, 30)
        elif (m_SensC_FrontLL <= k_Sens_Min_Dist and m_SensC_FrontLC <= k_Sens_Min_Dist):
          goToInterruptWaypoint(turn45,1)#turnBot(turn45, 1, 30)
        elif (m_SensC_FrontLL <= k_Sens_Min_Dist or m_SensC_FrontLC <= k_Sens_Min_Dist):
          goToInterruptWaypoint(turn30, 1)#turnBot(turn30, 1, 30)
      elif (frontTotL  > frontTotR):
        if (m_SensC_FrontLC <= k_Sens_Min_Dist and m_SensC_FrontRC <= k_Sens_Min_Dist):
          goToInterruptWaypoint(turn90,-1)#turnBot(turn90, -1, 30)
        elif (m_SensC_FrontRR <= k_Sens_Min_Dist and m_SensC_FrontRC <= k_Sens_Min_Dist):
          goToInterruptWaypoint(turn45,-1)#turnBot(turn45, -1, 30)
        elif (m_SensC_FrontRR <= k_Sens_Min_Dist or m_SensC_FrontRC <= k_Sens_Min_Dist):
          goToInterruptWaypoint(turn30,-1)#turnBot(turn30, -1, 30)
      else: pass   #leave as set prior
    else: pass  #leave as set prior
  else: pass   #leave as set prior


def getQuads(dist_mm_store):

  global robotHeading, robotPositionX, robotPositionY, homeX, homeY, goHome, waypoints, waypointCounter

  ##finding minimum distance in 180 degrees of vision, split into 45 degree quadrants
  quad1 = minimum_value(dist_mm_store[305:314])  #currently not 45 degree, just avoiding the wheel... need to move lidar out more
  quad2 = minimum_value(dist_mm_store[315:359]) #should be to 315 to 360, but we want from 315+offset up to 0+offset so split to 2
  quad3 = minimum_value(dist_mm_store[0:44])
  quad4 = minimum_value(dist_mm_store[45:55]) #currently not 45 degree, just avoiding the wheel... need to move lidar out more

  if (quad1 != 99999 and quad2 != 99999 and quad3 != 99999 and quad4 != 99999):
    if clear:
      if goHome:
        homeHeading = calculateHeading(robotHeading, robotPositionX, robotPositionY, homeX, homeY)
        if homeHeading > 0:
          homeDir = 1
        else:
          homeDir = -1
        goToInterruptWaypoint(homeHeading, homeDir)
        time.sleep(5) #update to like 30 or something
      else:
        goToWayPoint(waypoints[waypointCounter])
    collisionAvoid(quad1,quad2,quad3,quad4)

def get(q):
  x = q.get()
  while not(q.empty()):
    q.get()
  #print q.qsize()
  return x

def main():
  global goHome, com_port_master, com_port_lidar, waypoints, waypointCounter

  #q = Queue.LifoQueue() #maxsize=0
  #t = thread.start_new_thread ( serial_parser, (com_port_master, q,) )

  #q_lidar = Queue.LifoQueue() #maxsize=0
  #t_lidar = thread.start_new_thread ( lidarRead, (com_port_lidar, q_lidar,) )

  while True:
    try:
      #lidar_array = get(q_lidar)
      #master_dictionary = get(q)

      #pprint(lidar_array)


      #print master_dictionary["w_avg"]

      #if (master_dictionary["v_avg"] == 0): #20 balls = 918, 14 balls = 643
        #goHome = True
      #else:
        #goHome = False

      #getQuads(lidar_array)
      goToWayPoint(waypoints[waypointCounter]) #temp
      # turnBot(math.pi, 1, 40)
      # break
      # driveBot(1,40)

      #time.sleep(2)

    except (KeyboardInterrupt, SystemExit):
      print 'keyboard inturrupt!'
      #t.join()
      #t_lidar.join()
      cleanup()
      break
      #raise
      #finally:


rightMotor.start(0)
leftMotor.start(0)
main()
cleanup()