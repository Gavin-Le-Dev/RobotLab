import sys
import time
import Adafruit_PCA9685
import RPi.GPIO as GPIO
import signal
import math

#Defined Variable to store tick counts
rightCount = 0
leftCount = 0
preRightCount = 0
preLeftCount = 0

#Setup servo and encoders of robot
LSERVO = 0
RSERVO = 1
LENCODER = 17
RENCODER = 18
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(50)
pwm.set_pwm(LSERVO, 0, math.floor(1.5/20*4096))
pwm.set_pwm(RSERVO, 0, math.floor(1.5/20*4096))

# For calibrate speeds, contains acccurate speeds
leftFwdSpeeds = {}
rightFwdSpeeds = {}
leftBwdSpeeds = {}
rightBwdSpeeds = {}

#Get starting Time
startTime = time.time()

#Ctrl+c to exit
def ctrlC(signum, frame):
    pwm.set_pwm(LSERVO, 0, 0)
    pwm.set_pwm(RSERVO, 0, 0)
    exit()
    
signal.signal(signal.SIGINT, ctrlC)

GPIO.setmode(GPIO.BCM)


#------B2.ENCODER:-------
#Function count right encoder tick
def rEncoderTick(pin):
    global rightCount
    rightCount = rightCount + 1
    
#Function count left encoder tick
def lEncoderTick(pin):
    global leftCount
    leftCount = leftCount + 1
    
#Function to reset tick count
def resetCount():
    global rightCount
    global leftCount
    global preRightCount
    global preLeftCount
    global startTime
    preRightCount = rightCount
    preLeftCount = leftCount
    rightCount = 0
    leftCount = 0
    startTime = time.time()

#Fuction to get tick count
def getCounts():
    global preRightCount
    global preLeftCount
    return (preLeftCount, preRightCount)

#Function return left and right wheel speeds
def getSpeeds():
    global startTime
    global leftCount
    global rightCount
    currTime = time.time()
    if (leftCount > 0):
        leftSpeed = (leftCount/32) / (currTime - startTime)
    else:
        leftSpeed = 0
    
    if (rightCount > 0):
        rightSpeed = (rightCount/32) / (currTime - startTime)
    else:
        rightSpeed = 0
    return (leftSpeed, rightSpeed)

#Contain all code necessary for initialization
def initEncoders():
        # Set the pin numbering scheme to the numbering shown on the robot itself.
        GPIO.setmode(GPIO.BCM)
        # Set encoder pins as input
        # Also enable pull-up resistors on the encoder pins
        # This ensures a clean 0V and 3.3V is always outputted from the encoders.
        GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # Attach a rising edge interrupt to the encoder pins
        GPIO.add_event_detect(LENCODER, GPIO.RISING, lEncoderTick)
        GPIO.add_event_detect(RENCODER, GPIO.RISING, rEncoderTick)

#------B.3 Motor Control-----
#Function to calibrate Speed
def calibrateSpeeds():
    global SpeedData
    global startTime
    #Calibrate Left wheel
    i = 140
    while i < 150:
        x = float(i/100)
        resetCounts()
        startTime = time.time()
        pwm.set_pwm(LSERVO, 0, math.floor(x / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(x / 20 * 4096))
        time.sleep(2)
        #gets speeds of wheels after it changes we will need to add in time wait for the specified number of seconds.
	    #along with a 1 second interval
        y = getSpeeds()

	    #each time we get these speeds we will enter the values into our dictionary
	    #this will make it easier to print our graph
        leftBwdSpeeds[x] = y[0]
        rightFwdSpeeds[x] = y[1]
        i+=5
        print("Left wheel forward speed: ", x, " RPS: ", y[0], "       Right wheel backward speed: ", x, " RPS: ", y[1])
    
    #Add the value for stopped to all four maps
    x = float(i/100)
    resetCounts()
    startTime = time.time()
    pwm.set_pwm(LSERVO, 0, math.floor(x / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(x / 20 * 4096))
    time.sleep(2)
    #gets speeds of wheels after it changes we will need to add in time wait for the specified number of seconds.
	#along with a 1 second interval
    y = getSpeeds()
	#each time we get these speeds we will enter the values into our dictionary
	#this will make it easier to print our graph
    leftFwdSpeeds[x] = y[0]
    leftBwdSpeeds[x] = y[0]
    rightFwdSpeeds[x] = y[1]
    rightBwdSpeeds[x] = y[1] 
    i+=5   
    print("Left wheel stopped speed: ", x, " RPS: ", y[0], "       Right wheel stopped speed: ", x, " RPS: ", y[1])

    #Calibrate Right Wheel
    while i <=160:
        x = float(i/100)
        resetCounts()
        startTime = time.time()
        pwm.set_pwm(LSERVO, 0, math.floor(x / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(x / 20 * 4096))
        time.sleep(2)
        y = getSpeeds()
        rightBwdSpeeds[x] = y[1]
        leftFwdSpeeds[x] = y[0]        
        i+=5
        print("Left wheel backwards speed: ", x, " RPS: ", y[0],"       Right wheel forward speed: ", x, " RPS: ", y[1])

    pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
    print(rightFwdSpeeds)
    print(rightBwdSpeeds)
    print(leftFwdSpeeds)
    print(leftBwdSpeeds)


#Function to set speed of motors in PWM
def setSpeedsPWM(pwmLeft, pwmRight):
    self.pwm.set_pwm(LSERVO, 0, math.floor(pwmLeft / 20 * 4096))
    self.pwm.set_pwm(RSERVO, 0, math.floor(pwmRight / 20 * 4096))
    
#Function to set speed in RPS
def setSpeedsRPS(rpsLeft, rpsRight):
    LPWM = 1.5
    RPWM = 1.5
    LfloorPWM = 1.5
    LceilingPWM = 1.5
    RfloorPWM = 1.5
    RceilingPWM = 1.5
    
    #Forwards Left 
    if(rpsLeft > 0):
        #rpsLeft is faster than what is possible
        if(leftFwdSpeeds[1.6] <= rpsLeft):
            LPWM = 1.6
        else:
            #rpsLeft is in between 1.4 and 1.45 pwm
            if((leftFwdSpeeds[1.6] >= rpsLeft) and (rpsLeft > leftFwdSpeeds[1.55])):
                LfloorPWM = 1.55
                LceilingPWM = 1.6
            #rpsLeft is in between 1.45 and 1.5 pwm
            if((leftFwdSpeeds[1.55] >= rpsLeft) and (rpsLeft > leftFwdSpeeds[1.5])):
                LfloorPWM = 1.5
                LceilingPWM = 1.55
        
            #Find slope based on floor and ceiling, where PWM is y and rps is x
            slope = float((LceilingPWM - LfloorPWM) / (leftFwdSpeeds[LceilingPWM] - leftFwdSpeeds[LfloorPWM]))
            LPWM = float((slope * rpsLeft) + 1.5)  #1.5 is stopped, so our y-intercept
        
    #Backwards Left        
    if(rpsLeft < 0):
        #rpsLeft is faster than what is possible
        if(leftBwdSpeeds[1.4] <= (rpsLeft * -1)):
            LPWM = 1.4
        else:
            #rpsLeft is in between 1.4 and 1.45 pwm
            if((leftBwdSpeeds[1.4] >= (rpsLeft * -1)) and ((rpsLeft * -1) > leftBwdSpeeds[1.45])):
                LfloorPWM = 1.45
                LceilingPWM = 1.4
            #rpsLeft is in between 1.45 and 1.5 pwm
            if((leftBwdSpeeds[1.45] >= (rpsLeft * -1)) and ((rpsLeft * -1) > leftBwdSpeeds[1.5])):
                LfloorPWM = 1.5
                LceilingPWM = 1.45
        
            #Find slope based on floor and ceiling, where PWM is y and rps is x
            slope = float((LceilingPWM - LfloorPWM) / (leftBwdSpeeds[LceilingPWM] - leftBwdSpeeds[LfloorPWM]))
            LPWM = float((slope * (rpsLeft * -1)) + 1.5)  #1.5 is stopped, so our y-intercept
              
    #Forwards Right
    if(rpsRight > 0):
        #rpsLeft is faster than what is possible
        if(rightFwdSpeeds[1.4] <= rpsRight):
            RPWM = 1.4
        else: 
            #rpsLeft is in between 1.4 and 1.45 pwm
            if((rightFwdSpeeds[1.4] >= rpsRight) and (rpsRight > rightFwdSpeeds[1.45])):
                RfloorPWM = 1.45
                RceilingPWM = 1.4
            #rpsLeft is in between 1.45 and 1.5 pwm
            if((rightFwdSpeeds[1.45] >= rpsRight) and (rpsRight > rightFwdSpeeds[1.5])):
                RfloorPWM = 1.5
                RceilingPWM = 1.45
            
            #Find slope based on floor and ceiling, where PWM is y and rps is x
            slope = float((RceilingPWM - RfloorPWM) / (rightFwdSpeeds[RceilingPWM] - rightFwdSpeeds[RfloorPWM]))
            RPWM = float((slope * rpsRight) + 1.5)  #1.5 is stopped, so our y-intercept
        
    #Backwards Right
    if(rpsRight < 0):
        #rpsLeft is faster than what is possible
        if(rightBwdSpeeds[1.6] <= (rpsRight * -1)):
            RPWM = 1.6
        else: 
            #rpsLeft is in between 1.4 and 1.45 pwm
            if((rightBwdSpeeds[1.6] >= (rpsRight * -1)) and ((rpsRight * -1) > rightBwdSpeeds[1.55])):
                RfloorPWM = 1.55
                RceilingPWM = 1.6
            #rpsLeft is in between 1.45 and 1.5 pwm
            if((rightBwdSpeeds[1.55] >= (rpsRight * -1)) and ((rpsRight * -1) > rightBwdSpeeds[1.5])):
                RfloorPWM = 1.5
                RceilingPWM = 1.55
            
            #Find slope based on floor and ceiling, where PWM is y and rps is x
            slope = float((RceilingPWM - RfloorPWM) / (rightBwdSpeeds[RceilingPWM] - rightBwdSpeeds[RfloorPWM]))
            RPWM = float((slope * (rpsRight * -1)) + 1.5)  #1.5 is stopped, so our y-intercept
        
    print("LPWM: ", LPWM)
    print("RPWM: ", RPWM)
    #Set the speeds based off of the calculations
    pwm.set_pwm(LSERVO, 0, math.floor(LPWM / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(RPWM / 20 * 4096))

#Set the speed based on inches per second
def setSpeedsIPS(ipsLeft, ipsRight):

	#We know how many inches is a full rotation and we can go ahead and see how many rotations we want to travel and then apply it
	circumference = 3.14 * 2.61
	#inches per sec can be turned to RPS by dividing what was traveled by the total circumference of the wheel
	ILeft = ipsLeft/circumference
	IRight = ipsRight/circumference
	#Then we plug in here in order to set the speeds using our RPS function and the speeds used in its dictionaries
	setSpeedsRPS(ILeft,IRight)



# Function to set speed in VW
def setSpeedVW(v, w):
    absAngular = abs(w)
    #We will get our Radius again by using our velocity and angular velocity
    Radius = abs(v/w)

    #Dmid is about 2 inches
    dMid = 2
    #We get our velocities for left and right wheel
    vRight = absAngular * (Radius + dMid)
    vLeft = absAngular * (Radius - dMid)
    if w < 0:
        setSpeedsIPS(vRight, vLeft)
    else:
        setSpeedsIPS(vLeft, vRight)    
    #We set the speeds IPS
    print("Radius: ", Radius)
    print("vRight: ", vRight)
    print("vLeft: ", vLeft)
    

def getMaxLeftFwd():
    #1.6
    return leftFwdSpeeds[1.6]   

def getMaxRightFwd():
    #1.3
    return rightFwdSpeeds[1.4]
    
def getMaxLeftBwd():
    #1.3
    return leftBwdSpeeds[1.4]

def getMaxRightBwd():
    #1.6
    return rightBwdSpeeds[1.6]