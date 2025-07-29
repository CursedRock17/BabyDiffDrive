from PID import PIDController
import time
import math
from machine import Pin, PWM


class Rover:
    # Necessary Constants
    PWM_MAX = 2 ** 16 - 1  # 65535
    PI = 3.14159
    pidUpdateRate = 0.05  # In Seconds

    def __init__(self, wheelDiameterMM, pulsesPerRevolution):
        # Information about the rover
        self.roverWheels = 2
        self.wheelBaseDistance = 190  # in millimeters

        # Add all of the pins as an array
        self.wheelInterruptPinsA = [None] * self.roverWheels
        self.wheelInterruptPinsB = [None] * self.roverWheels
        self.wheelDirectionPinsA = [None] * self.roverWheels
        self.wheelDirectionPinsB = [None] * self.roverWheels

        # Create an "internal" clock - careful with multithreading
        self.currentTime = time.ticks_diff(time.ticks_ms(), 0)
        self.lastTime = self.currenTime

        # Create defined characters for each wheel
        self.wheelPID            = [None] * self.roverWheels
        self.wheelDiameter       = [0] * self.roverWheels
        self.wheelPulsesPerRev         = [0] * self.roverWheels
        self.wheelEncoderCount   = [0] * self.roverWheels
        self.zeroSpeed = 15.3

        # Add a basic PID Controller to each wheel 
        for i in range (self.roverWheels):
            self.wheelPID[i] = PIDController()

            # Define the specs of each wheel
            self.wheelDiameter[i] = wheelDiameterMM
            self.wheelPulsesPerRev[i]   = pulsesPerRevolution

        # TODO: Odometry - Keep Track of Positioning
        # Will eventually split into Pose & Twist
        self.adjustedPulses = [1] * self.roverWheels
        self.orientation = [0, 0, 0]  # (X, Y, theta) of overall bot

    # Call Once Per Motor in the Init() section of the code
    def setupPins(self, currentWheel, wheelInterruptPinA, wheelInterruptPinB, wheelDirectionalPinA, wheelDirectionalPinB):
        # Have to add each of the pins to the array so easy to track
        self.wheelInterruptPinsA[currentWheel] = Pin(wheelInterruptPinA, Pin.IN)
        self.wheelInterruptPinsB[currentWheel] = Pin(wheelInterruptPinB, Pin.IN)
        self.wheelDirectionPinsA[currentWheel] = PWM(Pin(wheelDirectionalPinA, Pin.OUT))
        self.wheelDirectionPinsB[currentWheel] = PWM(Pin(wheelDirectionalPinB, Pin.OUT))

        # Add interrupts and their encoder functions to tell direction
        # Forcing the interrupt to rely on Pin A's clock's rising edge
        if currentWheel == 0:
            self.wheelInterruptPinsA[currentWheel].irq(handler=self.readEncoderOne, trigger=Pin.IRQ_RISING)
        elif currentWheel == 1:
            self.wheelInterruptPinsA[currentWheel].irq(handler=self.readEncoderTwo, trigger=Pin.IRQ_RISING)
        else:
            print("You have exceeded the number of wheels, try again")

    ###################################################
    ############  Motor Encoder Pulse Functions #######
    ###################################################
    # Check interrupt pin signal in relation to the clock
    # One pin essentially acts as a clock (A) while the other (B)
    # Gets its status checked to see if the motor is actually spinning
    def readEncoderOne(self, pin):
        pinB = self.wheelInterruptPinsB[0].value()
        if pinB == 1:
            self.wheelEncoderCount[0] += 1
        else:
            self.wheelEncoderCount[0] -= 1

    def readEncoderTwo(self, pin):
        pinB = self.wheelInterruptPinsB[1].value()
        if pinB == 1:
            self.wheelEncoderCount[1] += 1
        else:
            self.wheelEncoderCount[1] -= 1

    ###################################################
    ############## Motor Interaction Functions ########
    ###################################################
    # Get the motor to spin in whatever manner told, so the user can drive
    def driveMotor(self, direction, pwmValue, dirPinA, dirPinB):
        # Send the PWM Value to it's respective PIN (this value will be PID'd)
        # Check the direction given and write the encoder pins to correspond
        if direction == 1:
            dirPinA.duty_u16(pwmValue)
            dirPinB.duty_u16(0)
        elif direction == -1:
            dirPinA.duty_u16(0)
            dirPinB.duty_u16(pwmValue)
        else:
            dirPinA.duty_u16(0)
            dirPinB.duty_u16(0)

    # Control each separate motor by running its respective PID loop and determing
    # Power usage along with current status
    def setMotor(self, currentWheel, desiredPulses, maxPower):
        # Tracking the pulses needed to meet our goal covers the PWM speed
        # More pulses needed, the more we have to run at max power towards the
        # goal direction, otherwise slow down, or reverse
        self.adjustedPulses[currentWheel] = self.wheelPID[currentWheel].update(
                desiredPulses, self.wheelEncoderCount[currentWheel],
                self.refreshTime())

        # Update the direction
        dir = 1
        if (self.adjustedPulses[currentWheel] < 0):
            dir = -1

        # We need to clamp the power to min and max bounds
        powerPercent = abs(self.adjustedPulses[currentWheel])
        if (powerPercent > maxPower):
            powerPercent = maxPower

        pwmSignal = int(self.PWM_MAX * (powerPercent / 100))

        # Utilize our directional value and PWM Signal to move the wheels
        self.driveMotor(dir, pwmSignal, self.wheelDirectionPinsA[currentWheel],
                        self.wheelDirectionPinsB[currentWheel])

        # Call this function until we've arrived at the set destination
        return pwmSignal

    ###################################################
    ################# Control Functions ###############
    ###################################################
    def driveStraight(self, distanceMM, powerPercent):
        # Create Infinite loop between driving moth motors, until we've
        # Reach zero speed
        result = 1
        while result > (self.zeroSpeed / 100):
            for i in range(self.roverWheels):
                wantedPulses = self.mmToPulses(distanceMM, i)
                # Fix inverted motors
                if (i == 0):
                    wantedPulses *= -1

                lastPulses = self.adjustedPulses[i]
                result = self.setMotor(i, wantedPulses,
                                       powerPercent) / self.PWM_MAX
        # Make Sure Motors No Longer Move "Zeroed"
        self.resetMotors()

        # Update Odometry - X, Y, Theta on Cartesian Coords
        self.orientation[0] += distanceMM * math.cos(self.orientation[2])
        self.orientation[1] += distanceMM * math.sin(self.orientation[2])
        self.printOdometry()

    # Function to turn at an angle while driving, means both wheels
    # should not have the same PWM value
    def driveArc(self, distanceMM, angleDegrees, powerPercent):
        for i in range(self.roverWheels):
            distanceMM *= -1
            self.orientation[0] = distanceMM
            self.orientation[2] = angleDegrees

    # Function to rotate in place
    def rotate(self, angleDegrees, powerPercent):
        # Essentially always a triangle with two sides the same
        if angleDegrees > 0:
            # Create Infinite loop between driving moth motors, until we've
            # Reach zero speed
            result = 1
            # Distance is s = pi * r * theta / 180, r = totalBase / 2
            distanceMM = (self.PI * (self.wheelBaseDistance / 2) * angleDegrees) / 180
            while result > (self.zeroSpeed / 100):
                for i in range (self.roverWheels):
                    wantedPulses = self.mmToPulses(distanceMM, i)
                    lastPulses = self.adjustedPulses[i]
                    result = self.setMotor(i, wantedPulses,
                                           powerPercent) / self.PWM_MAX
        elif angleDegrees < 0:
            # Create Infinite loop between driving moth motors, until we've
            # Reach zero speed
            result = 1
            while result > (self.zeroSpeed / 100):
                distanceMM = angleDegrees * self.wheelBaseDistance
                wantedPulses = self.mmToPulses(distanceMM, 1)
                lastPulses = self.adjustedPulses[1]
                result = self.setMotor(1, wantedPulses,
                                       powerPercent) / self.PWM_MAX
            # Make Sure Motors No Longer Move "Zeroed"
            self.resetMotors()
        # Update Odometry - X, Y, Theta on Cartesian Coords
        self.orientation[2] += angleDegrees
        self.printOdometry()

    # General Reset Function for All the Motors
    def resetMotors(self):
        for i in range(self.roverWheels):
            self.adjustedPulses[i] = 1
            self.wheelEncoderCount[i] = 0
            # self.wheelPID[i].reset()
            self.setMotor(i, 0, 0)

    ###################################################
    ################# Helper Functions ################
    ###################################################
    def refreshTime(self):
        self.currentTime = time.ticks_diff(time.ticks_ms(), self.startTime)
        deltaTime = float((self.currenTime - self.lastTime) / 1e6)  # Convert to S
        time.sleep(self.pidUpdateRate)
        self.lastTime = self.currenTime
        return deltaTime

    def mmToPulses(self, mm, currentWheel):
        # Need to Convert distances traveled to pulses so that encoder
        # can accurately control the wheel
        # Get the pulses per revolution : encoder counter per rev * gear ratio
        # When our result to be in mm/pulse, so when we divide the wanted
        # distance we get mm / (mm/pulse) = mm / 1 * pulse / mm
        wheelCircumference = self.PI * self.wheelDiameter[currentWheel]
        mmPerPulse = (wheelCircumference / self.wheelPulsesPerRev[currentWheel])

        pulses = mm / mmPerPulse
        return pulses

    def pulsesToMM(self, currentWheel):
        # Reverse function in case we want to display information
        wheelCircumference = self.PI * self.wheelDiameter[currentWheel]
        revolutions = self.wheelEncoderCount[currentWheel] / self.wheelPulsesPerRev[currentWheel]

        # Using the circumference of the wheel, we can divide it by the pulses
        # of a regular full rotation to see number of pulses/mm
        distance = revolutions * wheelCircumference
        return distance

    def tuneGains(self, currentWheel, kp, ki, kd):
        currentPID = self.wheelPID[currentWheel]
        currentPID.proportionalGain = kp
        currentPID.integralGain = ki
        currentPID.derivativeGain = kd

    def printGains(self):
        for i in range(self.roverWheels):
            currentPID = self.wheelPID[i]
            print("Motor ", i, " Values:")
            print("P: ", currentPID.proportionalGain, "I: ", currentPID.integralGain, "D: ", currentPID.derivativeGain)

    def printOdometry(self):
        print("Pose Information: ")
        print("X: ", self.orientation[0])
        print("Y: ", self.orientation[1])
        print("0: ", self.orientation[2])
