from Rover import RoverController

# Motor/Wheel Info - [DataSheet](https://people.ece.ubc.ca/~eng-services/files/courses/elec391-data_sheets/MOT4-info.pdf)
countersPerRev = 12  # Should be the same for all 130 RPM
gearRatio = 47
rpmRatio = 467 / 126
wheelDiameterMM = 68.07   # Millimeters (mm)
pulsesPerRevolution = (countersPerRev * gearRatio) / rpmRatio

# Define our wheels on the rover
motorRight = 0
motorLeft = 1

# Setting up the Controller
controller = RoverController(wheelDiameterMM, pulsesPerRevolution)

# Setup Right and Left Motor Respectively
# It goes Wheel Number Designator (0 for right, 1 for left)
# Encoder A, Encoder B, Input 1, Input 2
# Make sure your pins align with the correct direction
controller.setupPins(motorRight, 23, 22, 26, 27)
controller.setupPins(motorLeft, 21, 19, 12, 13)


# Motor Direction Truth Table
#  | IN1 | IN2 | DIRECTION  |
#  --------------------------
#  | 1   |  0  | Forward    |
#  | 0   |  1  | Backward   |
#  | 0   |  0  | STOP       |

maxSpeed = 50  # Probably should leave here
targetPos = pulsesPerRevolution * 1

# Example Functions
def driveForward(distanceInMM):
    controller.resetMotors()
    controller.driveStraight(distanceInMM, maxSpeed)

def driveBackward(distanceinMM):
    driveForward(-1 * distanceinMM)

def rotateRight(angleInDegrees):
    controller.resetMotors()
    controller.rotate(angleInDegrees, maxSpeed)

def rotateLeft(angleInDegrees):
    controller.resetMotors()
    controller.rotate(-1 * angleInDegrees, maxSpeed)

# Have to tune the gains to your own motors
# Mark a spot on the wheel and ensure it can make 1 full rotation
controller.tuneGains(motorRight, 0.764, 0.028, 0.0)
controller.tuneGains(motorLeft, 0.685, 0.028, 0.0)
controller.zeroSpeed = 15.3

# Insert however you want it to move (i.e keyboard, sensor-based, etc)
driveForward(targetPos)

# Should say pose estimation
controller.printOdometry()
