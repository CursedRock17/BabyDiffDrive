class PIDController:
    def __init__(self):
        # Main Controllable Gains For Which We Tune
        self.proportionalGain = 0
        self.integralGain = 0
        self.derivativeGain = 0

        # Tracking Our Errors to Compare to Current
        self.errorSum = 0
        self.lastInput = 0

        # Prevent the output from oversaturation
        self.outputMin = 0
        self.outputMax = 0
        self.output = 0

    # Allows the user to tune all the gains easily
    def tuneGains(self, kp, ki, kd):
        self.proportionalGain = kp
        self.integralGain = ki
        self.derivativeGain = kd

    def clampOutput(self, min, max):
        # Prevent accidental swap
        if (min > max):
            temp = min
            min = max
            max = temp

        self.outputMin = min
        self.outputMax = max

    # Simple Reset Function to Clear up on Startup
    def reset(self):
        self.errorSum = 0
        self.lastInput = 0
        self.output = 0

    # Update the error based on that PID update time
    def update(self, targetValue, processValue, deltaTime):
        # Check out current time in regards to the whole process
        if deltaTime <= 0:
            return -1

        # Need to compare the wanted value and the value we're actually getting
        # Our encoder track total number of counts so we don't have to alter
        error = targetValue - processValue
        # We can then pace the error to prevent overshoot
        self.errorSum += error * deltaTime
        deltaInput = (processValue - self.lastInput) / deltaTime

        # Create a PID Equation based on the OG equation:
        # u(t) = Kp * e(t) + Ki * integral(e(t) dt) + Kd * d (e(t)) / dt
        # Altered to be:
        # u(t) = kp * e(t) + Ki * integral(e(t) dt) - Kd * d (pv) / dt
        propOut = self.proportionalGain * error
        intOut = self.integralGain * self.errorSum
        derivOut = self.derivativeGain * deltaInput

        result = propOut + intOut - derivOut

        # Clamp the PID Output
        if (result > self.outputMax):
            result = self.outputMax
        elif (result < self.outputMin):
            result = self.outputMin

        # Now we can alter of "last" variables to now
        self.lastInput = processValue
        self.output = result
        return result
