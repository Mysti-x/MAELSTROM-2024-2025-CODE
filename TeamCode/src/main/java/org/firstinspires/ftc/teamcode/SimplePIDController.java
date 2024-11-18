package org.firstinspires.ftc.teamcode;

public class SimplePIDController {
    private double kP, kI, kD; // PID coefficients
    private double targetPosition = 0; // Desired target position
    private double integralSum = 0; // Integral accumulator
    private double lastError = 0; // Previous error for derivative calculation
    private double maxOutput = 1.0; // Maximum output
    private double minOutput = -1.0; // Minimum output
    private double feedforward = 0; // Feedforward term (optional)

    // Constructor
    public SimplePIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    // Set the target position
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    // Get the target position
    public double getTargetPosition() {
        return targetPosition;
    }

    // Update PID loop calculation
    public double update(double currentPosition) {
        // Calculate error
        double error = targetPosition - currentPosition;

        // Accumulate integral term
        integralSum += error;

        // Calculate derivative term
        double derivative = error - lastError;

        // Compute PID output
        double pidOutput = kP * error + kI * integralSum + kD * derivative;

        // Add feedforward component
        double output = pidOutput + feedforward;

        // Clip the output to specified limits
        output = Math.max(minOutput, Math.min(maxOutput, output));

        // Save the current error for the next loop
        lastError = error;

        return output;
    }

    // Set feedforward term
    public void setFeedforward(double feedforward) {
        this.feedforward = feedforward;
    }

    // Set output limits
    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    // Reset the controller (clears integral and error terms)
    public void reset() {
        integralSum = 0;
        lastError = 0;
    }
}

