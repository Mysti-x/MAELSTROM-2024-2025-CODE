package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp")
public class Teleop extends OpMode {
    // Hardware components
    private DcMotorEx liftMotor1;
    private Servo servomover;
    private Servo clawmover;
    private Servo claw;
    private Servo extendo;
    private DcMotorEx intakeMotor;
    // State variables for servos
    private boolean aButtonPressed = false;
    private boolean servoPosition = false;

    // PID Controller and Feedforward constants for the lift motor
    private SimplePIDController liftPIDController;
    private static final double FEEDFORWARD_UP = 0.0005;
    private static final double FEEDFORWARD_DOWN = 0.0002;
    private static final double TARGET_POSITION_INCREMENT = 8;
    private double targetPosition = 0;
    private static final double MAX_POSITION = 3700;

    // Mecanum Drive
    private MecanumDrive drive;

    @Override
    public void init() {
        // Initialize lift motor
        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //

        // Initialize servos
        servomover = hardwareMap.get(Servo.class, "servomover");
        clawmover = hardwareMap.get(Servo.class, "clawmover");
        claw = hardwareMap.get(Servo.class, "claw");
        extendo  =hardwareMap.get(Servo.class, "extendo");

        // Initialize PID controller
        liftPIDController = new SimplePIDController(0.002, 0.000, 0.00001);
        liftPIDController.setTargetPosition(targetPosition);

        // Initialize Mecanum Drive directly
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void start() {
        servomover.setPosition(0.0);
        clawmover.setPosition(0.3);
        claw.setPosition(1);
    }

    @Override
    public void loop() {
        liftPIDController.update(liftMotor1.getCurrentPosition());
        // Mecanum drive control
        if (drive != null) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y*0.85,
                            -gamepad1.left_stick_x*0.85
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();
        }

        // Handle servo toggling
        handleServoToggle();

        // Handle lift motor control
        handleLiftControl();

        // Update telemetry
        updateTelemetry();
    }

    /**
     * Handles toggling of servos using gamepad2 buttons.
     */
    private void handleServoToggle() {
        if (gamepad2.a && !aButtonPressed) {
            toggleServo(servomover, 0.3);
        } else if (gamepad2.x && !aButtonPressed) {
            toggleServo(servomover, 0.38);
        } else if (gamepad2.b && !aButtonPressed) {
            toggleServo(clawmover, 0.35);
        } else if (gamepad2.y && !aButtonPressed) {
            toggleServo(claw, 0.3);
        } else if (!gamepad2.a && !gamepad2.x && !gamepad2.b && !gamepad2.y) {
            aButtonPressed = false; // Reset button state when no relevant button is pressed
        }
    }

    /**
     * Toggles a servo position between two states.
     *
     * @param servo The servo to toggle
     * @param position The new position to toggle to
     */
    private void toggleServo(Servo servo, double position) {
        aButtonPressed = true;
        servoPosition = !servoPosition;
        servo.setPosition(servoPosition ? position : 0);
    }

    /**
     * Handles lift motor control using gamepad2 bumpers.
     */
    private void handleLiftControl() {
        if (gamepad2.right_trigger)
        {

        }

        if (gamepad2.right_bumper)
        {
            liftPIDController.setTargetPosition(0);

            //code for zero postion, so when the viper is all the way down
        }

        if (gamepad2.dpad_left)
        {
            liftPIDController.setTargetPosition(3900);
            //code for top basket
        }
        if (gamepad2.dpad_right)
        {
            liftPIDController.setTargetPosition(400);

            //code  to take specimen from wall
        }

        if (gamepad2.dpad_up)
        {
            liftPIDController.setTargetPosition(2350.00);
            //code for height of top bar for specimen

        }
        if (gamepad2.left_bumper)
        {
            liftPIDController.setTargetPosition(450.00);
            //code for height of top bar for specimen

        }
            if (gamepad2.dpad_down)
            {
                liftPIDController.setTargetPosition(2000.00);  // Set the target position

                ElapsedTime timer = new ElapsedTime();
                timer.reset();

                // Step 1: Move the servo first
                //toggleServo(clawmover, 0.15);  // Move the servo first
                clawmover.setPosition(0.15);

                while (timer.seconds() < 0.3) {
                }  // Wait for 0.3 seconds

                // Step 2: Move the lift motor using PID control
                while (Math.abs(liftMotor1.getCurrentPosition() - liftPIDController.getTargetPosition()) > 10) {  // Tolerance of 10 ticks
                    double pidOutput = liftPIDController.update(liftMotor1.getCurrentPosition());  // Get PID output

                    // Debugging telemetry
                    telemetry.addData("Lift Current Position", liftMotor1.getCurrentPosition());
                    telemetry.addData("PID Output", pidOutput);
                    telemetry.addData("Target Position", liftPIDController.getTargetPosition());
                    telemetry.update();

                    // Apply the PID output to the motor power
                    // Ensure the motor gets a minimum power if PID output is too low
                    double motorPower = Math.max(pidOutput, -0.2); // Ensure at least 20% power to move the motor
                    liftMotor1.setPower(motorPower);
                }

                // After lift motor reaches target, stop the motor
                liftMotor1.setPower(0);

                // Step 3: Run the robot for 0.5 seconds
                timer.reset();
                while (timer.seconds() < 0.0)
                {
                    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                    // Set the drive powers to move backward at maximum speed
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(0.0, 0), // Move backward at 100% power
                            0 // No rotational movement
                    ));

                    drive.updatePoseEstimate();
                }

                // Step 4: Stop the robot after the movement
                MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(0, 0), // Stop all movement
                        0
                ));

                // Step 5: Toggle the claw after the robot has stopped
                //toggleServo(claw, 0);
                liftPIDController.setTargetPosition(0);
            }


            double currentPosition = liftMotor1.getCurrentPosition();
            double pidOutput = liftPIDController.update(currentPosition);
            double feedforward = (currentPosition < targetPosition) ? FEEDFORWARD_UP : FEEDFORWARD_DOWN;

            liftMotor1.setPower(pidOutput + feedforward);
        }

    /**
     * Updates telemetry for debugging purposes.
     */
        private void updateTelemetry()
        {
            telemetry.addData("Lift Current Position", liftMotor1.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Lift Motor Power", liftMotor1.getPower());
            telemetry.addData("Servo Position", servomover.getPosition());
            telemetry.addData("Lift power", liftMotor1.getPower());
            telemetry.update();
        }
}
