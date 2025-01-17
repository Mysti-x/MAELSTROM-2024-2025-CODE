package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private DcMotorEx intakeMotor;

    private Servo claw;
    private Servo ex;
    private Servo ex1;
    private CRServo intakeCRServo;
    private Servo intakeMover;
    private Servo intakeMover1;
    private Servo bucketFlipper1;

    // State variables for servos
    private boolean intakeMoverState = false;
    private boolean intakeMover1State = false;
    private boolean exState = false;
    private boolean ex1State = false;
    private boolean intakeCRServoState = false;

    private boolean aButtonPressed = false;
    private boolean bButtonPressed = false;
    private boolean xButtonPressed = false;
    private boolean yButtonPressed = false;
    private boolean dpad_upButtonPressed = false;
    private boolean newActionButtonPressed = false; // State for new button

    private boolean newActionButtonState = false; // Toggle state for dpad_down

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

        // Initialize Motor for Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize servos
        claw = hardwareMap.get(Servo.class, "claw");
        ex = hardwareMap.get(Servo.class, "extendo");
        ex1 = hardwareMap.get(Servo.class, "extendo1");
        intakeCRServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeMover = hardwareMap.get(Servo.class, "intakeMover");
        intakeMover1 = hardwareMap.get(Servo.class, "intakeMover1");
        bucketFlipper1 = hardwareMap.get(Servo.class, "bucketFlipper");

        // Initialize PID controller
        liftPIDController = new SimplePIDController(0.002, 0.000, 0.00001);
        liftPIDController.setTargetPosition(targetPosition);

        // Initialize Mecanum Drive directly
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void start() {
        claw.setPosition(0);
        //ex.setPosition(0);
        //ex1.setPosition(1);
        intakeMover1.setPosition(0.7);
        intakeMover.setPosition(0.7);
        bucketFlipper1.setPosition(0);
    }

    @Override
    public void loop() {
        liftPIDController.update(liftMotor1.getCurrentPosition());

        // Mecanum drive control
        if (drive != null) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * 0.85,
                            -gamepad1.left_stick_x * 0.85
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();
        }

        // Handle servo toggling
        handleServoToggle();

        // Handle new action with a dedicated button
        handleNewCombinedAction();

        // Handle lift motor control
        handleLiftControl();

        // Update telemetry
        updateTelemetry();
    }

    private void handleServoToggle() {
        // Toggle intakeMover and intakeMover1 with gamepad2.a
        if (gamepad2.a && !aButtonPressed) {
            intakeMoverState = !intakeMoverState;
            intakeMover.setPosition(intakeMoverState ? 1.0 : 0.18);

            intakeMover1State = !intakeMover1State;
            intakeMover1.setPosition(intakeMover1State ? 1.0 : 0.18 );

            aButtonPressed = true;
        }
        if (!gamepad2.a) aButtonPressed = false;

        // Toggle ex and ex1 with gamepad2.b
        if (gamepad2.b && !bButtonPressed) {
            exState = !exState;
            ex.setPosition(exState ? 0.12 : 0.);
            ex1State = !ex1State;
            ex1.setPosition(ex1State ? 0.88 : 1.0 );

            bButtonPressed = true;
        }
        if (!gamepad2.b) bButtonPressed = false;

        // Toggle intakeCRServo with gamepad2.x
        if (gamepad2.x && !xButtonPressed) {
            intakeCRServoState = !intakeCRServoState;
            intakeCRServo.setPower(intakeCRServoState ? 1.0 : 0.0);
            xButtonPressed = true;
        }
        if (!gamepad2.x) xButtonPressed = false;


        if (gamepad2.y && !yButtonPressed) {
            intakeCRServoState = !intakeCRServoState;
            intakeCRServo.setPower(intakeCRServoState ? -1.0 : 0.0);
            yButtonPressed = true;
        }
        if (!gamepad2.y) yButtonPressed = false;
    }

    private void handleNewCombinedAction() {
        if (gamepad2.dpad_down && !newActionButtonPressed) { // When dpad_down is pressed, toggle action
            if (!newActionButtonState) {
                // Move to target position (position 1)
                ex.setPosition(0.35);
                ex1.setPosition(0.45);
                intakeMover.setPosition(0.0);
                intakeMover1.setPosition(0.0);

                // Move intakeCRServo in forward direction
                intakeCRServo.setPower(1.0);
                sleep(500); // Allow time for forward motion (500ms)

                // Reverse intakeCRServo
                //intakeCRServo.setPower(-1.0);
                //sleep(500); // Allow time for reverse motion (500ms)

                // Stop intakeCRServo
                intakeCRServo.setPower(0.0);

                // Update toggle state
                newActionButtonState = true;
            } else {
                // Move back to initial position (position 0)
                ex.setPosition(0);
                ex1.setPosition(1);
                intakeMover.setPosition(1);
                intakeMover1.setPosition(1);
                intakeCRServo.setPower(0.0);

                // Update toggle state
                newActionButtonState = false;
            }

            newActionButtonPressed = true;
        }
        if (!gamepad2.dpad_down) newActionButtonPressed = false;
    }

    private void handleLiftControl() {
        if (gamepad2.right_trigger > 0) { // Check if the right trigger is pressed
            //extendo.setPosition(1);
            //intakeMotor.setPower(1); // Run the motor at full power
        } else {
            // intakeMotor.setPower(0);
            //extendo.setPosition(0); // Stop the motor when the trigger is not pressed
        }


        if (gamepad2.right_bumper) {
            liftPIDController.setTargetPosition(0);

            //code for zero postion, so when the viper is all the way down
        }

        if (gamepad2.dpad_left) {
            liftPIDController.setTargetPosition(3900);
            //code for top basket
        }
        if (gamepad2.dpad_right) {
            liftPIDController.setTargetPosition(400);

            //code  to take specimen from wall
        }

        if (gamepad2.dpad_up) {
            liftPIDController.setTargetPosition(2350.00);
            //code for height of top bar for specimen

        }
        //if (gamepad2.dpad_down)
        //{
        //liftPIDController.setTargetPosition();
        //}


        if (gamepad2.left_bumper) {
            liftPIDController.setTargetPosition(450.00);
            //code for height of top bar for specimen

        }
        //if (gamepad2.dpad_down)
        //{
        // liftPIDController.setTargetPosition(2000.00);  // Set the target position

        //ElapsedTime timer = new ElapsedTime();
        //timer.reset();

        // Step 1: Move the servo first
        //toggleServo(clawmover, 0.15);  // Move the servo first
        //clawmover.setPosition(0.15);

        //while (timer.seconds() < 0.3) {
        //}  // Wait for 0.3 seconds

        // Step 2: Move the lift motor using PID control
        //while (Math.abs(liftMotor1.getCurrentPosition() - liftPIDController.getTargetPosition()) > 10) {  // Tolerance of 10 ticks
        //double pidOutput = liftPIDController.update(liftMotor1.getCurrentPosition());  // Get PID output

        // Debugging telemetry
        //telemetry.addData("Lift Current Position", liftMotor1.getCurrentPosition());
        //telemetry.addData("PID Output", pidOutput);
        //telemetry.addData("Target Position", liftPIDController.getTargetPosition());
        ///telemetry.update();

        // Apply the PID output to the motor power
        // Ensure the motor gets a minimum power if PID output is too low
        //double motorPower = Math.max(pidOutput, -0.2); // Ensure at least 20% power to move the motor
        //liftMotor1.setPower(motorPower);
        //}

        // After lift motor reaches target, stop the motor
        //liftMotor1.setPower(0);

        // Step 3: Run the robot for 0.5 seconds
        //timer.reset();
        //while (timer.seconds() < 0.0)
        //{
        //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Set the drive powers to move backward at maximum speed
        //drive.setDrivePowers(new PoseVelocity2d(
        //new Vector2d(0.0, 0), // Move backward at 100% power
        //0 // No rotational movement
        //));

        //drive.updatePoseEstimate();
        //}

        // Step 4: Stop the robot after the movement
        //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //drive.setDrivePowers(new PoseVelocity2d(
        //new Vector2d(0, 0), // Stop all movement
        // 0
        //));

        // Step 5: Toggle the claw after the robot has stopped
        //toggleServo(claw, 0);
        //liftPIDController.setTargetPosition(0);
        // }


        //double currentPosition = liftMotor1.getCurrentPosition();
        //double pidOutput = liftPIDController.update(currentPosition);
        //double feedforward = (currentPosition < targetPosition) ? FEEDFORWARD_UP : FEEDFORWARD_DOWN;

        //liftMotor1.setPower(pidOutput + feedforward);
        //}
    }
        private void updateTelemetry () {
            telemetry.addData("Intake Mover State", intakeMoverState);
            telemetry.addData("Intake Mover 1 State", intakeMover1State);
            telemetry.addData("Ex State", exState);
            telemetry.addData("Ex 1 State", ex1State);
            telemetry.addData("Intake CR Servo State", intakeCRServoState);
            telemetry.addData("Lift Current Position", liftMotor1.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Lift Motor Power", liftMotor1.getPower());
            telemetry.update();
        }

    private void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
