package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
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

    private boolean bucketFlipperState = false; // Toggle state for bucket flipper
    private boolean bucketFlipperButtonPressed = false; // Button state for bucket flipper

    // State variables for drivetrain speed
    private boolean touchpadPressed = false;
    private boolean isHighSpeed = true; // Default to high speed
    private double speedMultiplier = 1.0;

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

        intakeMover1.setPosition(0.7);
        intakeMover.setPosition(0.7);
        bucketFlipper1.setPosition(0);
    }

    @Override
    public void loop() {
        liftPIDController.update(liftMotor1.getCurrentPosition());

        // Handle drivetrain speed toggle
        handleDrivetrainSpeedToggle();

        // Mecanum drive control
        if (drive != null) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * speedMultiplier,
                            -gamepad1.left_stick_x * speedMultiplier
                    ),
                    -gamepad1.right_stick_x * speedMultiplier
            ));
            drive.updatePoseEstimate();
        }

        // Handle servo toggling
        handleServoToggle();

        // Handle new action with a dedicated button
        handleNewCombinedAction();

        // Handle bucket flipper toggle
        handleBucketFlipperToggle();

        // Handle lift motor control
        handleLiftControl();

        // Update telemetry
        updateTelemetry();
    }

    private void handleDrivetrainSpeedToggle() {
        if (gamepad1.touchpad_finger_1 && !touchpadPressed) {
            isHighSpeed = !isHighSpeed;
            speedMultiplier = isHighSpeed ? 1.0 : 0.5;
            touchpadPressed = true;
        }
        if (!gamepad1.touchpad_finger_1) touchpadPressed = false;
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
            ex.setPosition(exState ? 0.12 : 0.00);
            ex1State = !ex1State;
            ex1.setPosition(ex1State ? 0.88 : 1 );

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
                ex.setPosition(0.12);
                ex1.setPosition(0.88);
                intakeMover.setPosition(0.180);
                intakeMover1.setPosition(0.18);

                // Move intakeCRServo in forward direction
                //intakeCRServo.setPower(1.0);
                sleep(500); // Allow time for forward motion (500ms)

                // Stop intakeCRServo
                //intakeCRServo.setPower(0.0);

                // Update toggle state
                newActionButtonState = true;
            } else {
                // Move back to initial position (position 0)
                ex.setPosition(0);
                ex1.setPosition(1);
                intakeMover.setPosition(0.18);
                intakeMover1.setPosition(0.18);

                intakeCRServo.setPower(0.0);
                sleep(1000);
                intakeCRServo.setPower(-1.0);

                // Update toggle state
                newActionButtonState = false;
            }

            newActionButtonPressed = true;
        }
        if (!gamepad2.dpad_down) newActionButtonPressed = false;
    }

    private void handleBucketFlipperToggle() {
        if (gamepad1.left_stick_button && !bucketFlipperButtonPressed) {
            bucketFlipperState = !bucketFlipperState;
            bucketFlipper1.setPosition(bucketFlipperState ? 0.630 : 0.0);
            bucketFlipperButtonPressed = true;
        }
        if (!gamepad1.left_stick_button) bucketFlipperButtonPressed = false;
    }

    private void handleLiftControl() {
        double currentPosition = liftMotor1.getCurrentPosition();
        double pidOutput = liftPIDController.update(currentPosition);
        double feedforward = (currentPosition < targetPosition) ? FEEDFORWARD_UP : FEEDFORWARD_DOWN;

        liftMotor1.setPower(pidOutput + feedforward);
        if (gamepad2.right_trigger > 4) {
            // Additional code if needed
        }

        if (gamepad2.right_bumper) {
            liftPIDController.setTargetPosition(3);
            // zero postion

        }

        if (gamepad2.dpad_left) {
            liftPIDController.setTargetPosition(3900);
            //postion for top basket
        }
        if (gamepad2.dpad_right) {
            liftPIDController.setTargetPosition(400);
            //wall postion
        }

        if (gamepad2.dpad_up) {
            liftPIDController.setTargetPosition(2350.00);
            // postion for top bar
        }

        if (gamepad2.left_bumper) {
            liftPIDController.setTargetPosition(450.00);
            //up from wall postion
        }


    }

    private void updateTelemetry() {
        telemetry.addData("Intake Mover State", intakeMoverState);
        telemetry.addData("Intake Mover 1 State", intakeMover1State);
        telemetry.addData("Ex State", exState);
        telemetry.addData("Ex 1 State", ex1State);
        telemetry.addData("Intake CR Servo State", intakeCRServoState);
        telemetry.addData("Bucket Flipper State", bucketFlipperState);
        telemetry.addData("Lift Current Position", liftMotor1.getCurrentPosition());
        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Lift Motor Power", liftMotor1.getPower());
        telemetry.addData("Speed Mode", isHighSpeed ? "High Speed" : "Low Speed");
        telemetry.addData("Lift Motor Encoder Postion", liftMotor1.getCurrentPosition());
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
