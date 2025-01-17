    package org.firstinspires.ftc.teamcode;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
    import com.acmerobotics.roadrunner.Action;
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
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.ColorSensor;
    import com.qualcomm.robotcore.hardware.DistanceSensor;
    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

    import java.util.ArrayList;
    import java.util.List;


    @TeleOp(name = "TeleOp")
    public class Teleop extends OpMode {

        private FtcDashboard dash = FtcDashboard.getInstance();
        private List<Action> runningActions = new ArrayList<>();



        // Hardware components
        private DcMotor  liftMotor1;
        private DcMotorEx intakeMotor;
        private ColorSensor colorSensor;    // Original color sensor
        private ColorSensor colorSensorRed; // New sensor for detecting red
        private ColorSensor colorSensorOrange; // New sensor for detecting orange
        private DistanceSensor distanceSensor;
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

        private boolean clawState = false; // Toggle state for claw
        private boolean clawButtonPressed = false; // Button state for claw toggle

        // State variables for drivetrain speed
        private boolean touchpadPressed = false;
        private boolean isHighSpeed = true; // Default to high speed
        private double speedMultiplier = 1.0;

        // PID Controller and Feedforward constants for the lift motor
        //private SimplePIDController liftPIDController;
        //private static final double FEEDFORWARD_UP = 0.0005;
        //private static final double FEEDFORWARD_DOWN = 0.0002;
        //private static final double TARGET_POSITION_INCREMENT = 8;
        private double targetPosition = 0;
        //private static final double MAX_POSITION = 3700;

        // Mecanum Drive
        private MecanumDrive drive;

        @Override
        public void init() {
            // Initialize lift motor
            liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
            liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            //liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor1.setTargetPosition(30);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Initialize Motor for Intake
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
            intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            // Initialize servos
            claw = hardwareMap.get(Servo.class, "claw");
            ex = hardwareMap.get(Servo.class, "extendo");
            ex1 = hardwareMap.get(Servo.class, "extendo1");
            ex.setDirection(Servo.Direction.REVERSE);
            intakeCRServo = hardwareMap.get(CRServo.class, "intakeServo");
            intakeMover = hardwareMap.get(Servo.class, "intakeMover");
            intakeMover1 = hardwareMap.get(Servo.class, "intakeMover1");
            bucketFlipper1 = hardwareMap.get(Servo.class, "bucketFlipper");

            colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
            colorSensorRed = hardwareMap.get(ColorSensor.class, "colorSensorRed");
            colorSensorOrange = hardwareMap.get(ColorSensor.class, "colorSensorOrange");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");

            // Initialize PID controller
            //liftPIDController = new SimplePIDController(0.002, 0.000, 0.00001);
            //liftPIDController.setTargetPosition(targetPosition);

            // Initialize Mecanum Drive directly
            drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        }

        @Override
        public void start() {
            ex.setPosition(1);
            ex1.setPosition(1);
            claw.setPosition(1);

            intakeMover1.setPosition(1);
            intakeMover.setPosition(1);
            bucketFlipper1.setPosition(0);
        }

        @Override
        public void loop() {

            TelemetryPacket packet = new TelemetryPacket();

            // updated based on gamepads

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);


            detectColorActions();

            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Alpha (overall light intensity)
            int alpha = colorSensor.alpha();

            // Distance (optional, if you want proximity data)
            double distance = distanceSensor.getDistance(DistanceUnit.CM);

            // Display the data on telemetry
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Alpha", alpha);
            telemetry.addData("Distance (cm)", distance);

            // Detect white color (all RGB values above a certain threshold)
            if (colorSensor.red() > 118 && colorSensor.green() > 200 && colorSensor.blue() > 180) {
                ex.setPosition(0.178); // Extend position for ex
                ex1.setPosition(0.178); // Extend position for ex1
                telemetry.addData("Detected Color", "White");
            }

            //liftPIDController.update(liftMotor1.getCurrentPosition());

            // Handle drivetrain speed toggle
            handleDrivetrainSpeedToggle();

            // Mecanum drive control
            if (drive != null) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y *0.65* speedMultiplier,
                                -gamepad1.left_stick_x *0.65* speedMultiplier
                        ),
                        -gamepad1.right_stick_x * 0.65 * speedMultiplier
                ));
                drive.updatePoseEstimate();
            }

            // Handle servo toggling
            handleServoToggle();

            // Handle new action with a dedicated button
            handleNewCombinedAction();

            // Handle bucket flipper toggle
            handleBucketFlipperToggle();

            // Handle claw toggle
            handleClawToggle();

            // Handle lift motor control
            handleLiftControl();

            // Update telemetry
            updateTelemetry();
        }

        private void handleDrivetrainSpeedToggle() {
            if (gamepad1.touchpad_finger_1 && !touchpadPressed) {
                isHighSpeed = !isHighSpeed;
                speedMultiplier = isHighSpeed ? 1.0 : 0.2;
                touchpadPressed = true;
            }
            if (!gamepad1.touchpad_finger_1) touchpadPressed = false;
        }

        private void handleServoToggle() {
            // Toggle intakeMover and intakeMover1 with gamepad2.a
            if (gamepad2.a && !aButtonPressed) {
                intakeMoverState = !intakeMoverState;
                intakeMover.setPosition(intakeMoverState ? 1 : 0.26);

                intakeMover1State = !intakeMover1State;
                intakeMover1.setPosition(intakeMover1State ? 1 : 0.26 );

                aButtonPressed = true;
            }
            if (!gamepad2.a) aButtonPressed = false;

            // Toggle ex and ex1 with gamepad2.b
            if (gamepad2.b && !bButtonPressed) {
                exState = !exState;
                ex.setPosition(exState ? 0.69 : 0.85);
                ex1State = !ex1State;
                ex1.setPosition(ex1State ? 0.69 : 0.95 );

                bButtonPressed = true;
            }
            if (!gamepad2.b) bButtonPressed = false;

            // Toggle intakeCRServo with gamepad2.x
            if (gamepad2.x && !xButtonPressed) {
                intakeCRServoState = !intakeCRServoState;
                intakeCRServo.setPower(intakeCRServoState ? 0.50 : 0.0);
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
                    intakeMover.setPosition(0.180);
                    intakeMover1.setPosition(0.18);
                    sleep(300);
                    ex.setPosition(0.85);
                    ex1.setPosition(0.85);


                    //intakeCRServo.setPower(1.0);
                    sleep(500); // Allow time for forward motion (500ms)

                    //intakeCRServo.setPower(0.0);

                    newActionButtonState = true;
                } else {
                    // Move back to initial position (position 0)
                    ex.setPosition(0);
                    ex1.setPosition(1);
                    intakeMover.setPosition(1);
                    intakeMover1.setPosition(1);

                    intakeCRServo.setPower(0.0);
                    sleep(1000);
                    intakeCRServo.setPower(-0.5);
                    sleep(2500);
                    intakeCRServo.setPower(0.0);

                    newActionButtonState = false;
                }

                newActionButtonPressed = true;
            }
            if (!gamepad2.dpad_down) newActionButtonPressed = false;
        }

        private void handleBucketFlipperToggle() {
            if (gamepad2.left_stick_button && !bucketFlipperButtonPressed) {
                bucketFlipperState = !bucketFlipperState;
                bucketFlipper1.setPosition(bucketFlipperState ? 0.560 : 0.0 );
                bucketFlipperButtonPressed = true;
            }
            if (!gamepad2.left_stick_button) bucketFlipperButtonPressed = false;
        }

        private void handleClawToggle() {
            if (gamepad2.right_stick_button && !clawButtonPressed) {
                clawState = !clawState;
                claw.setPosition(clawState ? 0.3 : 0.9);
                clawButtonPressed = true;
            }
            if (!gamepad2.right_stick_button) clawButtonPressed = false;
        }

        private void handleLiftControl() {
                //double currentPosition = liftMotor1.getCurrentPosition();
                //double pidOutput = liftPIDController.update(currentPosition);
                //double feedforward = (currentPosition < targetPosition) ? FEEDFORWARD_UP : FEEDFORWARD_DOWN;
           // liftMotor1.setPower(pidOutput + feedforward);
            //if (gamepad2.right_trigger > 4) {
                // Additional code if needed
           // }

            if (gamepad2.right_bumper) {
                liftMotor1.setPower(0.8);

                liftMotor1.setTargetPosition(200);
                claw.setPosition(0);

                intakeMover1.setPosition(0.13);
                intakeMover.setPosition(0.13);
            }

            if (gamepad2.touchpad_finger_1) {

                intakeMover1.setPosition(0.13);
                intakeMover.setPosition(0.13);
                sleep(200);
                liftMotor1.setPower(1.0);

                liftMotor1.setTargetPosition(4500);






            }
            if (gamepad2.dpad_right) {
                liftMotor1.setPower(1.0);
                liftMotor1.setTargetPosition(400);
            }

            if (gamepad2.dpad_up) {
                liftMotor1.setPower(1.0);
                liftMotor1.setTargetPosition(1950);
            }

            if (gamepad2.left_bumper) {
                liftMotor1.setPower(1.0);

                liftMotor1.setTargetPosition(1500);
                ;
            }
        }


        private void updateTelemetry() {
            telemetry.addData("Intake Mover State", intakeMoverState);
            telemetry.addData("Intake Mover 1 State", intakeMover1State);
            telemetry.addData("Ex State", exState);
            telemetry.addData("Ex 1 State", ex1State);
            telemetry.addData("Intake CR Servo State", intakeCRServoState);
            telemetry.addData("Bucket Flipper State", bucketFlipperState);
            telemetry.addData("Claw State", clawState);
            telemetry.addData("Lift Current Position", liftMotor1.getCurrentPosition());
            //telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Lift Motor Power", liftMotor1.getPower());
            telemetry.addData("Speed Mode", isHighSpeed ? "High Speed" : "Low Speed");
            telemetry.addData("Lift Motor Encoder Position", liftMotor1.getCurrentPosition());
            telemetry.addData("White Sensor", "R: %d, G: %d, B: %d",
                    colorSensor.red(), colorSensor.green(), colorSensor.blue());
            telemetry.addData("Red Sensor", "R: %d, G: %d, B: %d",
                    colorSensorRed.red(), colorSensorRed.green(), colorSensorRed.blue());
            telemetry.addData("Orange Sensor", "R: %d, G: %d, B: %d",
                    colorSensorOrange.red(), colorSensorOrange.green(), colorSensorOrange.blue());
            telemetry.update();
        }
        private void detectColorActions() {
            // Original color sensor: Detect white
            if (colorSensor.red() > 200 && colorSensor.green() > 200 && colorSensor.blue() > 200) {
                ex.setPosition(0.878);
                ex1.setPosition(0.878);
                telemetry.addData("Detected Color", "White");
            }

            // New color sensor: Detect red
            if (colorSensorRed.red() > 580 && colorSensorRed.green() < 1000 ) {


                intakeMover1.setPosition(0.878);
                intakeMover.setPosition(0.878);
                sleep(600);



                ex.setPosition(0.875);
                ex1.setPosition(0.875);

                sleep(200);

                intakeCRServo.setPower(-0.5);



                telemetry.addData("Detected Color", "Red");
            }

            // New color sensor: Detect orange
            if (colorSensorOrange.red() > 180 && colorSensorOrange.green() > 100 && colorSensorOrange.blue() < 100) {
                bucketFlipper1.setPosition(0.3); // Example action: Adjust bucket flipper
                telemetry.addData("Detected Color", "Orange");
            }
        }


        private void sleep(long milliseconds) {
            try {
                Thread.sleep(milliseconds);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
