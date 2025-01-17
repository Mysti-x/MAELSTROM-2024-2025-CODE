package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.ParallelAction;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BLUE_SIDE_SPECIMEN_AUTO_", group = "Autonomous")
public class BlueSideSpecimenAuto extends LinearOpMode {

    // Lift Class
    public class Lift {
        private DcMotorEx liftMotor1;

        public Lift(HardwareMap hardwareMap) {
            liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
            liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            liftMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftMotor1.setPower(0.9);
                    initialized = true;
                }

                double pos = liftMotor1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1950) {
                    return true;
                } else {
                    liftMotor1.setPower(0);
                    return false;
                }
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftMotor1.setPower(-0.9);
                    initialized = true;
                }


                double pos = liftMotor1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 1350) {
                    liftMotor1.setPower(-0.9);
                    return true;
                } else {
                    liftMotor1.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }

        public class LiftTo150 implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftMotor1.setPower(0.9);
                    initialized = true;
                }

                double pos = liftMotor1.getCurrentPosition();
                packet.put("liftPos", pos);

                if (pos > 150) {
                    liftMotor1.setPower(-1.0);

                    return true;
                } else {
                    liftMotor1.setPower(0);
                    return false;
                }
            }
        }

        public Action liftTo150() {
            return new LiftTo150();
        }
    }

    // Claw Class
    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.3);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.9);
                return false;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }
    }

    // Extendos Class
    public static class Extendos {
        private Servo ex;
        private Servo ex1;

        public Extendos(HardwareMap hardwareMap) {
            ex1 = hardwareMap.get(Servo.class, "extendo1");
            ex = hardwareMap.get(Servo.class, "extendo");
        }

        public class CloseExtendo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ex.setPosition(0.90);
                ex1.setPosition(0.90);
                return false;
            }
        }

        public Action closeExtendo() {
            return new CloseExtendo();
        }

        public class OpenExtendo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                ex.setPosition(0.178);
                ex1.setPosition(0.178);
                return false;
            }
        }

        public Action openExtendo() {
            return new OpenExtendo();
        }
    }

    // BucketFlipper Class
    public class BucketFlipper {
        private Servo bucketFlipper1;

        public BucketFlipper(HardwareMap hardwareMap) {
            bucketFlipper1 = hardwareMap.get(Servo.class, "bucketFlipper");
        }

        public class CloseBucket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucketFlipper1.setPosition(0.00);
                return false;
            }
        }

        public Action closeBucket() {
            return new CloseBucket();
        }

        public class OpenBucket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucketFlipper1.setPosition(0.560);
                return false;
            }
        }

        public Action openBucket() {
            return new OpenBucket();
        }
    }

    // IntakeMover Class
    public class IntakeMover {
        private Servo intakeMover;
        private Servo intakeMover1;

        public IntakeMover(HardwareMap hardwareMap) {
            intakeMover = hardwareMap.get(Servo.class, "intakeMover");
            intakeMover1 = hardwareMap.get(Servo.class, "intakeMover1");
        }

        public class CloseBucket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeMover.setPosition(0.55);
                intakeMover1.setPosition(0.55);
                return false;
            }
        }

        public Action closeBucket() {
            return new CloseBucket();
        }

        public class OpenIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeMover.setPosition(1.0);
                intakeMover1.setPosition(1.0);
                return false;
            }
        }

        public Action openIntake() {
            return new OpenIntake();
        }
    }

    // IntakeServo Class
    public class IntakeServo {
        private CRServo intakeCRServo;

        public IntakeServo(HardwareMap hardwareMap) {
            intakeCRServo = hardwareMap.get(CRServo.class, "intakeServo");
        }

        public class BackwardsSpin implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeCRServo.setPower(-1);
                return false;
            }
        }

        public Action BackwardsSpin() {
            return new BackwardsSpin();
        }

        public class ForwardSpin implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeCRServo.setPower(0.5);
                return false;
            }
        }

        public Action ForwardSpin() {
            return new ForwardSpin();
        }
    }

    @Override
    public void runOpMode() {
        // Set the initial position and heading of the robot
        Pose2d initialPose = new Pose2d(-11.1, 60, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Extendos extendos = new Extendos(hardwareMap);

        // Define the trajectory action
        Action trajectoryAction = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-11.1, 28.9))
                .waitSeconds(1.3)
                .strafeTo(new Vector2d(-11.1, 45.9))


                .setTangent(9)
                .splineToLinearHeading(new Pose2d(-46.1, 13.8,Math.toRadians(90)), Math.PI / 2)
                .strafeTo(new Vector2d(-46.1, 60.5))
                .waitSeconds(0.8)
                .setTangent(1)
                .splineToSplineHeading(new Pose2d(-6.1, 36.9,Math.toRadians(270)), Math.PI / 10)
                .strafeTo(new Vector2d(-6.1, 28.9))
                .waitSeconds(0.8)

                .strafeTo(new Vector2d(-6.1, 40.9))

                .splineToLinearHeading(new Pose2d(-58.1, 13.9,Math.toRadians(90)), Math.PI / 2)

                .strafeTo(new Vector2d(-58.1, 56))

                .splineToSplineHeading(new Pose2d(-1.1, 36.9,Math.toRadians(270)), Math.PI / 10)

                .strafeTo(new Vector2d(-1.1, 28.9))
                .waitSeconds(0.8)
                .strafeTo(new Vector2d(-1.1, 45.9))

                .splineToSplineHeading(new Pose2d(-37.1, 57.9,Math.toRadians(90)), Math.PI / 1)
                .waitSeconds(0.5)
                .splineToSplineHeading(new Pose2d(5.1, 36.9,Math.toRadians(270)), Math.PI / 10)
                .strafeTo(new Vector2d(5.1, 28.9))



                .build();

        // Close the claw at the start of the OpMode
        Actions.runBlocking(claw.closeClaw());

        // Wait for the start signal
        waitForStart();

        if (isStopRequested()) return;

        // Execute the trajectory with parallel actions
        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        trajectoryAction,
                        new SequentialAction(
                                extendos.closeExtendo(),
                                lift.liftUp(),
                                new SleepAction(0.5),
                                lift.liftDown(),
                                new SleepAction(3.8),

                                claw.openClaw(),
                                new SleepAction(0.2),

                                lift.liftTo150(),
                                new SleepAction(0.6)
                        )
                )
        ));
    }
}
