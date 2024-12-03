package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.ParallelAction;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BLUE_SIDE_AUTO_", group = "Autonomous")
public class BlueSideTestAuto extends LinearOpMode {

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
                    liftMotor1.setPower(0.8);
                    initialized = true;
                }

                double pos = liftMotor1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
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
                    liftMotor1.setPower(-0.8);
                    initialized = true;
                }

                double pos = liftMotor1.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
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
                claw.setPosition(0.05);
                return false;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }

        public Action openClaw() { // Fixed method
            return new OpenClaw();
        }
    }

    @Override
    public void runOpMode() {
        // Set the initial position and heading of the robot
        Pose2d initialPose = new Pose2d(-60, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // Define the trajectory action
        Action trajectoryAction = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(44.5, 30))  // Move to a specific position
                .turn(Math.toRadians(180))        // Turn the robot 180 degrees
                .strafeTo(new Vector2d(48, 12))   // Move to another position
                .build();

        // Close the claw at the start of the OpMode
        Actions.runBlocking(claw.closeClaw());

        // Wait for the start signal
        waitForStart();

        if (isStopRequested()) return; // Exit if the stop signal is received

        // Execute the trajectory with parallel actions
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryAction,
                        lift.liftUp(),
                        claw.openClaw(), // This now works as expected
                        lift.liftDown()
                )
        );
    }
}
