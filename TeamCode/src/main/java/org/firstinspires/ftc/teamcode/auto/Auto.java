package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

//Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous(name="Autonomous")
public class Auto extends LinearOpMode {
    public class Lift {
        private DcMotorEx liftMotor1;
        public Lift(HardwareMap aHardwareMap) {
            liftMotor1 = aHardwareMap.get(DcMotorEx.class, "liftMotor1");
            liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
            liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class UpperGoalPosition implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(!initialized) {
                    liftMotor1.setPower(0.75);
                    initialized = true;
                }

                double position = liftMotor1.getCurrentPosition();
                packet.put("ViperPosition", position);

                if(position < 2350) {
                    return true;
                } else {
                    liftMotor1.setPower(0.0);
                    return false;
                }
            }
        }

        public Action liftUpperGoalPosition() {
            return new UpperGoalPosition();
        }

        public class LowerStoredPosition implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(!initialized) {
                    liftMotor1.setPower(0.75);
                    initialized = true;
                }

                double position = liftMotor1.getCurrentPosition();
                packet.put("liftPosition", position);

                if(position > 0) {
                    return true;
                } else {
                    liftMotor1.setPower(0.0);
                    return false;
                }
            }
        }

        public Action lowerStoredPosition() {
            return new LowerStoredPosition();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initalPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initalPose);
        Lift lift = new Lift(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initalPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(45))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 50))
                .turn(Math.toRadians(90))
                .lineToX(47.5)
                .waitSeconds(3);

        Actions.runBlocking(new SequentialAction(
                tab1.build(),
                lift.liftUpperGoalPosition()
        ));
    }
}
