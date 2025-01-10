package org.firstinspires.ftc.teamcode.DUC_auto_disabled;

import static org.firstinspires.ftc.teamcode.lib.Hardware.closeClawAngle;
import static org.firstinspires.ftc.teamcode.lib.Hardware.openClawAngle;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Config
@Autonomous(name = "TEST_AUTO", group = "Autonomous")
@Disabled
public class autotest extends LinearOpMode {

    int armTickPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-43, 64, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        Vector2d highRung = new Vector2d(-10, 28);

        TrajectoryActionBuilder strafeToHighRung = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(highRung, Math.toRadians(90));

        TrajectoryActionBuilder givePlayerFirstSample = drive.actionBuilder(new Pose2d(-10, 30, Math.toRadians(90)))
                .strafeTo(new Vector2d(-56, 33))
                .strafeToLinearHeading(new Vector2d(-56, 13), Math.toRadians(270))
                .strafeTo(new Vector2d(-69, 10))
                .strafeTo(new Vector2d(-69, 50))
                .strafeTo(new Vector2d(-73, 10))
                .strafeTo(new Vector2d(-82, 10))
                .strafeTo(new Vector2d(-82, 50))
                .strafeTo(new Vector2d(-82, 40));

        TrajectoryActionBuilder getFirstSpecimen = drive.actionBuilder(new Pose2d(-82, 40, Math.toRadians(270)))
                .strafeTo(new Vector2d(-65, 40))
                .strafeTo(new Vector2d(-65, 60));

        TrajectoryActionBuilder hangFirstSpecimen = drive.actionBuilder(new Pose2d(-72, 60, Math.toRadians(270)))
                        .strafeToLinearHeading(new Vector2d(-15, 40), Math.toRadians(100))
                        .strafeTo(new Vector2d(-15, 20));

        TrajectoryActionBuilder getSecondSpecimen = drive.actionBuilder(new Pose2d(-15, 20, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-73, 40), Math.toRadians(270))
                .strafeTo(new Vector2d(-73, 60));

        TrajectoryActionBuilder hangSecondSpecimen = drive.actionBuilder(new Pose2d(-73, 60, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(-5, 40), Math.toRadians(100))
                .strafeTo(new Vector2d(-5, 20));



        Actions.runBlocking(claw.closeClaw());

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                    strafeToHighRung.build(),
                                    arm.highRung()
                                ),
                                arm.highRung2(),
                                new SleepAction(0.25),
                                claw.openClaw(),
                                new SleepAction(1),
                                givePlayerFirstSample.build(),
                                new ParallelAction(
                                        arm.low(),
                                        getFirstSpecimen.build()
                                ),
                                claw.closeClaw(),
                                new ParallelAction(
                                        arm.highRung(),
                                        hangFirstSpecimen.build()
                                ),
                                arm.highRung2(),
                                new SleepAction(0.25),
                                claw.openClaw(),
                                new ParallelAction(
                                        arm.low(),
                                        getSecondSpecimen.build()
                                ),
                                claw.closeClaw(),
                                new ParallelAction(
                                        arm.highRung(),
                                        hangSecondSpecimen.build()
                                ),
                                arm.highRung2(),
                                new SleepAction(0.25),
                                claw.openClaw()
                        ),
                        arm.keepPosition()
                )

        );

    }
    public class Arm {
        private Motor arm;

        public Arm(HardwareMap hardwareMap) {
            arm = new Motor(hardwareMap, "specimenArm");
            arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            arm.resetEncoder();
        }

        public class KeepPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setRunMode(Motor.RunMode.PositionControl);
                arm.setPositionCoefficient(0.01);
                arm.setTargetPosition(armTickPosition);
                arm.set(1);
                arm.setPositionTolerance(10);
                return true;
            }
        }
        public Action keepPosition() {
            return new KeepPosition();
        }

        public class HighRung implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 2000;
                keepPosition();
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action highRung() {
            return new HighRung();
        }
        public class HighRung2 implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 1300;
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action highRung2() {
            return new HighRung2();
        }

        public class Low implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 0;
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action low() {
            return new Low();
        }
    }
    public class Claw {
        private ServoEx claw;

        public Claw(HardwareMap hardwareMap) {
            claw = new SimpleServo(hardwareMap, "specimenClaw", 0, 180);
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(closeClawAngle);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(openClawAngle);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

}

