package org.firstinspires.ftc.teamcode.DUC_auto;

import static org.firstinspires.ftc.teamcode.lib.Hardware.closeClawAngle;
import static org.firstinspires.ftc.teamcode.lib.Hardware.openClawAngle;

import androidx.annotation.NonNull;

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
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "2RED_FRONTSTAGE_CLIP", group = "Autonomous")
public class RED_FRONTSTAGE_CLIP extends LinearOpMode {

    int armTickPosition = 250;
    public static int hookPosition = 1800;
    public static int myHeading = 180;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-37, -64, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Spool spool = new Spool(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        Vector2d ascentArea = new Vector2d(-10, -15);
        Vector2d highRung = new Vector2d(-5, -28);

        TrajectoryActionBuilder strafeToHighRung = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(highRung, Math.toRadians(270));

        TrajectoryActionBuilder parkTier1Ascent = drive.actionBuilder(new Pose2d(-5, -28, Math.toRadians(270)))
                .strafeTo(new Vector2d(-40, -35))
                .strafeTo(new Vector2d(-40, -8))
                .turnTo(Math.toRadians(180))
                .strafeTo(new Vector2d(-23, -8));



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
                                new SleepAction(0.25),
                                parkTier1Ascent.build()
                        ),
                        arm.keepPosition()
                )

        );

    }

    public class Spool {
        private DcMotorEx spool;

        public Spool(HardwareMap hardwareMap) {
            spool = hardwareMap.get(DcMotorEx.class, "spool");
            spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            spool.setDirection(DcMotorSimple.Direction.FORWARD);
        }
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
            return new Arm.KeepPosition();
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
            return new Arm.HighRung();
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
            return new Arm.HighRung2();
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
            return new Arm.Low();
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
            return new Claw.CloseClaw();
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(openClawAngle);
                return false;
            }
        }
        public Action openClaw() {
            return new Claw.OpenClaw();
        }
    }

}

