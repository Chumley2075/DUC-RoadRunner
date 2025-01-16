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
@Autonomous(name = "2BLUE_BACKSTAGE_BUCKET", group = "Autonomous")
public class BLUE_BACKSTAGE_BUCKET extends LinearOpMode {

    int armTickPosition = 250;
    int spoolTickPosition = 0;
    public static int hookPosition = 1800;
    public static int myHeading = 180;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-37, -64, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        NormalClaw claw = new NormalClaw(hardwareMap);
        Spool spool = new Spool(hardwareMap);
        NormalArm arm = new NormalArm(hardwareMap);

        Vector2d highBasket = new Vector2d(-47, -54);

        TrajectoryActionBuilder goToHighBasket = drive.actionBuilder(initialPose)
                .waitSeconds(0.25)
                .strafeToLinearHeading(new Vector2d(-70, -59), Math.toRadians(225));

        TrajectoryActionBuilder leaveHighBasket = drive.actionBuilder(new Pose2d(-70, -59, Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(-50, -50), Math.toRadians(225));

        TrajectoryActionBuilder parkTier1Ascent = drive.actionBuilder(new Pose2d(-10, 28, Math.toRadians(225)))
                .strafeToLinearHeading(new Vector2d(40, 8), Math.toRadians(0))
                .strafeTo(new Vector2d(23, 8));


        Actions.runBlocking(claw.closeClaw());

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                claw.closeClaw(),
                            new SleepAction(1),
                                arm.highBasket(),
                                new SleepAction(1),
                                spool.highBasket(),
                                new SleepAction(1),
                            goToHighBasket.build(),
                            new SleepAction(2),
                                claw.openClaw(),
                            new SleepAction(1),
                            spool.low(),
                                new SleepAction(1),
                                parkTier1Ascent.build(),
                            arm.mid()
                        ),
                        arm.keepPosition(),
                        spool.keepPosition()
                )

        );

    }

    public class Spool {
        private Motor spool;

        public Spool(HardwareMap hardwareMap) {
            spool = new Motor(hardwareMap, "spool");
            spool.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            spool.resetEncoder();
        }

        public class HighBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spoolTickPosition = 2500;
                if (spool.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action highBasket() {
            return new HighBasket();
        }

        public class Low implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spoolTickPosition = 0;
                if (spool.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action low() {
            return new Low();
        }

        public class KeepPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                spool.setRunMode(Motor.RunMode.PositionControl);
                spool.setPositionCoefficient(0.01);
                spool.setTargetPosition(spoolTickPosition);
                spool.set(1);
                spool.setPositionTolerance(10);
                return true;
            }
        }
        public Action keepPosition() {
            return new KeepPosition();
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

    public class NormalArm {
        private Motor arm;

        public NormalArm(HardwareMap hardwareMap) {
            arm = new Motor(hardwareMap, "arm");
            arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            arm.setInverted(true);
            arm.resetEncoder();
        }

        public class KeepPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setRunMode(Motor.RunMode.PositionControl);
                arm.setPositionCoefficient(0.01);
                arm.setTargetPosition(armTickPosition);
                arm.set(0.75);
                arm.setPositionTolerance(10);
                return true;
            }
        }
        public Action keepPosition() {
            return new KeepPosition();
        }

        public class HighBasket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 2500;
                keepPosition();
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action highBasket() {
            return new HighBasket();
        }
        public class Mid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 1900;
                keepPosition();
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action mid() {
            return new Mid();
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
    public class NormalClaw {
        private ServoEx claw;

        public NormalClaw(HardwareMap hardwareMap) {
            claw = new SimpleServo(hardwareMap, "claw", 0, 180);
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

