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
@Autonomous(name = "RED_BACKSTAGE_CLIP", group = "Autonomous")
public class RED_BACKSTAGE_CLIP extends LinearOpMode {


    int armTickPosition = 250;
    public static int hookPosition = 1700;
    boolean clawClosed = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(37, -64, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Spool spool = new Spool(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        Pose2d highRung = new Pose2d(37, -35, Math.toRadians(90));
        Pose2d highRungLatch = new Pose2d(37, -30, Math.toRadians(90));
        Vector2d observation = new Vector2d(65, -60);
        Pose2d ascentArea = new Pose2d(-10, -15, Math.toRadians(20));

        TrajectoryActionBuilder strafeToHighRung = drive.actionBuilder(initialPose)
                .splineToLinearHeading(highRung, Math.toRadians(90));

        TrajectoryActionBuilder highRungLatch1 = drive.actionBuilder(highRung)
                .splineToLinearHeading(highRungLatch, Math.toRadians(90));

        TrajectoryActionBuilder highRungLatch2 = drive.actionBuilder(highRungLatch)
                .waitSeconds(1)
                .splineToLinearHeading(highRung, Math.toRadians(90));

        TrajectoryActionBuilder parkObservation = drive.actionBuilder(highRung)
                .strafeTo(observation);

        TrajectoryActionBuilder parkTier1Ascent = drive.actionBuilder(highRung)
                .strafeTo(new Vector2d(-37, -40))
                .strafeTo(new Vector2d(-37, -15))
                .splineToLinearHeading(ascentArea, Math.toRadians(90));



        Actions.runBlocking(claw.closeClaw());

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                claw.closeClaw(),
                                new ParallelAction(
                                    strafeToHighRung.build(),
                                    arm.highRung()
                                ),
                                highRungLatch1.build(),
                                arm.highRung2(),
                                new SleepAction(1),
                                claw.openClaw(),
                                highRungLatch2.build(),
                                new SleepAction(1),
                                parkObservation.build(),
                                arm.low(),
                                claw.closeClaw()
                        ),
                        arm.keepPosition(),
                        claw.tightenClaw()
                )

        );

        stop();

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
            arm = new Motor(hardwareMap, "arm");
            arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            arm.setInverted(true);
        }

        public class KeepPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setRunMode(Motor.RunMode.PositionControl);
                arm.setPositionCoefficient(0.01);
                arm.setTargetPosition(armTickPosition);
                arm.set(.65);
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
                armTickPosition = hookPosition;
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
                armTickPosition = hookPosition-200;
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
            claw = new SimpleServo(hardwareMap, "claw", 0, 180);
        }
        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.turnToAngle(closeClawAngle);
                clawClosed = true;
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }
        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                clawClosed = false;
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
        public class TightClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (clawClosed) {
                    claw.turnToAngle(closeClawAngle);
                } else {
                    claw.turnToAngle(openClawAngle);
                }
                return true;
            }
        }
        public Action tightenClaw() {
            return new TightClaw();
        }
    }

}

