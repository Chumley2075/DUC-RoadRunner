package org.firstinspires.ftc.teamcode.DUC_teleop;

//reserve for constants import
import static org.firstinspires.ftc.teamcode.Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.Constants.clawWeightCoefficient;
import static org.firstinspires.ftc.teamcode.lib.Hardware.armAngles;
import static org.firstinspires.ftc.teamcode.lib.Hardware.closeClawAngle;
import static org.firstinspires.ftc.teamcode.lib.Hardware.openClawAngle;
import static org.firstinspires.ftc.teamcode.lib.Hardware.specimenLowerBounds;
import static org.firstinspires.ftc.teamcode.lib.Hardware.specimenUpperBounds;
import static org.firstinspires.ftc.teamcode.lib.Hardware.spoolLowerBounds;
import static org.firstinspires.ftc.teamcode.lib.Hardware.spoolUpperBounds;


import android.graphics.Color;
import android.os.SystemClock;
import android.util.Size;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

@TeleOp(name="Teleop1", group="Robot")
@Config
public class Drive extends LinearOpMode {

    Hardware robot = new Hardware();
    YawPitchRollAngles robotOrientation;
    public static int armTickPosition = 0;
    public static double positionCoefficient=.01;
    double precisionCoefficient = 1;
    double precisionCoefficient2 = 1;
    double rightStickYValue;
    double intervalMS = 100;
    double heading = 0;
    boolean clawOpen = true;
    boolean specimenClawOpen = false;
    ElapsedTime timerArmRotate = new ElapsedTime();
    ElapsedTime timerArmSpecimen = new ElapsedTime();
    public int specimenTargetPosition = 0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.imu.resetYaw();

        telemetry.addLine("Finished INIT");
        telemetry.update();
        MecanumDrive mecanum = new MecanumDrive(robot.fL, robot.fR, robot.rL, robot.rR);
        robot.spool.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        robot.armRotate.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        robot.fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        robot.fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        robot.rR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        robot.fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        robot.spool.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        robot.spool.resetEncoder();
        robot.armRotate.resetEncoder();
        robot.specimenArm.resetEncoder();


        heading = robot.robotOrientation.getYaw(AngleUnit.DEGREES);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        GamepadEx controller1 = new GamepadEx(gamepad1);
        GamepadEx controller2 = new GamepadEx(gamepad2);
        ButtonReader clawReader = new ButtonReader(controller2, GamepadKeys.Button.X);
        ButtonReader specimenReader = new ButtonReader(controller1, GamepadKeys.Button.X);
        //Color sensor stuff



        waitForStart();
        timerArmRotate.reset();
        while (opModeIsActive()) {
            robotOrientation = robot.imu.getRobotYawPitchRollAngles();
            if (DEBUG) {

                telemetry.addData("Spool encoders: ", robot.spool.getCurrentPosition());
                telemetry.addData("Specimen encoders: ", robot.specimenArm.getCurrentPosition());
                telemetry.addData("Arm tick Pos: ", robot.armRotate.getCurrentPosition());
                telemetry.addData("Yaw:", robot.robotOrientation.getYaw(AngleUnit.DEGREES));
            //    telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
                telemetry.addData("TARGET: ", armTickPosition);
                telemetry.addData("Right stick y", Math.cbrt(-gamepad2.right_stick_y));
                telemetry.addData("claw open", clawOpen);



                dashboardTelemetry.addData("Spool encoders: ", robot.spool.getCurrentPosition());
                dashboardTelemetry.addData("Arm tick Pos: ", robot.armRotate.getCurrentPosition());
                dashboardTelemetry.addData("TARGET Ticks: ", armTickPosition);
              //  dashboardTelemetry.addData("Arm power target", Hardware.calculateArmPower(armAngles.get(robot.armRotate.getCurrentPosition()), clawWeightCoefficient));
                dashboardTelemetry.addData("Arm power", robot.armRotate.get());
             //   dashboardTelemetry.addData("Best Match Color Swatch:", result.closestSwatch);
           //     dashboardTelemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
            }

            robot.robotOrientation = robot.imu.getRobotYawPitchRollAngles();
            if (gamepad1.left_bumper) {robot.imu.resetYaw();}
            mecanum.driveFieldCentric(
                    controller1.getLeftX() * precisionCoefficient,
                    controller1.getLeftY() * precisionCoefficient,
                    controller1.getRightX() * precisionCoefficient,
                    robot.robotOrientation.getYaw(AngleUnit.DEGREES)
            );

            if (gamepad1.right_trigger > 0) {
                precisionCoefficient = 0.25;
            } else {
                precisionCoefficient = 1;
            }
            if (gamepad2.right_trigger > 0) {
                precisionCoefficient2 = 0.5;
            } else {
                precisionCoefficient2 = 1;
            }
            clawReader.readValue();
            if (clawReader.wasJustReleased()) {
                clawOpen = !clawOpen;
            }
            if (clawOpen) {
                robot.claw.turnToAngle(openClawAngle);
            } else {
                robot.claw.turnToAngle(closeClawAngle);
            }

            specimenReader.readValue();
            if (specimenReader.wasJustReleased()) {
                specimenClawOpen = !specimenClawOpen;
            }
            if (specimenClawOpen) {
                robot.specimenClaw.turnToAngle(openClawAngle);
            } else {
                robot.specimenClaw.turnToAngle(95);
            }



            if (gamepad2.dpad_up && ((robot.spool.getCurrentPosition() < spoolUpperBounds) || gamepad2.b)) {
                if (robot.armRotate.getCurrentPosition() < 3010) {
                    robot.spool.set(1 * precisionCoefficient2);
                }
            } else if (gamepad2.dpad_down && ((robot.spool.getCurrentPosition() > spoolLowerBounds) || gamepad2.b)) {
                robot.spool.set(-1 * precisionCoefficient2);
            } else {
                robot.spool.set(0);
            }

            if(gamepad1.dpad_up || gamepad1.dpad_down){
                if (timerArmSpecimen.milliseconds() >= intervalMS) {
                    if (gamepad1.dpad_up) {
                        specimenTargetPosition += (int) 100 * precisionCoefficient;
                    } else if (gamepad1.dpad_down) {
                        specimenTargetPosition += (int) -100 * precisionCoefficient;
                    }
                    timerArmSpecimen.reset();
                }
            } else if (gamepad1.b) {
                specimenTargetPosition= 1500;
            } else if (gamepad1.a) {
                specimenTargetPosition = 0;
            } else if (gamepad1.y) {
                specimenTargetPosition = 2000;
            }

            rightStickYValue = Math.cbrt(-gamepad2.right_stick_y);
            if(rightStickYValue>0 || rightStickYValue<0){
                if (timerArmRotate.milliseconds() >= intervalMS) {
                    armTickPosition += (int) ((int) rightStickYValue * (100 * precisionCoefficient2));
                    timerArmRotate.reset();
                }
            } else if (gamepad2.a) {
                armTickPosition= 50;
            } else if (gamepad2.y) {
                armTickPosition = 2500;
            }
            if (gamepad2.left_bumper) {
                robot.armRotate.resetEncoder();
                armTickPosition = 0;

            }
            keepPosition();

            /*
            if(!intakeMacro){
                if (gamepad1.x) {
                    robot.claw.set(1);
                } else if (gamepad1.b) {
                    robot.claw.set(-1);
                }else{
                    robot.claw.stopMotor();
                }
            }else{
                beginIntake();
            }

            */

            //basically:
            /*
            calculate the extra power needed to lift up the robot with a claw by
            interpreting the arm motor's encoder position as an angle (to perform cosine operations on)
            and giving it a coefficient that is the amount of power it adds at max amount
            */
            //armPower = Hardware.calculateArmPower(armAngles.get(robot.armRotate.getCurrentPosition()), kp, clawWeightCoefficient);
            //robot.armRotate.set(armPower + Math.cbrt(controller1.getRightY() / 5));

            telemetry.update();
            dashboardTelemetry.update();
        }
    }
    public void beginIntake(){

    }
    public void keepPosition(){
        robot.armRotate.setRunMode(Motor.RunMode.PositionControl);
        robot.armRotate.setPositionCoefficient(positionCoefficient);
        robot.armRotate.setTargetPosition(armTickPosition);
        robot.armRotate.set(.75);
        robot.armRotate.setPositionTolerance(10);

        robot.specimenArm.setRunMode(Motor.RunMode.PositionControl);
        robot.specimenArm.setPositionCoefficient(positionCoefficient);
        robot.specimenArm.setTargetPosition(specimenTargetPosition);
        robot.specimenArm.set(1);
        robot.specimenArm.setPositionTolerance(10);
    }
}