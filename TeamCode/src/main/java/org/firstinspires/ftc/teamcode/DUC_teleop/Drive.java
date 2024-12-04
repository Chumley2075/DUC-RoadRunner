package org.firstinspires.ftc.teamcode.DUC_teleop;

//reserve for constants import
import static org.firstinspires.ftc.teamcode.Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.Constants.clawWeightCoefficient;
import static org.firstinspires.ftc.teamcode.lib.Hardware.armAngles;
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
    public static int armTickUpper = 2000;
    public static int armTickLower = 0;
    public static int armTickPosition = 0;
    public static double positionCoefficient=.01;
    double precisionCoefficient = 1;
    double rightStickYValue;
    double armPower = 0;
    boolean intakeMacro = false;
    double intervalMS = 100;
    double heading = 0;
    ElapsedTime timerArmRotate = new ElapsedTime();
    ElapsedTime macro1 = new ElapsedTime();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

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

        heading = robot.robotOrientation.getYaw(AngleUnit.DEGREES);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        GamepadEx controller1 = new GamepadEx(gamepad1);
        GamepadEx controller2 = new GamepadEx(gamepad2);
        //Color sensor stuff
        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW)
                .setSwatches()
                .build();
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(640, 360))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
        PredominantColorProcessor.Result result = colorSensor.getAnalysis();


        waitForStart();
        timerArmRotate.reset();
        while (opModeIsActive()) {
            robotOrientation = robot.imu.getRobotYawPitchRollAngles();
            if (DEBUG) {

                telemetry.addData("Spool encoders: ", robot.spool.getCurrentPosition());
                telemetry.addData("Arm tick Pos: ", robot.armRotate.getCurrentPosition());
                telemetry.addData("Yaw:", robot.robotOrientation.getYaw(AngleUnit.DEGREES));
                telemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
                telemetry.addData("TARGET: ", armTickPosition);
                telemetry.addData("Right stick y", Math.cbrt(-gamepad2.right_stick_y));
                telemetry.addData("Gamepad2 x", gamepad2.x);



                dashboardTelemetry.addData("Spool encoders: ", robot.spool.getCurrentPosition());
                dashboardTelemetry.addData("Arm tick Pos: ", robot.armRotate.getCurrentPosition());
                dashboardTelemetry.addData("TARGET Ticks: ", armTickPosition);
                dashboardTelemetry.addData("Arm power target", Hardware.calculateArmPower(armAngles.get(robot.armRotate.getCurrentPosition()), clawWeightCoefficient));
                dashboardTelemetry.addData("Arm power", robot.armRotate.get());
                dashboardTelemetry.addData("Best Match Color Swatch:", result.closestSwatch);
                dashboardTelemetry.addLine(String.format("R %3d, G %3d, B %3d", Color.red(result.rgb), Color.green(result.rgb), Color.blue(result.rgb)));
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

            if (gamepad2.left_bumper) {
                robot.claw.set(1);
            } else if (gamepad2.right_bumper) {
                robot.claw.set(-1);
            }else{
                robot.claw.stopMotor();
            }




            if (gamepad2.dpad_up /*&& (robot.spool.getCurrentPosition() < spoolUpperBounds && !gamepad2.right_bumper)*/) {
                robot.spool.set(1);
            } else if (gamepad2.dpad_down /*&& (robot.spool.getCurrentPosition() > spoolLowerBounds && !gamepad2.right_bumper)*/) {
                robot.spool.set(-1);
            } else {
                robot.spool.set(0);
            }

            rightStickYValue = Math.cbrt(-gamepad2.right_stick_y);
            if(rightStickYValue>0 || rightStickYValue<0){
                if (timerArmRotate.milliseconds() >= intervalMS) {
                    armTickPosition += (int) rightStickYValue * (100 );
                    timerArmRotate.reset();
                }
            } else if (gamepad2.a) {
                armTickPosition= 50;
            } else if (gamepad2.y) {
                armTickPosition = 2500;
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
    }
}