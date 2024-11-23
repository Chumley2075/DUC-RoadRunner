package org.firstinspires.ftc.teamcode.DUC_teleop;

//reserve for constants import
import static org.firstinspires.ftc.teamcode.Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.Constants.clawWeightCoefficient;
import static org.firstinspires.ftc.teamcode.lib.Hardware.armAngles;
import static org.firstinspires.ftc.teamcode.lib.Hardware.spoolLowerBounds;
import static org.firstinspires.ftc.teamcode.lib.Hardware.spoolUpperBounds;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.lib.Hardware;
@TeleOp(name="Teleop1", group="Robot")
@Config
public class Drive extends LinearOpMode {

    Hardware robot = new Hardware();
    YawPitchRollAngles robotOrientation;
    public static int armTickUpper = 2000;
    public static int armTickLower = 0;
    public static int armTickPosition = 200;
    boolean encoderMethod = false;
    public static double positionCoefficient=.01;
    double precisionCoefficient = 1;
    double armPower = 0;
//ButtonReader r1 = new ButtonReader(controller1, GamepadKeys.Button.RIGHT_BUMPER);

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
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        GamepadEx controller1 = new GamepadEx(gamepad1);
       //
        ButtonReader reader = new ButtonReader(controller1, GamepadKeys.Button.RIGHT_BUMPER);
        waitForStart();
        while (opModeIsActive()) {
            robotOrientation = robot.imu.getRobotYawPitchRollAngles();
            if (DEBUG) {
             //   telemetry.addData("Yaw", robotOrientation.getYaw());
               // telemetry.addData("Pitch", robotOrientation.getPitch());
               // telemetry.addData("Roll", robotOrientation.getRoll());
              //  telemetry.addData("Claw power", robot.claw.get());
                telemetry.addData("Spool encoders: ", robot.spool.getCurrentPosition());
                telemetry.addData("Arm tick Pos: ", robot.armRotate.getCurrentPosition());
                telemetry.addData("Arm coefficient: ", positionCoefficient);
                telemetry.addData("BooleanMethod: ",encoderMethod);
                telemetry.addData("TARGET: ", armTickPosition);
                telemetry.addData("Arm power target", Hardware.calculateArmPower(armAngles.get(robot.armRotate.getCurrentPosition()), clawWeightCoefficient));
                telemetry.addData("Arm power", robot.armRotate.get());



                dashboardTelemetry.addData("Spool encoders: ", robot.spool.getCurrentPosition());
                dashboardTelemetry.addData("Arm tick Pos: ", robot.armRotate.getCurrentPosition());
                dashboardTelemetry.addData("Arm coefficient: ", positionCoefficient);
                dashboardTelemetry.addData("BooleanMethod: ",encoderMethod);
                dashboardTelemetry.addData("TARGET: ", armTickPosition);
                dashboardTelemetry.addData("Arm power target", Hardware.calculateArmPower(armAngles.get(robot.armRotate.getCurrentPosition()), clawWeightCoefficient));
                dashboardTelemetry.addData("Arm power", robot.armRotate.get());

                // telemetry.addData("Arm angle", armAngles.get(robot.armRotate.getCurrentPosition()));
                //telemetry.addData("Arm power", Hardware.calculateArmPower(armAngles.get(robot.armRotate.getCurrentPosition()), clawWeightCoefficient) + controller1.getRightY());
            }
            if (gamepad1.right_trigger > 0) {
                precisionCoefficient = 0.5;
            } else {
                precisionCoefficient = 1;
            }

            mecanum.driveRobotCentric(
                    controller1.getLeftX() * precisionCoefficient,
                    controller1.getLeftY() * precisionCoefficient,
                    controller1.getRightX() * precisionCoefficient,
                    true
            );

            if (gamepad1.x) {
                robot.claw.set(1);
            } else if (gamepad1.b) {
                robot.claw.set(-1);
            }else{
                robot.claw.stopMotor();
            }

            if (gamepad1.dpad_up && (robot.spool.getCurrentPosition() < spoolUpperBounds && !gamepad1.right_bumper)) {
                robot.spool.set(1);
            } else if (gamepad1.dpad_down && (robot.spool.getCurrentPosition() > spoolLowerBounds && !gamepad1.right_bumper)) {
                robot.spool.set(-1);
            } else {
                robot.spool.set(0);
            }


            //START OF RYAN CODE
            if(!encoderMethod){
                robot.armRotate.setRunMode(Motor.RunMode.RawPower);
                robot.armRotate.set(-gamepad1.right_stick_y);
               // armPower = Hardware.calculateArmPower(armAngles.get(robot.armRotate.getCurrentPosition()), clawWeightCoefficient);
                //robot.armRotate.set(armPower + Math.cbrt(controller1.getRightY()));

            }else{
                if(gamepad1.a){
                  armTickPosition =armTickUpper;
                }else if(gamepad1.y){
                    armTickPosition = armTickLower;
                }
            }
            reader.readValue();
            if(reader.wasJustReleased()){
                encoderMethod = !encoderMethod;
            }
            if(encoderMethod){
                keepPosition();
            }
            // END OF RYAN CODE



            /*if (gamepad1.a) {
                robot.armRotate.setTargetPosition(armPosition);
                robot.armRotate.set(0);
                robot.armRotate.setPositionTolerance(13.6); //error
                while (!robot.armRotate.atTargetPosition()) {
                    robot.armRotate.set(0.5);
                }
                robot.armRotate.stopMotor(); // stop the motor
            }*/

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
    public void keepPosition(){
        robot.armRotate.setRunMode(Motor.RunMode.PositionControl);
        robot.armRotate.setPositionCoefficient(positionCoefficient);
        robot.armRotate.setTargetPosition(armTickPosition);
        robot.armRotate.set(.5);
        robot.armRotate.setPositionTolerance(10);
    }
}