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

@TeleOp(name="encoders", group="Robot")
@Config
public class encoders extends LinearOpMode {

    Hardware robot = new Hardware();
    YawPitchRollAngles robotOrientation;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addLine("Finished INIT");
        telemetry.update();
        robot.rR.resetEncoder();
        robot.rL.resetEncoder();
        robot.fR.resetEncoder();
        robot.fL.resetEncoder();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        waitForStart();
        while (opModeIsActive()) {
            robotOrientation = robot.imu.getRobotYawPitchRollAngles();
            if (DEBUG) {
                telemetry.addData("Left Front", robot.fL.getCurrentPosition());
                telemetry.addData("Right Front", robot.fR.getCurrentPosition());
                telemetry.addData("Left Back", robot.rL.getCurrentPosition());
                telemetry.addData("right back", robot.rR.getCurrentPosition());


                dashboardTelemetry.addData("Left Front", robot.fL.getCurrentPosition());
                dashboardTelemetry.addData("Right Front", robot.fR.getCurrentPosition());
                dashboardTelemetry.addData("Left Back", robot.rL.getCurrentPosition());
                dashboardTelemetry.addData("right back", robot.rR.getCurrentPosition());

                // telemetry.addData("Arm angle", armAngles.get(robot.armRotate.getCurrentPosition()));
                //telemetry.addData("Arm power", Hardware.calculateArmPower(armAngles.get(robot.armRotate.getCurrentPosition()), clawWeightCoefficient) + controller1.getRightY());
            }

            telemetry.update();
            dashboardTelemetry.update();
        }
    }

}