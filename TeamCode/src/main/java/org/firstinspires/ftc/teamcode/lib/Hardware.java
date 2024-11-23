package org.firstinspires.ftc.teamcode.lib;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//hardware class because im learning from my mistakes last year! good job me
//thank you pufferfishking for the template - stolen from 7588 (sorta kinda)

@Config
public class Hardware {

    //wheel motors
    public Motor fL, fR, rL, rR, armRotate, spool;
    //imu and YPRA
    public YawPitchRollAngles robotOrientation;
    public IMU imu;
    public CRServo claw;
    private HardwareMap hwMap;
    public static double spoolLowerBounds = 0;
    public static double spoolUpperBounds = 1000;
    public static InterpLUT armAngles;

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        //HARDWARE MAP

        //ftclib motors
        fL = new Motor(hwMap, "leftFront");
        fR = new Motor(hwMap, "rightFront");
        rL = new Motor(hwMap, "leftBack");
        rR = new Motor(hwMap, "rightBack");
        armRotate = new Motor(hwMap, "arm");
        spool = new Motor(hwMap, "spool");
        imu = hwMap.get(IMU.class, "imu");
        //claw = new CRServo(hwMap, "claw");
        claw = new CRServo(hwMap, "claw");
        //INITIALIZATION

        //imu
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        robotOrientation = imu.getRobotYawPitchRollAngles();

        //drive
        fL.setInverted(true);
        rL.setInverted(true);
        fR.setInverted(true);
        rR.setInverted(true);
        armRotate.setInverted(true);
        //claw.turnToAngle(0);
        //claw.setInverted(true);
        /*fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); not needed for ftclib motors me thinks*/
        armRotate.setRunMode(Motor.RunMode.PositionControl);
        fL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //init claw if it was NORMAL.........
        //claw.turnToAngle(160);

        armAngles = new InterpLUT();
        armAngles.add(-10000, -61);
        armAngles.add(0, -60);
        armAngles.add(1000, 0);
        armAngles.add(2700, 90);
        armAngles.add(4370, 180);
        armAngles.add(5290, 225);
        armAngles.add(10000, 226);
        armAngles.createLUT();

        spool.set(0);
        armRotate.set(0);
        fL.set(0);
        fR.set(0);
        rR.set(0);
        rL.set(0);
    }

    public static double calculateArmPower(double armAngle, double kCos) {
        return kCos * Math.cos(Math.toRadians(armAngle));
    }

}
