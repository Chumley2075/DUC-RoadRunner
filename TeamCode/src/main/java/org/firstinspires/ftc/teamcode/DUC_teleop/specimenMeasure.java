package org.firstinspires.ftc.teamcode.DUC_teleop;

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
import org.firstinspires.ftc.teamcode.DUC_auto.openCVTest;
import org.firstinspires.ftc.teamcode.lib.Hardware;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "specimenMeasure", group="Robot")
@Config
public class specimenMeasure extends LinearOpMode {

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

    double cX = 0;
    double cY = 0;
    int width = 0;
    int height = 0;

    private OpenCvCamera controlHubCam;
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 960;
//    private static final int TARGET_X = 1;
//    private static final int TARGET_Y = 1;

    public static final double objectWidthInRealWorldUnits = 3.75;  // Actual width (in inches?)
    public static final double focalLength = 1430;  // Replace with the focal length of the camera in pixels

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

        initOpenCV();
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

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

                telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
                telemetry.addData("Specimen width in pixels", width);
                telemetry.addData("Specimen height in pixels", height);
                telemetry.addData("Distance in Inch", (getDistance(width)));

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
            if (gamepad1.right_bumper) {robot.specimenArm.resetEncoder();}
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

    private void initOpenCV() {
        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new SpecimenDetectionPipeline());

        controlHubCam.openCameraDevice();
        // TODO: Check if this orientation is correct
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);

    }

    class SpecimenDetectionPipeline extends OpenCvPipeline {
        // TODO: Adjust param for specimen contour
        // Values should be in pixel scale
//        private static final int SPECIMEN_WIDTH = 1;
//        private static final int SPECIMEN_HEIGHT = 1;
//        private static final int SPECIMEN_AREA = SPECIMEN_WIDTH * SPECIMEN_HEIGHT;
//        private static final double MARGIN = 0.1;
//        private static final double LOWER_LIMIT = SPECIMEN_AREA * (1 - MARGIN);
//        private static final double UPPER_LIMIT = SPECIMEN_AREA * (1 + MARGIN);

        @Override
        public Mat processFrame(Mat input) {
            Mat blueMask = preprocessFrame(input);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint specimenContour = findLargestContour(contours);

            if(specimenContour != null) {
                Imgproc.drawContours(input, contours, contours.indexOf(specimenContour), new Scalar(255, 0, 0), 2);

                Rect boundingRect = Imgproc.boundingRect(specimenContour);
                width = boundingRect.width;
                height = boundingRect.height;

                String widthLabel = "Width: " + width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                String heightLabel = "Width: " + height + " pixels";
                Imgproc.putText(input, heightLabel, new Point(cX + 20, cY + 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                Moments moments = Imgproc.moments(specimenContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            // TODO: Check which side of field; this one is blue
            Scalar lowerColor = new Scalar(180, 100, 100);
            Scalar upperColor = new Scalar(275, 255, 255);

            Mat colorMask = new Mat();
            Core.inRange(hsvFrame, lowerColor, upperColor, colorMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(colorMask, colorMask, Imgproc.MORPH_CLOSE, kernel);

            return colorMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }

    }

    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;

        return distance;
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
