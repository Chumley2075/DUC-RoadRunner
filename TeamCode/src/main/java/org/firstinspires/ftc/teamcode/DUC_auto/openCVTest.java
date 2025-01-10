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

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "openCVTest", group = "Autonomous")
public class openCVTest extends LinearOpMode {

    int armTickPosition = 250;
    public static int hookPosition = 1800;
    public static int myHeading = 180;

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 960;
    private static final int TARGET_X = 1;
    private static final int TARGET_Y = 1;

    public static final double objectWidthInRealWorldUnits = 3.75;  // Actual width (in inches?)
    public static final double focalLength = 1430;  // Replace with the focal length of the camera in pixels

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(37, 64, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Spool spool = new Spool(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        Pose2d highRung = new Pose2d(-10, 40, Math.toRadians(270));
        Pose2d highRungLatch = new Pose2d(-10, 35, Math.toRadians(270));
        Vector2d observation = new Vector2d(60, 64);
        Vector2d ascentArea = new Vector2d(-10, 15);

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        TrajectoryActionBuilder strafeToHighRung = drive.actionBuilder(initialPose)
                .splineToLinearHeading(highRung, Math.toRadians(270));

        TrajectoryActionBuilder highRungLatch1 = drive.actionBuilder(highRung)
                .splineToLinearHeading(highRungLatch, Math.toRadians(270));

        TrajectoryActionBuilder highRungLatch2 = drive.actionBuilder(highRungLatch)
                .waitSeconds(1)
                .splineToLinearHeading(highRung, Math.toRadians(270));

        TrajectoryActionBuilder parkObservation = drive.actionBuilder(highRung)
                .strafeTo(observation);

        TrajectoryActionBuilder parkTier1Ascent = drive.actionBuilder(highRung)
                .strafeTo(new Vector2d(37, 40))
                .strafeTo(new Vector2d(37, 15))
                .setReversed(true)
                .strafeToLinearHeading(ascentArea, Math.toRadians(180));

        Actions.runBlocking(claw.closeClaw());

        waitForStart();
        if (isStopRequested()) return;

//        while (opModeIsActive()) {
//            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
//            telemetry.addData("Distance in Inch", (getDistance(width)));
//            telemetry.update();
//
//            // The OpenCV pipeline automatically processes frames and handles detection
//        }

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        strafeToHighRung.build(),
                                        arm.highRung()
                                ),
                                highRungLatch1.build(),
                                arm.highRung2(),
                                highRungLatch2.build(),
                                claw.openClaw(),
                                new SleepAction(1),
                                parkTier1Ascent.build(),
                                arm.tierOneAscent(),
                                claw.closeClaw()
                        ),
                        arm.keepPosition()
                )

        );

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
                arm.set(.75);
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

        public class TierOneAscent implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armTickPosition = 1250;
                if (arm.atTargetPosition()) {
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action tierOneAscent() {
            return new TierOneAscent();
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

    class SpecimenDetectionPipeline extends OpenCvPipeline {
        // TODO: Adjust param for specimen contour
        // Values should be in pixel scale
        private static final int SPECIMEN_WIDTH = 1;
        private static final int SPECIMEN_HEIGHT = 1;
        private static final int SPECIMEN_AREA = SPECIMEN_WIDTH * SPECIMEN_HEIGHT;
        private static final double MARGIN = 0.1;
        private static final double LOWER_LIMIT = SPECIMEN_AREA * (1 - MARGIN);
        private static final double UPPER_LIMIT = SPECIMEN_AREA * (1 + MARGIN);

        @Override
        public Mat processFrame(Mat input) {
            // TODO: WRITE FOR BLUE
            Mat blueMask = preprocessFrame(input);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint specimenContour = null;
            for(int i = 0; i < contours.size(); i++) {
                double cont_area = Imgproc.contourArea(contours.get(i));
                if((cont_area > LOWER_LIMIT) && (cont_area < UPPER_LIMIT)) {
                    specimenContour = contours.get(i);

                }

            }

            if(specimenContour != null) {
                Imgproc.drawContours(input, contours, contours.indexOf(specimenContour), new Scalar(255, 0, 0), 2);

//                width = calculateWidth(specimenContour);
//
//                // Display the width next to the label
//                String widthLabel = "Width: " + (int) width + " pixels";
//                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
//                //Display the Distance

                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                // Calculate the centroid
                Moments moments = Imgproc.moments(specimenContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
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
            Scalar lowerBlue = new Scalar(180, 100, 100);
            Scalar upperBlue = new Scalar(275, 255, 255);


            Mat blueMask = new Mat();
            Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

//        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
//            double maxArea = 0;
//            MatOfPoint largestContour = null;
//
//            for (MatOfPoint contour : contours) {
//                double area = Imgproc.contourArea(contour);
//                if (area > maxArea) {
//                    maxArea = area;
//                    largestContour = contour;
//                }
//            }
//
//            return largestContour;
//        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;

        return distance;
    }

}

