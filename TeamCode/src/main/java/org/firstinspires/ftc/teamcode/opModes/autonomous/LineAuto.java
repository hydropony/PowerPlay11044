package org.firstinspires.ftc.teamcode.opModes.autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OneOperatorRobot2022;
import org.firstinspires.ftc.teamcode.Robot22;
import org.firstinspires.ftc.teamcode.Vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.ServoAuto;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous

public class LineAuto extends LinearOpMode {
   OneOperatorRobot2022 R;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    public static double run = 49.5;
    public static double runHigh = 9.8;
    public static double runHighBack = -2.2;
    public static double SteakRun = 29.1;
    public static double FromSteakRun = -52;
    public static double MiddleRun = 2.3;
    public static double fromMiddleRun = 0.1 ;
    public static double FromCentralHighRun = -2;
    public static double SecondPosition = 0.5;
    public static double FirstPosition = 2;
    public static double ThirdPosition = 0.001;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 12;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        R = new OneOperatorRobot2022(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ServoAuto servoClass = new ServoAuto(this);
        Trajectory forward = R.drive.trajectoryBuilder(new Pose2d(0,0))
                .forward(run)
                .build();
        /*Trajectory runSecondPosition3 = R.drive.trajectoryBuilder(new Pose2d(12,0))
                .forward(-ParkSecond)
                .build();*/
        Trajectory runToHigh = R.drive.trajectoryBuilder(forward.end().plus(new Pose2d(0,0,Math.toRadians(-48))))
                .forward(runHigh)
                .build();
        Trajectory runFromHigh = R.drive.trajectoryBuilder(runToHigh.end().plus(new Pose2d(0,0,Math.toRadians(0))))
                .forward(runHighBack)
                .build();
        Trajectory RunToSteak = R.drive.trajectoryBuilder(forward.end().plus(new Pose2d(0,0,Math.toRadians(90))))
                .forward(SteakRun)
                .build();
       /* Trajectory forwardRIGHT = R.drive.trajectoryBuilder(forward.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .forward(right)
                .build();*/
        Trajectory RunFromSteak = R.drive.trajectoryBuilder(forward.end().plus(new Pose2d(-14,0,Math.toRadians(90))))
                .forward(FromSteakRun)
                .build();
        Trajectory RunToMiddle = R.drive.trajectoryBuilder(RunFromSteak.end().plus(new Pose2d(0,9,Math.toRadians(15))))
                .forward(MiddleRun)
                .build();
        Trajectory FromCentrRun = R.drive.trajectoryBuilder(RunFromSteak.end().plus(new Pose2d(0,0,Math.toRadians(0))))
                .forward(FromCentralHighRun)
                .build();
        Trajectory RunThirdPosition1 = R.drive.trajectoryBuilder(RunFromSteak.end().plus(new Pose2d(15,0,Math.toRadians(0))))
                .forward(ThirdPosition)
                .build();
        Trajectory RunThirdPosition2 = R.drive.trajectoryBuilder(RunToSteak.end().plus(new Pose2d(0,0,Math.toRadians(0))))
                .forward(ThirdPosition)
                .build();
        Trajectory RunSecondPosition = R.drive.trajectoryBuilder(RunToSteak.end().plus(new Pose2d(0,0,Math.toRadians(0))))
                .forward(SecondPosition)
                .build();
        Trajectory RunFirstPosition = R.drive.trajectoryBuilder(RunToSteak.end().plus(new Pose2d(0,0,Math.toRadians(0))))
                .forward(FirstPosition)
                .build();
        /*Trajectory RunSecondPosition = R.drive.trajectoryBuilder(forward.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .forward(SecondPosition)
                .build();
        Trajectory RunFirstPosition = R.drive.trajectoryBuilder(forward.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .forward(FirstPosition)
                .build();
        Trajectory RunThirdPosition = R.drive.trajectoryBuilder(forward.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .forward(ThirdPosition)
                .build();*/

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();

        }


        //R.drive.followTrajectory(forward);
        // R.lift.state = Lift.State.MIDDLE;
        R.lift.AutoMotorSetP(0.9);
        R.drive.followTrajectory(forward);
        //sleep(700);
        /*R.delay(700);*/
        R.drive.turn(Math.toRadians(-46));
        R.drive.followTrajectory(runToHigh);
        //R.lift.Pos0();
        servoClass.run();
        R.drive.followTrajectory(runFromHigh);
        R.lift.PositionTeleop1(230, 1);
        //R.lift.Pos2();
        R.drive.turn(Math.toRadians(137));
        R.drive.followTrajectory(RunToSteak);
        //R.lift.Pos3();
        R.lift.AutoMotorSetP(-0.05);
        servoClass.just_run();
        R.lift.AutoMotorSetP(0.9);
       // sleep(600);
        R.delay(600);
        R.drive.followTrajectory(RunFromSteak);
       // R.drive.turn(Math.toRadians(40));

        R.drive.followTrajectory(RunToMiddle);

        //R.lift.Pos4();
        //R.drive.followTrajectory(RunFromMiddle);
        //R.lift.Pos4();
        //sleep(700);
        R.delay(700);
        servoClass.run();
        R.drive.followTrajectory(FromCentrRun);
        R.drive.followTrajectory(RunThirdPosition1);
        R.lift.PositionTeleop1(270, 1);
        R.drive.followTrajectory(RunThirdPosition2);
       // R.drive.followTrajectory(RunFirstPosition);
        //R.lift.PositionTeleop1(270, 1);
       // R.drive.followTrajectory(RunSecondPosition);
        /*R.drive.followTrajectory(RunToMiddle);

        R.drive.turn(Math.toRadians(-50));*/
        //R.drive.followTrajectory(RunSecondPosition);

        if (tagOfInterest.id == LEFT) {
           // R.drive.followTrajectory(RunFirstPosition);
        }
        else if (tagOfInterest.id == RIGHT) {

        }




    }
    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}