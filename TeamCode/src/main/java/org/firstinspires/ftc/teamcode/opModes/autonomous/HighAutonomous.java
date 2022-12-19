package org.firstinspires.ftc.teamcode.opModes.autonomous;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

public class HighAutonomous extends LinearOpMode {
    Robot22 R;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    public static double run = 50;
    public static double runHigh = 10;
    public static double runHighBack = -6;
    public static double ParkSecond= -24;
    public static double leftT = 24;
    public static double rightT = 23;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT = 1;
    int MIDDLE = 12;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        R = new Robot22(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ServoAuto servoClass = new ServoAuto(this);
        Trajectory forward = R.drive.trajectoryBuilder(new Pose2d(0,0))
                .forward(run)
                .build();
        Trajectory runToHigh = R.drive.trajectoryBuilder(forward.end().plus(new Pose2d(0,0,Math.toRadians(-43))))
                .forward(runHigh)
                .build();
        Trajectory runFromHigh = R.drive.trajectoryBuilder(forward.end().plus(new Pose2d(0,0,Math.toRadians(-43))))
                .forward(runHighBack)
                .build();
        Trajectory runSecondPosition = R.drive.trajectoryBuilder(forward.end().plus(new Pose2d(0,0,Math.toRadians(0))))
                .forward(ParkSecond)
                .build();
        Trajectory forwardLEFTT = R.drive.trajectoryBuilder(forward.end().plus(new Pose2d(0,0,Math.toRadians(90))))
                .forward(leftT)
                .build();
        Trajectory forwardRIGHTT = R.drive.trajectoryBuilder(forward.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .forward(rightT)
                .build();
       /* Trajectory forwardRIGHT = R.drive.trajectoryBuilder(forward.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .forward(right)
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
        R.drive.followTrajectory(forward);
        R.drive.turn(Math.toRadians(-43));
        R.lift.Pos0();
        R.drive.followTrajectory(runToHigh);
        sleep(100);
        servoClass.run();
        R.drive.followTrajectory(runFromHigh);
        R.lift.Pos1();
        R.drive.turn(Math.toRadians(43));
        R.drive.followTrajectory(runSecondPosition);
        /*while(R.drive.isBusy() && !isStopRequested()){
            R.lift.updateavto();
            //R.drive.followTrajectoryAsync(forward);
        }*/
        if (tagOfInterest.id == LEFT) {
            R.drive.update();
            R.drive.turn(Math.toRadians(90));
            R.drive.followTrajectory(forwardLEFTT);
        }
        else if (tagOfInterest.id == RIGHT) {
            R.drive.update();
            R.drive.turn(Math.toRadians(-90));
            R.drive.followTrajectory(forwardRIGHTT);
        }




    }
    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}