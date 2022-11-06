package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.Virtual4bar;

public class Robot22 extends Robot {
    private SampleMecanumDrive drive;
    public Lift lift;
    public Intake intake;
    public double k = 0.5;
    public double heading, x, y;

    public Robot22(LinearOpMode opMode) {
        super(opMode);
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(opMode);
        intake = new Intake(opMode);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Init ready!", "");
        telemetry.update();
    }

    public void control() {
        heading = drive.getRawExternalHeading();
        x = -gamepad1.left_stick_y;
        y = -gamepad1.left_stick_x;

        if(gamepad1.dpad_up){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0.5,
                            -0,
                            -0
                    )
            );
        }
        else if(gamepad1.dpad_down){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -0.5,
                            -0,
                            -0
                    )
            );
        }
        else if(gamepad1.dpad_left){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            0.5,
                            -0
                    )
            );
        }
        else if(gamepad1.dpad_right){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            -0.5,
                            -0
                    )
            );
        }
        else if(gamepad1.left_bumper){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -0,
                            -0,
                            0.5
                    )
            );
        }
        else if(gamepad1.right_bumper){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -0,
                            -0,
                            -0.5
                    )
            );
        }
      /* else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -(gamepad1.right_trigger - gamepad1.left_trigger)
                    )
            );
        }*/
        else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            x * Math.cos(heading) - y * Math.sin(heading),
                            x * Math.sin(heading) + y * Math.cos(heading),
                            -(gamepad1.right_trigger - gamepad1.left_trigger)
                    )
            );
        }

       telemetry.addData("heading", heading);
        lift.update();
        drive.update();
        intake.teleop();
    }
}
