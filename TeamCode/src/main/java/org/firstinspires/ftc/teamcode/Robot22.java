package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot22 extends Robot {
    private SampleMecanumDrive drive;

    public Robot22(LinearOpMode opMode) {
        super(opMode);
        drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addData("Init ready!", "");
        telemetry.update();
    }

    public void control() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -(gamepad1.right_trigger - gamepad1.left_trigger)
                )
        );

        drive.update();
    }
}
