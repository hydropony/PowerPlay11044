package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.Virtual4bar;

public class Robot22 extends Robot {
    private SampleMecanumDrive drive;
    private Lift lift;
    private Intake intake;
    private Virtual4bar virtual4bar;

    public Robot22(LinearOpMode opMode) {
        super(opMode);
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(opMode);
        intake = new Intake(opMode);
        virtual4bar = new Virtual4bar(opMode);

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
        lift.teleop();
        intake.teleop();
        virtual4bar.teleop();
    }
}
