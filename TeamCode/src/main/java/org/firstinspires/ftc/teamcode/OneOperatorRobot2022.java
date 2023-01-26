package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.LiftOriginal;
import org.firstinspires.ftc.teamcode.modules.Virtual4bar;

public class OneOperatorRobot2022 extends Robot {
    //public DigitalChannel digitalTouch;
    public SampleMecanumDrive drive;
    public LiftOriginal lift;
    public Intake intake;
    public double k = 0.5;
    CRServo servo3;
    CRServo servo2;
    public double heading, x, y;

    public OneOperatorRobot2022(LinearOpMode opMode) {
        super(opMode);
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new LiftOriginal(opMode);
        intake = new Intake(opMode);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);*/


        telemetry.addData("Init ready!", "");
        telemetry.update();
    }



    public void control() {
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servo3 = hardwareMap.get(CRServo.class, "servo3");
        heading = drive.getRawExternalHeading();
        x = -gamepad1.left_stick_y;
        y = -gamepad1.left_stick_x;

        /*if (gamepad1.y){
            lift.DoZeroLift_DigitalSensor();
        }

        /*if(gamepad2.right_stick_y > 0 && !digitalTouch.getState()) {
            servo3.setPower(1);
            servo2.setPower(-1);
        }
        else if(gamepad2.right_stick_y < 0) {
            servo3.setPower(-1);
            servo2.setPower(1);
        }
        /*if(digitalTouch.getState()){
            if(gamepad2.right_stick_y > 0) {
                servo3.setPower(0);
                servo2.setPower(0);
            }
            else if(gamepad2.right_stick_y < 0) {
                servo3.setPower(-1);
                servo2.setPower(1);
            }
            else{
                servo3.setPower(0);
                servo2.setPower(0);
            }
        }
        else {
            servo3.setPower(0);
            servo2.setPower(0);
        }*/
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
        else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -(gamepad1.right_trigger - gamepad1.left_trigger)
                    )
            );
        }
      /*  else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            x * Math.cos(heading) - y * Math.sin(heading),
                            x * Math.sin(heading) + y * Math.cos(heading),
                            -(gamepad1.right_trigger - gamepad1.left_trigger)
                    )
            );
        }*/

        telemetry.addData("heading", heading);
        lift.update();
        drive.update();
        intake.teleop();
    }
}
