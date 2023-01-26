package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.Intake;
import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.LiftOriginal;
import org.firstinspires.ftc.teamcode.modules.Virtual4bar;

public class Robot22 extends Robot {
    //public DigitalChannel digitalTouch;
    public SampleMecanumDrive drive;
    public Lift lift;
    public LiftOriginal liftOriginal;
    public Intake intake;
    public double k = 0.5;
    public double Pos1 = 300;
    CRServo servo3;
    CRServo servo2;
    public double heading, x, y, heading2;
    /*public ElapsedTime T = new ElapsedTime();
    public int nwL = 0;*/

    //private DigitalChannel digitalTouch;
    //public boolean WithoutKonDigit = false;

    public Robot22(LinearOpMode opMode) {
        super(opMode);
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(opMode);
        liftOriginal = new LiftOriginal(opMode);
        intake = new Intake(opMode);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        //digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        servo2 = hardwareMap.get(CRServo.class, "intakeservo1");
        servo3 = hardwareMap.get(CRServo.class, "intakeservo2");

        telemetry.addData("Init ready!", "");
        telemetry.update();
    }

    /*public int Znakint(double i){
        if (i > 0.7) return 1;
        if (i < -0.7) return -1;
        return 0;
    }*/

    public void control() {
        heading = drive.getRawExternalHeading();
        x = -gamepad1.left_stick_y;
        y = -gamepad1.left_stick_x;

        /*if (gamepad2.y){
            lift.DoZeroLift_DigitalSensor();
        }*/
        //int nnr = Znakint(gamepad1.left_stick_y);

        if (gamepad1.dpad_up) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0.5,
                            0,
                            0
                    )
            );
        } else if (gamepad1.dpad_down) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -0.5,
                            -0,
                            -0
                    )
            );
        } else if (gamepad1.dpad_left) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            0.5,
                            -0
                    )
            );
        } else if (gamepad1.dpad_right) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            -0.5,
                            -0
                    )
            );
        } else if (gamepad1.left_bumper) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -0,
                            -0,
                            0.5
                    )
            );
        } else if (gamepad1.right_bumper) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -0,
                            -0,
                            -0.5
                    )
            );
        } /*else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            x * Math.cos(heading) - y * Math.sin(heading),
                            x * Math.sin(heading) + y * Math.cos(heading),
                            -(gamepad1.right_trigger - gamepad1.left_trigger)
                    )
            );
        }*/


        else{
            drive.setWeightedDrivePower(
                    new Pose2d(
                            /*x * Math.cos(heading) + y * Math.sin(heading),
                            -1 * x * Math.sin(heading) + y * Math.cos(heading),*/-gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            (gamepad1.left_trigger - gamepad1.right_trigger)/* + (heading2 - heading) * 0.1*/
                    )
            );
           // heading2 = heading;
        }
        /*else if (gamepad1.left_stick_y < 0) {

        }*/
        if (gamepad1.left_trigger + gamepad1.right_trigger != 0){
            heading2 = heading;
        }
          /*  drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            (gamepad1.left_trigger - gamepad1.right_trigger) + gamepad1.left_stick_y * 0.042 + Math.abs(gamepad1.left_stick_x) * 0.15
                    )
            );
        }
        /*if (nnr != nwL){
            if (T.milliseconds() > 300 && nwL != 0){
                drive.setWeightedDrivePower(
                        new Pose2d(
                                nwL,
                                -0,
                                -0
                        )
                );
                int y = (int) T.milliseconds() / 5;
                T = new ElapsedTime();
                while (T.milliseconds() < y);
            }
            nwL = nnr;
            T = new ElapsedTime();
        }
        else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            (gamepad1.left_trigger - gamepad1.right_trigger)
                    )
            );
        }*/


            telemetry.addData("heading", heading);
            lift.updateWithst();
            intake.teleop_v1(lift.lifPoseForIntake, 0);
            drive.update();
            //intake.teleop();
        /*if (gamepad2.y){
            if (lift.PositionTeleop1(Pos1) == 0){
                if (Pos1 > 0) Pos1 = 0;
                else {
                    Pos1 = 300;
                }
            }
            intake.teleop_v1(0, Pos1 > 0 ? 1 : -1);
        }
        else {
            lift.updateWithst();
            intake.teleop_v1(lift.lifPoseForIntake, 0);
        }*/
            telemetry.update();
        }
    }


    /*public void controlWithst() {

        heading = drive.getRawExternalHeading();
        x = -gamepad1.left_stick_y;
        y = -gamepad1.left_stick_x;


        if (gamepad2.y){
            lift.DoZeroLift_DigitalSensor();
        }

        boolean CanIntakeUp = !digitalTouch.getState();
        if (gamepad2.y){
            if (WithoutKonDigit)
                WithoutKonDigit = false;
            else
                WithoutKonDigit = true;
        }
        if((WithoutKonDigit || CanIntakeUp) && gamepad2.right_stick_y > 0) {
            servo3.setPower(1);
            servo2.setPower(-1);
        }
        else if(gamepad2.right_stick_y < 0) {
            servo3.setPower(-1);
            servo2.setPower(1);
        }
        else if(digitalTouch.getState() == true){
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
        }
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
                            -(gamepad1.left_stick_y) * Math.abs(gamepad1.left_stick_y),
                            -(gamepad1.left_stick_x) * Math.abs(gamepad1.left_stick_x),
                            -(gamepad1.right_trigger - gamepad1.left_trigger)
                    )
            );
        }
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
        lift.updateWithst();
        drive.update();
        intake.teleop();
        telemetry.update();
    }
}*/
