package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderMecanumDrive {

    private int pos;
    private double err;
    private LinearOpMode linearOpMode;
    private HardwareMap hardwareMap;
    private DcMotor leftFront, rightFront, leftRear, rightRear;
    public  EncoderMecanumDrive(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        pos = 0;
    }
    /*public void EncoderTurnLeft(double MecanumPosition, double speed){
        pos += MecanumPosition;
        leftFront.setTargetPosition(pos);
        rightFront.setTargetPosition(pos);
        leftRear.setTargetPosition(pos);
        rightRear.setTargetPosition(pos);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(-speed);
        rightFront.setPower(speed);
        leftRear.setPower(-speed);
        rightRear.setPower(speed);
    }
   /* public void EncoderTurnRight(double MecanumPosition, double speed){
        pos += MecanumPosition;
        leftFront.setTargetPosition(pos);
        rightFront.setTargetPosition(pos);
        leftRear.setTargetPosition(pos);
        rightRear.setTargetPosition(pos);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        rightFront.setPower(-speed);
        leftRear.setPower(speed);
        rightRear.setPower(-speed);
    }*/
   public void EncoderDrive(double pos1, double pos2, double speed){

       leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (leftFront.getCurrentPosition() < pos1 && rightFront.getCurrentPosition() < pos2 && leftRear.getCurrentPosition() < pos1 && rightRear.getCurrentPosition() < pos2) {
            leftFront.setPower(speed);
            rightFront.setPower(speed);
            leftRear.setPower(speed);
            rightRear.setPower(speed);
        }
    }
    public void EncoderForwardDrive( double speed){
        ElapsedTime t = new ElapsedTime();
       t.reset();
      /// while (t.milliseconds() < time){
            leftFront.setPower(speed);
            rightFront.setPower(speed);
            leftRear.setPower(speed);
            rightRear.setPower(speed);
        }
        public void TurnLeft(double speed){
            ElapsedTime t = new ElapsedTime();
            t.reset();
            /// while (t.milliseconds() < time){
            leftFront.setPower(speed);
            rightFront.setPower(-speed);
            leftRear.setPower(speed);
            rightRear.setPower(-speed);
        }
        public void TurnRight(double speed){
            ElapsedTime t = new ElapsedTime();
            t.reset();
            /// while (t.milliseconds() < time){
            leftFront.setPower(-speed);
            rightFront.setPower(speed);
            leftRear.setPower(-speed);
            rightRear.setPower(speed);
        }

    public void TagId12(){

            while (leftFront.getCurrentPosition() > -2000) {
               leftFront.setPower(-0.5);
                rightFront.setPower(0.5);
                rightRear.setPower(0.5);
                leftRear.setPower(0.5);
        }
    }
        /* leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
    /* public void EncoderMovementToTheSide(double speed, double time){
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < time){
            leftFront.setPower(speed);
            rightFront.setPower(-speed);
            leftRear.setPower(-speed);
            rightRear.setPower(speed);
        }*/
    }

