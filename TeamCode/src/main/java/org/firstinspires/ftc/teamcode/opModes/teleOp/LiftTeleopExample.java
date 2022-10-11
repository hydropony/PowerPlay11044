package org.firstinspires.ftc.teamcode.lift;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class LiftTeleopExample extends LinearOpMode {
    DcMotor lift1,lift2;
    double power1,pos1, pos2, err1, err2, u;

    @Override
    public void runOpMode() throws InterruptedException {
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift1.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        lift2.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        lift2.setDirection(DcMotorSimple.Direction.FORWARD);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        while(opModeIsActive()){

                power1 = gamepad1.right_stick_y;
               pos1 = lift1.getCurrentPosition();
               pos2 = lift2.getCurrentPosition();
                err1 = pos1 - pos2;
                err2 = pos2 - pos1;

                if(err1 != 0){

                    lift2.setPower(1);
                }
                else if(err2 != 0){
                    lift1.setPower(1);
                }

            if(gamepad1.right_stick_y < 0.1 || gamepad1.right_stick_y == 0){
                lift1.setPower(0);
                lift2.setPower(0);
            }
            else{

                lift1.setPower(power1);
                lift2.setPower(power1);
            }
        }

    }
}
