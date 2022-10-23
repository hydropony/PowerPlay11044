package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class LiftTeleopExample extends LinearOpMode {
    DcMotorEx lift1,motor2;
    double power1,pos1, pos2, err1, err2, u;

    @Override
    public void runOpMode() throws InterruptedException {
        //lift1 = hardwareMap.get(DcMotor.class, "lift1");
        //lift1.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        //lift1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2 = hardwareMap.get(DcMotorEx.class, "liftmotor2");
        motor2.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        //lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        while(opModeIsActive()){

                power1 = gamepad1.right_stick_y;


            if(gamepad1.right_stick_y == 0){
                //lift1.setPower(0);
                motor2.setPower(0);
            }
            else{

                //lift1.setPower(power1);
                motor2.setPower(power1);
            }
        }

    }
}
