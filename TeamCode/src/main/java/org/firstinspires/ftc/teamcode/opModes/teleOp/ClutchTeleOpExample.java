package org.firstinspires.ftc.teamcode.clutch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClutchTeleOpExample extends LinearOpMode {
    //Servo srv1;
    //Servo srv2;

    CRServo servo3;
    CRServo servo2;

    @Override
    public void runOpMode() throws InterruptedException {
        //srv1 = hardwareMap.get(Servo.class, "S1");
        //srv2 = hardwareMap.get(Servo.class, "S2");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servo3 = hardwareMap.get(CRServo.class, "servo3");

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a) {
                while (gamepad1.a) {
                    double power = 1;
                    servo3.setPower(power);
                    servo2.setPower(-power);
                }
            }
            if(gamepad1.b) {
                while (gamepad1.b) {
                    double power = 1;
                    servo3.setPower(-power);
                    servo2.setPower(power);
                }
            }
            else {
                servo3.setPower(0);
                servo2.setPower(0);
            }
            /*else if(gamepad1.y){
                srv1.setPosition(0.5);
                srv2.setPosition(0.5);
            }
            else if(gamepad1.x){
                srv1.setPosition(1);
                srv2.setPosition(1);
            }
            else{
                telemetry.addLine("click on button");
                telemetry.update();
            }*/
            }
        }
    }

