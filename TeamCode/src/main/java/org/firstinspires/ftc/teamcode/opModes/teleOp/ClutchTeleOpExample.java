package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp
public class ClutchTeleOpExample extends LinearOpMode {

    CRServo servo3;
    CRServo servo2;

    @Override
    public void runOpMode() throws InterruptedException {
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
            }
        }
    }

