package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;
    private Gamepad gamepad2;
    private CRServo servo3;
    private CRServo servo2;

    private double kF = 0;
    private double kP = 0;
    private DcMotorEx motor1, motor2;
    //private DcMotorEx motor2;

    private enum State {
        HOLD,
        TELE
    }
    private State state = State.HOLD;

    public Lift(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad2 = linearOpMode.gamepad2;

        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servo3 = hardwareMap.get(CRServo.class, "servo3");

        motor1 = hardwareMap.get(DcMotorEx.class, "liftmotor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "liftmotor2");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("", "Lift initialized!");
    }

    public void teleop() {

        motor1.setPower(gamepad2.left_stick_y);
       motor2.setPower(gamepad2.left_stick_y);

        /* if (Math.abs(gamepad2.left_stick_y) > 0)
            state = State.TELE;
        else
            state = State.HOLD;

        switch (state) {
            case HOLD:
                motor1.setPower(-kP * motor1.getCurrentPosition() + kF);
                 motor2.setPower(-kP * motor1.getCurrentPosition() + kF);
            case TELE:
                motor1.setPower(gamepad2.left_stick_y + kF);
               motor2.setPower(gamepad2.left_stick_y + kF);

        }*/
        telemetry.addData("lift",motor1.getCurrentPosition());
        telemetry.update();
      /*  if(gamepad2.a) {
            while (gamepad2.a) {
                double power = 1;
                servo3.setPower(power);
                servo2.setPower(-power);
            }
        }
        if(gamepad2.b) {
            while (gamepad2.b) {
                double power = 1;
                servo3.setPower(-power);
                servo2.setPower(power);
            }
        }
        else {
            servo3.setPower(0);
            servo2.setPower(0);
        }*/
    }
}
