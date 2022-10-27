package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftPosition {

    private Gamepad gamepad2;
    private HardwareMap hardwareMap;
    private LinearOpMode linearOpMode;
    private DcMotorEx motor1, motor2;
    private double kF = 0;
    private double kP = 0;
    public LiftPosition(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        gamepad2 = linearOpMode.gamepad2;

        motor1 = hardwareMap.get(DcMotorEx.class, "liftmotor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "liftmotor2");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private enum State {
        HOLD,
        TELE
    }
    private State state = State.HOLD;
    public void Pos0(){
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(gamepad2.x) {
            while (motor1.getCurrentPosition() > 0 && motor2.getCurrentPosition() > 0) {
                motor1.setPower(-0.5);
                motor2.setPower(-0.5);
            }
        }
    }
    public void Pos1(){
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(gamepad2.y) {
            while (motor1.getCurrentPosition() < 100 && motor2.getCurrentPosition() < 100) {
                motor1.setPower(0.5);
                motor2.setPower(0.5);
            }
        }
    }
    public void Retention(){
        if (gamepad2.x != true && gamepad2.y != true){
            motor1.setPower(-kP * motor1.getCurrentPosition()+kF);
            motor2.setPower(-kP * motor1.getCurrentPosition() + kF);
        }
    }
}
