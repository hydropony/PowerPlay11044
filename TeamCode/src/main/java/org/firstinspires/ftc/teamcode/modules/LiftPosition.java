package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftPosition {

    private Gamepad gamepad2;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;
    private DcMotorEx motor1, motor2;
    private double kF = 0;
    private double kP = 0;
    public LiftPosition(LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
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
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(gamepad2.y) {
            while (motor1.getCurrentPosition() < -200) {
                motor1.setPower(1);
                motor2.setPower(1);
            }
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }
    public void Pos1(){
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(gamepad2.a) {
            while (motor1.getCurrentPosition() > -1400) {
                motor1.setPower(-1);
                motor2.setPower(-1);
            }
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }
    public void Pos2(){
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(gamepad2.b) {
            while (motor1.getCurrentPosition() > -2800) {
                motor1.setPower(-1);
                motor2.setPower(-1);
            }
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }
    // Pos 3 is correct variant
    public void Pos3(){
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(gamepad2.x) {
            while (motor1.getCurrentPosition() > -4200) {
                motor1.setPower(-1);
                motor2.setPower(-1);
            }
                motor1.setPower(0);
                motor2.setPower(0);
        }

    }

    public void Retention(){
        if (gamepad2.x != true && gamepad2.y != true){
            motor1.setPower(-kP * motor1.getCurrentPosition()+kF);
            motor2.setPower(-kP * motor1.getCurrentPosition() + kF);
        }
    }
}
