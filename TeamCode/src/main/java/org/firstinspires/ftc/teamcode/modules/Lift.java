package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;
    private Gamepad gamepad2;

    private double kF = 0;
    private double kP = 0.01;
    private double error;
    private double intakePos = 0;
    private double lowPos = -2000;
    private double middlePos = -2500;
    private double highPos = -4300;
    private double groundPos = -300;
    private DcMotorEx motor1, motor2;

    private enum State {
        BYPASS,
        INTAKE,
        GROUND,
        LOW,
        MIDDLE,
        HIGH,
        NULL
    }
    private State state = State.BYPASS;

    public Lift(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad2 = linearOpMode.gamepad2;


        motor1 = hardwareMap.get(DcMotorEx.class, "liftmotor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "liftmotor2");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("", "Lift initialized!");
    }
    public void update(){

        if (gamepad2.dpad_down){
            state = State.INTAKE;
        }
        if (gamepad2.dpad_up){
            state = State.HIGH;
        }
        if (gamepad2.dpad_left){
            state = State.LOW;
        }
        if (gamepad2.dpad_right){
            state = State.MIDDLE;
        }
        if(gamepad2.right_bumper){
            state = State.BYPASS;
        }
        if(gamepad2.left_bumper){
            state = State.GROUND;
        }
        if(gamepad2.left_stick_y >= 0 && motor1.getCurrentPosition() > 0){
            state = State.NULL;
        }
        else{
            state = State.BYPASS;
        }

        switch (state) {
            case NULL:
                motor1.setPower(0);
                motor2.setPower(0);
                break;
            case BYPASS:
                motor1.setPower(gamepad2.left_stick_y);
                motor2.setPower(gamepad2.left_stick_y);
                break;
            case INTAKE:
                error = intakePos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case GROUND:
                error = groundPos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case LOW:
                error = lowPos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case MIDDLE:
                error = middlePos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case HIGH:
                error = highPos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
        }
        telemetry.addData("gamepad", gamepad2.left_stick_y);
        telemetry.addData("error", error);
        telemetry.addData("lift",motor1.getCurrentPosition());
        telemetry.addData("state", state);
        telemetry.update();


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
    public void Pos0() {

        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < 2300) {
            motor1.setPower(-1);
            motor2.setPower(-1);
        }
        motor1.setPower(0);
        motor2.setPower(0);
    }
    public void Pos1(){
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(gamepad2.dpad_down) {
            while (motor1.getCurrentPosition() < 1450) {
                motor1.setPower(1);
                motor2.setPower(-1);
            }
            motor1.setPower(0);
            motor2.setPower(0);
        }
    }
    public void Pos2(){
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(gamepad2.dpad_right) {
            while (motor1.getCurrentPosition() < 2000) {
                motor1.setPower(1);
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

        if(gamepad2.dpad_up) {
            while (motor1.getCurrentPosition() < 2500) {
                motor1.setPower(1);
                motor2.setPower(-1);
            }
            motor1.setPower(0);
            motor2.setPower(0);
        }

    }
}
