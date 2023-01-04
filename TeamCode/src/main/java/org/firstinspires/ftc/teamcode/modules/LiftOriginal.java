package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftOriginal {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;
    private Gamepad gamepad2;
    private AnalogInput conz;
    private DigitalChannel digitalTouch;
    public ServoAuto servo;

    /*
 digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
 digitalTouch.setMode(DigitalChannel.Mode.INPUT);
     */

    private double kF = 0;
    private double kP = 0.01;
    private double error;
    private double intakePos = 0;
    private double lowPos = -2000;
    private double middlePos = -2900;
    private double nwZero = 0;
    private double highPos = -4300;
    private double groundPos = -300;
    private DcMotorEx motor1, motor2;
    private boolean candonwz = false;

    public enum State {
        BYPASS,
        INTAKE,
        GROUND,
        LOW,
        MIDDLE,
        HIGH,
        NWZERO,
        CANDON,
    }

    public State state = State.BYPASS;

    public LiftOriginal(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        servo = new ServoAuto(linearOpMode);
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad2 = linearOpMode.gamepad2;
        conz = hardwareMap.get(AnalogInput.class, "sensor_analog");


        motor1 = hardwareMap.get(DcMotorEx.class, "liftmotor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "liftmotor2");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("", "Lift initialized!");
    }

    private int znakdbl(double a) {
        if (a > 0)
            return 1;
        else if (a < 0)
            return -1;
        else
            return 0;
    }

    public void update() {

        if (gamepad2.dpad_down) {
            state = State.NWZERO;
        } else if (gamepad2.dpad_up) {
            state = State.HIGH;
        } else if (gamepad2.dpad_left) {
            state = State.LOW;
        } else if (gamepad2.dpad_right) {
            state = State.MIDDLE;
        } else if (gamepad2.right_bumper) {
            state = State.BYPASS;
        } else if (gamepad2.left_bumper) {
            state = State.GROUND;
        } else if (gamepad2.x) {
            state = State.CANDON;
        } else {
            state = State.BYPASS;
        }

        switch (state) {
            case BYPASS:
                if (gamepad2.left_stick_y >= 0 && motor1.getCurrentPosition() > nwZero) {
                    motor1.setPower(0);
                    motor2.setPower(0);
                } else {
                    motor1.setPower(gamepad2.left_stick_y);
                    motor2.setPower(gamepad2.left_stick_y);
                }
                if (Math.abs(gamepad2.left_stick_x) > 0.7 && candonwz) {
                    motor1.setPower(znakdbl(gamepad2.left_stick_x) * 0.25);
                    motor2.setPower(znakdbl(gamepad2.left_stick_x) * 0.25);
                }
                break;
            case INTAKE:
                error = intakePos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
            case NWZERO:
                nwZero = motor1.getCurrentPosition();
                break;
            case CANDON:
                if (candonwz)
                    candonwz = false;
                else
                    candonwz = true;
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
        telemetry.addData("lift", motor1.getCurrentPosition());
        telemetry.addData("state", state);
        telemetry.update();


    }

    public void DoZeroLift_DigitalSensor() {
        if (!(digitalTouch.getState())) {
            while (!(digitalTouch.getState())) {
                motor1.setPower(-1);
                motor2.setPower(-1);
            }
            motor1.setPower(0);
            motor2.setPower(0);
        }
        while ((digitalTouch.getState())) {
            motor1.setPower(0.25);
            motor2.setPower(0.25);
        }
        motor1.setPower(0);
        motor2.setPower(0);
        nwZero = motor1.getCurrentPosition();
    }


    public void updateavto() {
        /*error = lowPos - motor1.getCurrentPosition();
        motor1.setPower(kP * error);
        motor2.setPower(kP * error);*/
        switch (state) {
            case BYPASS:
                if (gamepad2.left_stick_y >= 0 && motor1.getCurrentPosition() > nwZero) {
                    motor1.setPower(0);
                    motor2.setPower(0);
                } else {
                    motor1.setPower(gamepad2.left_stick_y);
                    motor2.setPower(gamepad2.left_stick_y);
                }
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
                while (motor1.getCurrentPosition() > middlePos) {
                    motor1.setPower(-1);
                    motor2.setPower(-1);
                }

                motor1.setPower(0);
                motor2.setPower(0);

                /*
                error = middlePos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);*/
                break;
            case HIGH:
                error = highPos - motor1.getCurrentPosition();
                motor1.setPower(kP * error);
                motor2.setPower(kP * error);
                break;
        }
        telemetry.addData("gamepad", gamepad2.left_stick_y);
        telemetry.addData("error", error);
        telemetry.addData("lift", motor1.getCurrentPosition());
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

        telemetry.addData("lift", motor1.getCurrentPosition());
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
       /* error = 1600 - motor1.getCurrentPosition();
        motor1.setPower(-kP * error);
        motor2.setPower(-kP * error);*/

        while (motor1.getCurrentPosition() < 1455) {
            motor1.setPower(1);
            motor2.setPower(1);
        }
        /*ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < 4000) {
            motor1.setPower(1);
            motor2.setPower(1);
        }
        motor1.setPower(0);
        motor2.setPower(0);*/
    }

    public void Pos1() {
        /*error = 6000 - motor1.getCurrentPosition();
        motor1.setPower(kP * error);
        motor2.setPower(kP * error);*/
        while (motor1.getCurrentPosition() > 0) {
            motor1.setPower(-1);
            motor2.setPower(-1);
        }
        motor1.setPower(0);
        motor2.setPower(0);

        /*ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < 4000) {
            motor1.setPower(1);
            motor2.setPower(1);
        }
        motor1.setPower(0);
        motor2.setPower(0);*/
    }

    public void Pos2() {
        while (motor1.getCurrentPosition() > 320) {
            motor1.setPower(-1);
            motor2.setPower(-1);
        }
        motor1.setPower(0);
        motor2.setPower(0);
    }

    // Pos 3 is correct variant
   public void Pos3() {
        while (motor1.getCurrentPosition() > 600) {
            motor1.setPower(-1);
            motor2.setPower(-1);
        }
        motor1.setPower(0);
        motor2.setPower(0);
    }
    public void Pos4() {
        /*error = 6000 - motor1.getCurrentPosition();
        motor1.setPower(-kP * error);
        motor2.setPower(-kP * error);*/

        while (motor1.getCurrentPosition() < 1200) {
            motor1.setPower(1);
            motor2.setPower(1);
        }
        motor1.setPower(0);
        motor2.setPower(0);
        /*ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < 4000) {
            motor1.setPower(1);
            motor2.setPower(1);
        }
        motor1.setPower(0);
        motor2.setPower(0);*/
    }
}
