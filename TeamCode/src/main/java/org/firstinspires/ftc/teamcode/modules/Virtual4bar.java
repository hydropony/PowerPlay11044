package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.misc.HardwareConfig;

public class Virtual4bar {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;
    private Gamepad gamepad2;

    private DcMotorEx motor;

    private double kF = 0;
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;

    private double P, I, D;

    private double ticksPerRev = 1120;
    private double startAngle = 0; //in Radians
    private double holdPosition = 0;
    private double prevTime = 0;
    public double error = 0;
    private double prevError = 0;
    private double kSlow = 93 / 1000.0;

    private enum State {
        FRONT,
        BACK,
        HOLD,
        TELE
    }
    private State state = State.HOLD;

    public Virtual4bar(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad2 = linearOpMode.gamepad2;

        motor = hardwareMap.get(DcMotorEx.class, HardwareConfig.V4B_MOTOR);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("", "Virtual 4 bar initialized, HOLD!");
    }

    public void teleop() {
        motor.setPower(gamepad2.right_stick_y);
    }

    public void startHold(double time) {
        error = holdPosition - motor.getCurrentPosition();
        P = kP * error;
        I += kI * error * (time - prevTime);
        D = kD * (error - prevError) / (time - prevTime);
        motor.setPower(P + I + D);
        prevError = error;
        prevTime = time;
    }

    public void slowDownMovement(double time) {
        error = holdPosition - motor.getCurrentPosition();
        while (Math.abs(error) > 30) {
            error = holdPosition - motor.getCurrentPosition();
            P = kP * error;
            I += kI * error * (time - prevTime);
            D = kD * (error - prevError) / (time - prevTime);
            motor.setPower(P + I + D);
            if (holdPosition > 0)
                holdPosition -= (time - prevTime) * kSlow;
            prevError = error;
            prevTime = time;
        }
    }
}
