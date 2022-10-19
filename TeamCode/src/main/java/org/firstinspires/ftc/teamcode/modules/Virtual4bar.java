package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Virtual4bar {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;
    private Gamepad gamepad2;

    private DcMotorEx motor;

    private double kF = 0;
    private double kP = 0;

    private double ticksPerRev = 1120;
    private double startAngle = 0; //in Radians
    private double holdPosition = 0;

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

        motor = hardwareMap.get(DcMotorEx.class, "v4bmotor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("", "Virtual 4 bar initialized!");
    }

    public void teleop() {
        double gravityCompensation = kF * Math.cos(motor.getCurrentPosition() / ticksPerRev * 2 * Math.PI + startAngle);
        if (Math.abs(gamepad2.right_stick_y) > 0)
            state = State.TELE;
        else
            state = State.HOLD;

        switch (state) {
            case HOLD:
                motor.setPower(kP * (holdPosition - motor.getCurrentPosition()) + gravityCompensation);
            case TELE:
                motor.setPower(gamepad2.right_stick_y + gravityCompensation);
                holdPosition = motor.getCurrentPosition();
        }
    }
}
