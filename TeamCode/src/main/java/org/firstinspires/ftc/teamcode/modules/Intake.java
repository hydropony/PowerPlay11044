package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.misc.HardwareConfig;

public class Intake {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;
    private Gamepad gamepad1;
    //public DigitalChannel digitalTouch;

    private CRServo servo1;
    private CRServo servo2;

    public Intake(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad1 = linearOpMode.gamepad1;

        servo1 = hardwareMap.get(CRServo.class, HardwareConfig.INTAKE_SERVO_1);
        servo2 = hardwareMap.get(CRServo.class, HardwareConfig.INTAKE_SERVO_2);

        /*digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);*/

        servo1.setDirection(DcMotorSimple.Direction.FORWARD);
        servo2.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("", "Intake initialized!");
    }

    public void teleop() {
        if(gamepad1.a /*&& digitalTouch.getState() == true*/) {
            servo1.setPower(1);
            servo2.setPower(-1);
        }
        else if(gamepad1.b) {
            servo1.setPower(-1);
            servo2.setPower(1);
        }
        else {
            servo1.setPower(0);
            servo2.setPower(0);
        }
    }
}
