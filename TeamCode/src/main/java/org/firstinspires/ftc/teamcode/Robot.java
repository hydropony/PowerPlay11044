package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Robot {
    protected LinearOpMode opMode = null;
    protected HardwareMap hardwareMap = null;
    protected Telemetry telemetry = null;

    protected Gamepad gamepad1 = null;
    protected Gamepad gamepad2 = null;

    public Robot(LinearOpMode opMode) { //basic init
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;
    }

    public final void delay(long milliseconds) {
        ElapsedTime t = new ElapsedTime();
        double t0 = t.milliseconds();
        while (t.milliseconds() - t0 < milliseconds && !opMode.isStopRequested()){}
    }
}
