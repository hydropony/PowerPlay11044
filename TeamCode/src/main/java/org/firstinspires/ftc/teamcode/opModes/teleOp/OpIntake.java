package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Robot22;
import org.firstinspires.ftc.teamcode.modules.Intake;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class OpIntake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Robot22 R = new Robot22(this);
        AnalogInput digitalTouch;
        digitalTouch = hardwareMap.get(AnalogInput.class, "sensor_analog");
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (!isStopRequested()) {
            if (digitalTouch.getVoltage() > 0){
                telemetry.addData("", "Pressed");
            }
            telemetry.addData("", digitalTouch.getVoltage());
            telemetry.update();

            //R.controlWithst();
        }

    }
}