package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot22;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.misc.HardwareConfig;

public class ServoAuto{

    private HardwareMap hardwareMap;

    private LinearOpMode linearOpMode;
    CRServo servo3, servo2;
    public ServoAuto(LinearOpMode linearOpMode){

        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servo3 = hardwareMap.get(CRServo.class, "servo3");

        servo3 = hardwareMap.get(CRServo.class, HardwareConfig.INTAKE_SERVO_1);
        servo2 = hardwareMap.get(CRServo.class, HardwareConfig.INTAKE_SERVO_2);

        /*digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);*/

        servo3.setDirection(DcMotorSimple.Direction.FORWARD);
        servo2.setDirection(DcMotorSimple.Direction.FORWARD);
        }

    public void run(){
        ElapsedTime t = new ElapsedTime();
        while (t.milliseconds() < 700) {
            servo3.setPower(1);
            servo2.setPower(-1);
        }
        servo3.setPower(0);
        servo2.setPower(0);
    }
}
