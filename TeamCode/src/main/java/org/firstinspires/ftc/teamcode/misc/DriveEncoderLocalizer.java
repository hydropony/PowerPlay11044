package org.firstinspires.ftc.teamcode.misc;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.List;

public class DriveEncoderLocalizer {
    private HardwareMap hardwareMap;
    public static double TICKS_PER_REV = 0;
    public static double WHEEL_RADIUS = 1.97; // in
    private double x, y, heading;
    private double forwardVelo, strafeVelo;

    private Encoder leftFrontEnc, rightFrontEnc, rightBackEnc, leftBackEnc;
    private double leftFrontVelo, rightFrontVelo, rightBackVelo, leftBackVelo; //in ticks per second
    private SampleMecanumDrive drive;
    private double prevtime = 0;

    public DriveEncoderLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        this.hardwareMap = hardwareMap;
        this.drive = drive;

        leftFrontEnc = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConfig.LEFT_FRONT_MOTOR));
        leftBackEnc = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConfig.LEFT_BACK_MOTOR));
        rightFrontEnc = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConfig.RIGHT_FRONT_MOTOR));
        rightBackEnc = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConfig.RIGHT_BACK_MOTOR));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    private void updateMotorVelo() {
        leftFrontVelo = leftFrontEnc.getRawVelocity();
        leftBackVelo = leftBackEnc.getRawVelocity();
        rightFrontVelo = rightFrontEnc.getRawVelocity();
        rightBackVelo = rightBackEnc.getRawVelocity();
    }

    private void calculateKinematics() {
        updateMotorVelo();
        forwardVelo = encoderTicksToInches((leftFrontVelo + leftBackVelo + rightFrontVelo + rightBackVelo) / 4);
        strafeVelo = encoderTicksToInches((leftBackVelo + rightFrontVelo - leftFrontVelo - rightBackVelo) / 4);
    }

    public void update(double time) {
        calculateKinematics();
        heading = getHeading();
        y += forwardVelo * (time - prevtime) * Math.cos(heading);
        x += strafeVelo * (time - prevtime) * Math.sin(heading);
        prevtime = time;
    }

    public void stopUpdating() {
        prevtime = 0;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return  y;
    }
}
