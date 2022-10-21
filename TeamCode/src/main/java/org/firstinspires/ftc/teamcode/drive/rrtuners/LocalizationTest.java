package org.firstinspires.ftc.teamcode.drive.rrtuners;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.Lift;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    DcMotor StCh;
    CRServo servo3;
    CRServo servo2;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double ref = 0;
    double kv = 0.1;
    double prevTime = 0;
    double prevError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Lift lift = new Lift(this);

        StCh = hardwareMap.get(DcMotor.class, "StaticChain");
        StCh.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        StCh.setDirection(DcMotorSimple.Direction.FORWARD);
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servo3 = hardwareMap.get(CRServo.class, "servo3");



        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {

            double err = ref - StCh.getCurrentPosition();
            double P = kp * err;
            double I = ki * err * (time - prevTime);
            double D = kd * (err - prevError) / (time - prevTime);
            prevError = err;
            prevTime = time;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            - gamepad1.left_stick_y
                            -gamepad1.left_stick_x,
                            -(gamepad1.right_trigger - gamepad1.left_trigger)

                    )
            );

            if(gamepad2.right_stick_y <0.1 && gamepad2.right_stick_y >= 0){
                StCh.setPower(0);
            }
            else{
                StCh.setPower(gamepad2.right_stick_y);
            }


            if(gamepad2.a) {
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
            }

            if(err > 30){
                StCh.setPower(gamepad2.right_stick_y * kv);
            }
            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
