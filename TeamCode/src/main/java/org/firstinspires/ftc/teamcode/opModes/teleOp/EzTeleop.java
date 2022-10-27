package org.firstinspires.ftc.teamcode.opModes.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.modules.Lift;
import org.firstinspires.ftc.teamcode.modules.LiftPosition;
import org.opencv.core.Mat;
import java.util.List;

@TeleOp
public class EzTeleop extends LinearOpMode {
    DcMotorEx lf, lb, rf, rb;
    DcMotorEx stCh, lift1, lift2;
    CRServo servo3;
    CRServo servo2;
    double err, konst, konst1;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.get(DcMotorEx.class, "leftFront");
        lb = hardwareMap.get(DcMotorEx.class, "leftRear");
        rb = hardwareMap.get(DcMotorEx.class, "rightRear");
        rf = hardwareMap.get(DcMotorEx.class, "rightFront");
        stCh = hardwareMap.get(DcMotorEx.class, "StaticChain");
        lift1 = hardwareMap.get(DcMotorEx.class, "liftmotor1");
        lift2 = hardwareMap.get(DcMotorEx.class, "liftmotor2");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        servo3 = hardwareMap.get(CRServo.class, "servo3");

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stCh.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stCh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setDirection(DcMotorSimple.Direction.REVERSE); // Mb nado pomenyat na rb i rf, protestit'
        lb.setDirection(DcMotorSimple.Direction.REVERSE); //Nado chtobi pri podache 1 na vse motori kb robot ehal vpered

        waitForStart();

        while (!isStopRequested()) {
            LiftPosition lift = new LiftPosition(this);
            //Lift lift = new Lift(this);
            double ref = 50;
            err = ref - stCh.getCurrentPosition();
            konst = 0.7;

            konst1 = 0.2;
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rotation = gamepad1.right_trigger - gamepad1.left_trigger;
            //boolean rotationSlow = gamepad1.right_bumper;
            double lfpower, lbpower, rfpower, rbpower;
            lfpower = y + x + rotation;
            lbpower = y - x + rotation;
            rfpower = y - x - rotation;
            rbpower = y + x - rotation;
            double[] powers = {Math.abs(lfpower), Math.abs(lbpower), Math.abs(rfpower), Math.abs(rbpower)};
            double max = 0;
            for (int i = 0; i < 3; i++) {
                if (powers[i] > max)
                    max = powers[i];
            }
            if (max > 1) {
                lfpower /= max;
                lbpower /= max;
                rfpower /= max;
                rbpower /= max;
            }
            lf.setPower(lfpower);
            lb.setPower(-lbpower);
            rf.setPower(-rfpower);
            rb.setPower(-rbpower);

            stCh.setPower(gamepad2.right_stick_y);

            if(gamepad2.a) {
                servo3.setPower(1);
                servo2.setPower(-1);
            }
            if(gamepad2.b) {
                servo3.setPower(-1);
                servo2.setPower(1);
            }
            if(gamepad1.a) {
                servo3.setPower(1);
                servo2.setPower(-1);
            }
            if(gamepad1.b) {
                servo3.setPower(-1);
                servo2.setPower(1);
            }
            else {
                servo3.setPower(0);
                servo2.setPower(0);
            }


            lift.Pos0();
            lift.Pos1();
            lift.Retention();

        }


        if(Math.abs(err) > 30){
            stCh.setPower(gamepad2.right_stick_y*konst1);
        }

    }
}
