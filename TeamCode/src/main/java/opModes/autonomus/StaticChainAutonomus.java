package opModes.autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class StaticChainAutonomus extends LinearOpMode {
    DcMotor StCh;
    private int pos;
    private  int position;

    @Override
    public void runOpMode() throws InterruptedException {


        StCh = hardwareMap.get(DcMotorEx.class, "StaticChain");
        StCh.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StCh.setDirection(DcMotorSimple.Direction.REVERSE);
        StCh.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pos = 0;


        waitForStart();
        while (!isStarted()){
            //ChainMove(100, 0.25);
            idle();

        }
        while (!isStopRequested()){
            //ChainMove(100, 0.25);
            idle();
        }
    }

public void  ChainMove(int ChainPosition, double speed, double ki, double kd,double reference){
        pos += ChainPosition;
        StCh.setTargetPosition(pos);
        StCh.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        StCh.setPower(speed);
}

}

