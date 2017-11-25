package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.beta_log.BetaLog;

/**
 * Created by JimLori on 11/18/2017.
 */

@Autonomous (name = "TestBetaLog", group = "Test")
public class TestBetaLog extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        try{
            BetaLog.initialize();

            ElapsedTime et = new ElapsedTime();
            ElapsedTime totalElapsedTime = new ElapsedTime();
            int outputCount = 0;
            int loopCount = 0;
            float floatLoopCount = 0f;

            waitForStart();

            while (opModeIsActive()){
                loopCount++;
                floatLoopCount += 1.0f;
                if (et.milliseconds() < 500) continue;
                et.reset();
                outputCount++;
                BetaLog.dd("TEST","Count = %d Millisec = %.1f", outputCount, totalElapsedTime.milliseconds());
            }
        }
        finally{
            BetaLog.close();
        }

    }
}
