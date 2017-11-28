package org.firstinspires.ftc.teamcode.beta_log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by FTC Team 8397 on 11/24/2017.
 */

public abstract class LoggingLinearOpMode extends LinearOpMode {

    @Override
    public final void runOpMode() throws InterruptedException {
        try{
            BetaLog.initialize();
            runLoggingOpmode();
        }
        finally{
            BetaLog.close();
        }
    }

    public abstract void runLoggingOpmode() throws InterruptedException;

}
