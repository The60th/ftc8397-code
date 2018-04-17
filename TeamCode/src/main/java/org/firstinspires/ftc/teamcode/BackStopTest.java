package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotScranton;

/**
 * Created by FTC Team 8397 on 4/15/2018.
 */
@TeleOp(name = "Back stopped tester servo",group = "Test")
public class BackStopTest extends LoggingLinearOpMode {
    private MechBotScranton bot = new MechBotScranton();
    double pos = .50;
    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap);
        bot.backStop.setPosition(pos);
        waitForStart();

        ElapsedTime et = new ElapsedTime();

        while (opModeIsActive()){
            if (et.milliseconds() < 10) continue;
            et.reset();

            if(gamepad1.dpad_down) {
                pos = pos - .002;
                if (pos <= 0) pos = 0;
                bot.backStop.setPosition(pos);
            }
            else  if(gamepad1.dpad_up){
                pos = pos+.002;
                if(pos >= 1.0) pos = 1.0;
                bot.backStop.setPosition(pos);
            }

            telemetry.addData("Pos: ", pos);
            telemetry.update();
        }
    }
}
