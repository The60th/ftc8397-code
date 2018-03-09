package org.firstinspires.ftc.teamcode.competition_in_work.auto.scranton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotAutonomousScranton;

/**
 * Created by FTC Team 8397 on 3/8/2018.
 */
@Autonomous(name = "Crazy stuff",group = "Auto")
public class AutoQuickTurnDemo extends MechBotAutonomousScranton {

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap,0); //The starting value of the gyro heading comapred to the wall.
        robotZXPhi = new float[3];
        waitForStart();

        robotZXPhi = new float[] {0,0,bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        multiGlyph();
    }
}
