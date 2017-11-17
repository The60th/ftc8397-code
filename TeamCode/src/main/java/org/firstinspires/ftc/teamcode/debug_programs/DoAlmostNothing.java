package org.firstinspires.ftc.teamcode.debug_programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechbot.MechBotSensor;

/**
 * Created by FTC Team 8397 on 10/27/2017.
 */
@Autonomous( name= "Do Almost Nothing", group = "Test")
public class DoAlmostNothing extends LinearOpMode {
    MechBotSensor bot = new MechBotSensor();
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        bot.setDriveSpeed(30,0,0);
        sleep(2000);
        bot.setDriveSpeed(0,0,0);
        sleep(1000);

        bot.setDriveSpeed(0,30,0);
        sleep(2000);
        bot.setDriveSpeed(0,0,0);
        sleep(1000);

        bot.setDriveSpeed(0,0,.5);
        sleep(2000);
        bot.setDriveSpeed(0,0,0);
        sleep(2000);

    }
}
