package org.firstinspires.ftc.teamcode.competition_in_work.auto.old;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;

/**
 * Created by FTC Team 8397 on 12/2/2017.
 */
@Disabled
public class ArmAutoTesting extends MechBotAutonomous {
    @Override
    public void runLoggingOpmode() throws InterruptedException {
        //11 inches from wall
        //7.5 from crpto box.
        bot.init(hardwareMap,0); //Init the hardware map with a starting angle of 0.
        waitForStart();
        //extendCollecter(MIN_COLLECTER_DRIVE_TIME_BEFORE_ARM_CAN_BE_LIFTED); //Locks thread

        //raiseArm(ARM_RAISE_TIME_FOR_GLYPH + 200); //Locks thread

        //Servo methods do not lock thread.
       // lowerRamp();
       // kickBlock();

        sleep(400);
        //resetKicker();
        sleep(1250);

       // kickBlock();
        sleep(5000);

        //raiseRamp();
        sleep(5000);

        //sleep(3000);
        //kickBlock();

   }
}
