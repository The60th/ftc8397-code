package org.firstinspires.ftc.teamcode.competition_in_work.auto;

import android.graphics.Color;

import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;

/**
 * Created by FTC Team 8397 on 11/28/2017.
 */

public class BlueBottomStartCryptoNav extends MechBotAutonomous{
    final float[] hsvValues = new float[3];
    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap, 0); //Init the hardware map with a starting angle of 0.
        //The starting angle is the gyro heading relative to the crypto box.

        initAuto(TeamColor.BLUE, 2000, 2000); //Find the targetJewl side and the target crypto key.

        knockJewel(this.targetSide);

        //Assume the robot is facing the wall once again still on the balance stone and the wall is a heading of 0.

        driveDirectionGyro(20, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                if (hsvValues[1] < .5) {
                    sleep(250);
                    return true;
                }
                return false;
            }
        });

        turnToHeadingGyro(0, 2, 100); //Turn to face the wall again.
    }

}
