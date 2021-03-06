package org.firstinspires.ftc.teamcode.competition_in_work.auto.scranton;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotAutonomousScranton;

/**
 * Created by FTC Team 8397 on 3/1/2018.
 */
@Autonomous(name = "Blue Bottom", group = "Auto")
@Disabled
public class BlueBottomScranton extends MechBotAutonomousScranton {
    final float[] hsvValues = new float[3];

    final boolean BLUE_BOTTOM_START_LOG = true;
    final String BLUE_BOTTOM_START_TAG = "Blue bottom start Red Hook:";
    final float OFF_STONE_DISTANCE = 68.5f;
    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap, 180); //The starting value of the gyro heading comapred to the wall.

        //The starting angle is the gyro heading relative to the crypto box.
        robotZXPhi = new float[3];

        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "INITIALIZE AUTO");
        //Contains wait for start.
        initAuto(TeamColor.BLUE, VUMARK_KEY_SCAN_TIME, JEWEL_SCAN_TIME); //Knocks the jewel off the stone and finds crypto key.
        telemetry.update();

        setOdometry(0,0);
        driveDirectionGyro(OFF_STONE_SPEED, 90, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[1] > OFF_STONE_DISTANCE;
            }
        });
        setOdometry(0,0);

        driveDirectionGyro(DRIVE_TOWARDS_TRIANGLE_SPEED, 90,180, new Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(bot.backColorLeft.red() * 8, bot.backColorLeft.green() * 8, bot.backColorLeft.blue() * 8, hsvValues);
                return hsvValues[1] > HSV_SAT_CUT_OFF;
            }
        });


        handleTriangle(TriangleApproachSide.LEFT,LINE_FOLLOW_SPEED,20,180,bot.backColorLeft,bot.backColorRight,2000);

        freeFlipPlate();

        scoreGlyph(this.cryptoKey);

    }
}



