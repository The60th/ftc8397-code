package org.firstinspires.ftc.teamcode.competition_in_work.auto.scranton;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotAutonomousScranton;

/**
 * Created by FTC Team 8397 on 3/6/2018.
 */
@Autonomous(name = "Blue Bottom Encoder", group = "Auto")
public class BlueBottomScrantonEncoder extends MechBotAutonomousScranton{
    final float[] hsvValues = new float[3];

    final boolean BLUE_BOTTOM_START_LOG = true;
    final String BLUE_BOTTOM_START_TAG = "Blue bottom start Red Hook:";

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap,-90); //The starting value of the gyro heading comapred to the wall.

        //The starting angle is the gyro heading relative to the crypto box.
        robotZXPhi = new float[3];

        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "INITIALIZE AUTO");

        //Contains wait for start.
        initAuto(MechBotAutonomousScranton.TeamColor.BLUE, VUMARK_KEY_SCAN_TIME,JEWEL_SCAN_TIME); //Knocks the jewel off the stone and finds crypto key.

        //Assume the robot is facing the wall once again still on the balance stone and the wall is a heading of 0.
        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "driveDirectionGyro 1");
        //added the 180 to this line of code to keep the robot from turning around.
        driveDirectionGyro(OFF_STONE_SPEED, 90,-90, new MechBotAutonomousScranton.Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);

                if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "Driving on stone sats: S: %.2f",hsvValues[1]);

                if(hsvValues[1] < HSV_SAT_CUT_OFF_STONE){
                    //Color sensors are off the stone.
                    return true;
                }
                return false;
            }
        });

        robotZXPhi = new float[]{0,0,bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        //Robot is now partly off the stone. Just the front color sensors are off, time to drive the rest of the robot off the stone.
        driveDirectionGyro(45, 90, -90, new MechBotAutonomousScranton.Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[1] > 66; //Need a constant defined here.
            }
        });

        //Robot is now all the way off the balance stone and ready to turn towards the crypto box.
        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "turnToheadingGyro");


        turnToHeadingGyroQuick(0,GLOBAL_STANDERD_TOLERANCE,GLOBAL_STANDERD_LATENCY, MechBotAutonomousScranton.RotationDirection.COUNTER_CLOCK); //Turn to face the wall.

        prepareToScoreGlyphQuick();
    }
}
