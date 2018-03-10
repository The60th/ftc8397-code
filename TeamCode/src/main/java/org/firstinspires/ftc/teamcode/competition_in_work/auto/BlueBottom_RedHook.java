package org.firstinspires.ftc.teamcode.competition_in_work.auto;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;

/**
 * Created by FTC Team 8397 on 11/22/2017.
 */
@Autonomous(name="Blue Bottom",group = "comp")
@Disabled
public class BlueBottom_RedHook extends MechBotAutonomous {
    final float[] hsvValues = new float[3];

    final boolean BLUE_BOTTOM_START_LOG = true;
    final String BLUE_BOTTOM_START_TAG = "Blue bottom start Red Hook:";

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap,180); //Init the hardware map with a starting angle of 0.
        //The starting angle is the gyro heading relative to the crypto box.
        robotZXPhi = new float[3];

        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "INITIALIZE AUTO");
        initAuto(TeamColor.BLUE, VUMARK_KEY_SCAN_TIME,JEWEL_SCAN_TIME); //Find the targetJewl side and the target crypto key.
        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "KNOCK JEWEL");


        //Assume the robot is facing the wall once again still on the balance stone and the wall is a heading of 0.
        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "driveDirectionGyro 1");
        //added the 180 to this line of code to keep the robot from turning around.
        driveDirectionGyro(OFF_STONE_SPEED, 90,180, new Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "Driving on stone sats: S: %.2f",hsvValues[1]);
                if(hsvValues[1] < HSV_SAT_CUT_OFF_STONE){
                    sleep(750);
                    return true;
                }
                return false;
            }
        });

        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "turnToheadingGyro");

        turnToHeadingGyro(0,GLOBAL_STANDERD_TOLERANCE,GLOBAL_STANDERD_LATENCY,RotationDirection.COUNTER_CLOCK); //Turn to face the wall again. Now it should turn 180 to face the wall.

        //Drive towards the box till the colored tape is detected.
        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "driveDirectionGyro 2");

        driveDirectionGyro(DRIVE_TOWARDS_TRIANGLE_SPEED, 90, new Predicate() {
            @Override
            public boolean isTrue() { // I just changed the speed of this to see if we can get back over the balancing stone it used to be 20. Also if this works change all the other auto programs.
                Color.RGBToHSV(bot.colorLeft.red() * 8, bot.colorLeft.green() * 8, bot.colorLeft.blue() * 8, hsvValues);
                if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "Driving to line sats: S: %.2f",hsvValues[1]);
                if(hsvValues[1] > HSV_SAT_CUT_OFF){
                    return true;
                }

                return false;
            }
        });

        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "Checking pre line follow.");

        Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
        //Follow the line depending on how many times it has already been seen.
        if(hsvValues[1] < HSV_SAT_CUT_OFF){
            if (BLUE_BOTTOM_START_LOG)  BetaLog.dd(BLUE_BOTTOM_START_TAG, "Line following forward left.");
            followLineProportionate(LineFollowSide.LEFT, bot.colorLeft, LINE_FOLLOW_SPEED, new Predicate() {
                @Override
                public boolean isTrue() {
                    Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                    if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "Follow line forward sats: S: %.2f",hsvValues[1]);
                    if(hsvValues[1] > HSV_SAT_CUT_OFF)return true;
                    return false;
                }
            });
        }else{
            if (BLUE_BOTTOM_START_LOG)  BetaLog.dd(BLUE_BOTTOM_START_TAG, "Line following backwards left.");
            followLineProportionate(LineFollowSide.LEFT, bot.colorLeft, -LINE_FOLLOW_SPEED, new Predicate() {
                @Override
                public boolean isTrue() {
                    Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                    if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "Follow line backwards sats: S: %.2f",hsvValues[1]);
                    return (hsvValues[1] < HSV_SAT_CUT_OFF);
                }
            });
        }

        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "adjust on triangle");

        prepareToScoreGlyph(); //TODO
        scoreGylph(); //TODO

        /*
        adjustPosOnTriangle(ADJUST_POS_TIMEOUT);

        final float distanceFromCrptoBoxAfterAdjust = 30;
        robotZXPhi = new float[] {distanceFromCrptoBoxAfterAdjust,0,bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();

        switch (this.cryptoKey){
            case LEFT:
                if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "driveDirectionGyro left");
                driveDirectionGyro(10, -90, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return robotZXPhi[1] < -CRYPTO_BOX_SIDE_SHIFT_VALUE;
                    }
                });
                break;
            case RIGHT:
                if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "driveDirectionGyro right");
                driveDirectionGyro(10, 90, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return robotZXPhi[1] > CRYPTO_BOX_SIDE_SHIFT_VALUE;
                    }
                });
                break;
            case CENTER:
            case UNKNOWN:
        }

        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "driveDirectionGyro 3");

        driveDirectionGyro(10, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < CRYPTO_BOX_FORWARD_SHIFT_VALUE; //Z is at 30 robot cords here, we have to move forward now so lower Z.
            }
        });

        telemetry.addData("Auto data: ","Vumark target: " + cryptoKey + " target jewel side: " + targetSide);
        telemetry.update();
        scoreGylph();

        */
    }
}
