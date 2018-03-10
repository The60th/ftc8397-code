package org.firstinspires.ftc.teamcode.competition_in_work.auto;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;

/**
 * Created by FTC Team 8397 on 12/8/2017.
 */
@Autonomous(name = "Blue Top", group = "comp")
@Disabled
public class BlueTop_RedHook extends MechBotAutonomous {
    final float[] hsvValues = new float[3];

    final String BLUE_TOP_START_TAG = "Blue top start:";
    final boolean BLUE_TOP_START_LOG = true;

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap, -90); //Init the hardware map with a starting angle of 0. //Unsure on what this value you should be.
        //The starting angle is the gyro heading relative to the crypto box.
        robotZXPhi = new float[3];

        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "initAuto");
        initAuto(TeamColor.BLUE, VUMARK_KEY_SCAN_TIME, JEWEL_SCAN_TIME); //Find the targetJewl side and the target crypto key.
        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "knockJewel");

        //Assume the robot is facing the wall once again still on the balance stone and the wall is a heading of 0.
        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "driveDirectionGyro 1");
        driveDirectionGyro(OFF_STONE_SPEED, 180, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                if (hsvValues[1] < HSV_SAT_CUT_OFF_STONE) {
                    sleep(250);
                    return true;
                }
                return false;
            }
        });

        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "turnToheadingGyro");

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        driveDirectionGyro(20, 180, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < -4; //was -6 on 1/26/18
            }
        });

        //This value needs testing.
        turnToHeadingGyro(0, GLOBAL_STANDERD_TOLERANCE, GLOBAL_STANDERD_LATENCY); //Turn to face the wall again.

        //Drive towards the box till the colored tape is detected.

        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "driveDirectionGyro2");

        driveDirectionGyro(DRIVE_TOWARDS_TRIANGLE_SPEED, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(bot.colorLeft.red() * 8, bot.colorLeft.green() * 8, bot.colorLeft.blue() * 8, hsvValues);
                if (hsvValues[1] > HSV_SAT_CUT_OFF) {
                    return true;
                }

                return false;
            }
        });

        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "Checking pre line follow.");

        Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
        //Follow the line depending on how many times it has already been seen.
        if (hsvValues[1] < HSV_SAT_CUT_OFF) {
            if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "Line following forward left.");
            followLineProportionate(LineFollowSide.LEFT, bot.colorLeft, LINE_FOLLOW_SPEED, new Predicate() {
                @Override
                public boolean isTrue() {
                    Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                    if (hsvValues[1] > HSV_SAT_CUT_OFF) return true;
                    return false;
                }
            });
        } else {
            if (BLUE_TOP_START_LOG)
                BetaLog.dd(BLUE_TOP_START_TAG, "Line following backwards left.");
            followLineProportionate(LineFollowSide.LEFT, bot.colorLeft, -LINE_FOLLOW_SPEED, new Predicate() {
                @Override
                public boolean isTrue() {
                    Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                    return (hsvValues[1] < HSV_SAT_CUT_OFF);
                }
            });
        }

        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "adjust on triangle");


        prepareToScoreGlyph();
        scoreGylph();
        /*adjustPosOnTriangle(ADJUST_POS_TIMEOUT);
        final float distanceFromCrptoBoxAfterAdjust = 30;
        robotZXPhi = new float[] {distanceFromCrptoBoxAfterAdjust,0,bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();


        switch (this.cryptoKey){
            case LEFT:
                if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "driveDirectionGyro left");
                driveDirectionGyro(10, -90, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return robotZXPhi[1] < -CRYPTO_BOX_SIDE_SHIFT_VALUE;
                    }
                });
                break;
            case RIGHT:
                if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "driveDirectionGyro right");
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

        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "driveDirectionGyro 3");

        driveDirectionGyro(10, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < CRYPTO_BOX_FORWARD_SHIFT_VALUE; //Z is at 30 robot cords here, we have to move forward now so lower Z.
            }
        });

        telemetry.addData("Auto data: ","Vumark target: " + cryptoKey + " target jewel side: " + targetSide);
        telemetry.update();

        scoreGylph();*/
    }
}

