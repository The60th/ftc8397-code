package org.firstinspires.ftc.teamcode.competition_in_work.auto;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;

/**
 * Created by FTC Team 8397 on 11/22/2017.
 */
@Autonomous(name="Red Top",group = "comp")
@Disabled
public class RedTop_RedHook extends MechBotAutonomous {
    final float[] hsvValues = new float[3];

    final String RED_START_TOP_TAG = "RED_TOP_START";
    final boolean RED_START_TOP_LOG = true;

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap,90); //Init the hardware map with a starting angle of 0.
        //The starting angle is the gyro heading relative to the crypto box.
        robotZXPhi = new float[3];

        if (RED_START_TOP_LOG) BetaLog.dd(RED_START_TOP_TAG, "initAuto");
        initAuto(TeamColor.RED,  VUMARK_KEY_SCAN_TIME,JEWEL_SCAN_TIME); //Find the targetJewl side and the target crypto key.

        if (RED_START_TOP_LOG) BetaLog.dd(RED_START_TOP_TAG, "knockJewel");
        knockJewelWithBalanceTurn(this.targetSide);

        //Assume the robot is facing the wall once again still on the balance stone and the wall is a heading of 0.
        if (RED_START_TOP_LOG) BetaLog.dd(RED_START_TOP_TAG, "driveDirectionGyro1");
        driveDirectionGyro(20, 180, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                if(hsvValues[1] < HSV_SAT_CUT_OFF){
                    sleep(250);
                    return true;
                }
                return false;
            }
        });

        if (RED_START_TOP_LOG) BetaLog.dd(RED_START_TOP_TAG, "turnToheadingGyro");

        robotZXPhi = new float[]{0,0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        driveDirectionGyro(20, 180, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < -10;
            }
        });

        turnToHeadingGyro(0,GLOBAL_STANDERD_TOLERANCE,GLOBAL_STANDERD_LATENCY); //Turn to face the wall again.

        //Drive towards the box till the colored tape is detected.

        if (RED_START_TOP_LOG) BetaLog.dd(RED_START_TOP_TAG, "driveDirectionGyro2");

        driveDirectionGyro(25, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                if(hsvValues[1] > HSV_SAT_CUT_OFF){
                    return true;
                }

                return false;
            }
        });

        if (RED_START_TOP_LOG) BetaLog.dd(RED_START_TOP_TAG, "Checking pre line follow.");

        Color.RGBToHSV(bot.colorLeft.red() * 8, bot.colorLeft.green() * 8, bot.colorLeft.blue() * 8, hsvValues);
        //Follow the line depending on how many times it has already been seen.
        if(hsvValues[1] < HSV_SAT_CUT_OFF){
            if (RED_START_TOP_LOG) BetaLog.dd(RED_START_TOP_TAG, "Line following forward left.");
            followLineProportionate(LineFollowSide.RIGHT, bot.colorRight, new Predicate() {
                @Override
                public boolean isTrue() {
                    Color.RGBToHSV(bot.colorLeft.red() * 8, bot.colorLeft.green() * 8, bot.colorLeft.blue() * 8, hsvValues);
                    if(hsvValues[1] > HSV_SAT_CUT_OFF)return true;
                    return false;
                }
            });
        }else{
            if (RED_START_TOP_LOG) BetaLog.dd(RED_START_TOP_TAG, "Line following backwards left.");
            followLineProportionate(LineFollowSide.RIGHT, bot.colorRight, -10, new Predicate() {
                @Override
                public boolean isTrue() {
                    Color.RGBToHSV(bot.colorLeft.red() * 8, bot.colorLeft.green() * 8, bot.colorLeft.blue() * 8, hsvValues);
                    return (hsvValues[1] < HSV_SAT_CUT_OFF);
                }
            });
        }

        if (RED_START_TOP_LOG) BetaLog.dd(RED_START_TOP_TAG, "adjust on triangle");

        adjustPosOnTriangle(ADUST_POS_TIMEOUT);


        //18.8 shift.
        robotZXPhi = new float[] {30,0,bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        switch (this.cryptoKey){
            case LEFT:
                if (RED_START_TOP_LOG) BetaLog.dd(RED_START_TOP_TAG, "driveDirectionGyro left");
                driveDirectionGyro(10, -90, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return robotZXPhi[1] < -CRYPTO_BOX_SIDE_SHIFT_VALUE;
                    }
                });
                break;
            case RIGHT:
                if (RED_START_TOP_LOG) BetaLog.dd(RED_START_TOP_TAG, "driveDirectionGyro right");
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

        BetaLog.dd(RED_START_TOP_TAG, "driveDirectionGyro3");
        driveDirectionGyro(10, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < CRYPTO_BOX_FOWARD_SHIFT_VALUE;
            }
        });

        telemetry.addData("Auto data: ","Vumark target: " + cryptoKey + " target jewel side: " + targetSide);
        telemetry.update();

        //scoreGlyph();
    }
}
