package org.firstinspires.ftc.teamcode.competition_in_work.auto;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;

/**
 * Created by FTC Team 8397 on 11/22/2017.
 */
@Autonomous(name="Blue  Top Comp Start",group = "Blue")
public class BlueTopStart extends MechBotAutonomous {
    int timesColorFoundSensorOne =0;
    final float[] hsvValues = new float[3];

    final String BLUE_TOP_START_TAG = "Blue top start:";
    final boolean BLUE_TOP_START_LOG = true;

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap,90); //Init the hardware map with a starting angle of 0.
        //The starting angle is the gyro heading relative to the crypto box.
        robotZXPhi = new float[3];

        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "initAuto");
        initAuto(TeamColor.BLUE, 2000,2000); //Find the targetJewl side and the target crypto key.
        BetaLog.dd(BLUE_TOP_START_TAG, "knockJewel");

        knockJewelWithBalanceTurn(this.targetSide);

        //Assume the robot is facing the wall once again still on the balance stone and the wall is a heading of 0.
        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "driveDirectionGyro 1");
        driveDirectionGyro(20, 180, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(bot.colorLeft.red() * 8, bot.colorLeft.green() * 8, bot.colorLeft.blue() * 8, hsvValues);
                if(hsvValues[1] < .5){
                    sleep(250);
                    return true;
                }
                return false;
            }
        });

        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "turnToheadingGyro");

        robotZXPhi = new float[]{0,0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        driveDirectionGyro(20, 180, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                telemetry.addData("","Z " + robotZXPhi[0] + " X" +robotZXPhi[1]);
                telemetry.update();
                return robotZXPhi[0] < -10;
            }
        });

        turnToHeadingGyro(0,2,0.3f); //Turn to face the wall again.

        //Drive towards the box till the colored tape is detected.
        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "driveDirectionGyro2");
        driveDirectionGyro(20, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(bot.colorLeft.red() * 8, bot.colorLeft.green() * 8, bot.colorLeft.blue() * 8, hsvValues);
                if(hsvValues[1] > .5){
                    return true;
                }

                return false;
            }
        });

        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "Checking pre line follow.");

        Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
        //Follow the line depending on how many times it has already been seen.
        if(hsvValues[1] < .5){
            if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "Line following forward left.");
            followLineProportionate(LineFollowSide.LEFT, bot.colorLeft, new Predicate() {
                @Override
                public boolean isTrue() {
                    Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                    if(hsvValues[1] > .5)return true;
                    return false;
                }
            });
        }else{
            if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "Line following backwards left.");
            followLineProportionate(LineFollowSide.LEFT, bot.colorLeft, -10, new Predicate() {
               @Override
               public boolean isTrue() {
                   Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                   return (hsvValues[1] < .5);
               }
           });
        }

        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "adjust on triangle");

        adjustPosOnTriangle(4000);


        //18.8 shift.
        robotZXPhi = new float[] {30,0,bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        switch (this.cryptoKey){
            case LEFT:
                if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "driveDirectionGyro left");
                driveDirectionGyro(10, -90, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return robotZXPhi[1] < -18;
                    }
                });
                break;
            case RIGHT:
                if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "driveDirectionGyro right");
                driveDirectionGyro(10, 90, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return robotZXPhi[1] > 18;
                    }
                });
                break;
            case CENTER:
            case UNKNOWN:
        }

        if (BLUE_TOP_START_LOG) BetaLog.dd(BLUE_TOP_START_TAG, "driveDirectionGyro3");
        driveDirectionGyro(10, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < 15;
            }
        });

        telemetry.addData("Auto data: ","Vumark target: " + cryptoKey + " target jewel side: " + targetSide);
        telemetry.update();
        while (opModeIsActive());
    }


}
