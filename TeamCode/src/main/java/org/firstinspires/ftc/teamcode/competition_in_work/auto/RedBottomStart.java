package org.firstinspires.ftc.teamcode.competition_in_work.auto;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;

/**
 * Created by FTC Team 8397 on 11/22/2017.
 */
@Autonomous(name="Red Bottom Comp Start",group = "Comp")
public class RedBottomStart extends MechBotAutonomous {
    final float[] hsvValues = new float[3];
    final String STARTING_BETA_LOG = "Red bottom start:";
    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap,0); //Init the hardware map with a starting angle of 0.
        //The starting angle is the gyro heading relative to the crypto box.
        robotZXPhi = new float[3];
        BetaLog.dd(STARTING_BETA_LOG, "initAuto");
        initAuto(TeamColor.RED, 2000,2000); //Find the targetJewl side and the target crypto key.
        BetaLog.dd(STARTING_BETA_LOG, "knockJewel");
        knockJewel(this.targetSide);

        //Assume the robot is facing the wall once again still on the balance stone and the wall is a heading of 0.
        BetaLog.dd(STARTING_BETA_LOG, "driveDirectionGyro1");
        driveDirectionGyro(20, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                if(hsvValues[1] < .5){
                    sleep(250);
                    return true;
                }
                return false;
            }
        });
        BetaLog.dd(STARTING_BETA_LOG, "turnToheadingGyro");

        turnToHeadingGyro(0,2,100); //Turn to face the wall again.

        //Drive towards the box till the colored tape is detected.
        BetaLog.dd(STARTING_BETA_LOG, "driveDirectionGyro2");
        driveDirectionGyro(20, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                if(hsvValues[1] > .5){
                    return true;
                }

                return false;
            }
        });

        BetaLog.dd(STARTING_BETA_LOG, "Checking pre line follow.");

        Color.RGBToHSV(bot.colorLeft.red() * 8, bot.colorLeft.green() * 8, bot.colorLeft.blue() * 8, hsvValues);
        //Follow the line depending on how many times it has already been seen.
        if(hsvValues[1] < .5){
            BetaLog.dd(STARTING_BETA_LOG, "Line following forward left.");
            followLineProportionate(LineFollowSide.RIGHT, bot.colorRight, new Predicate() {
                @Override
                public boolean isTrue() {
                    Color.RGBToHSV(bot.colorLeft.red() * 8, bot.colorLeft.green() * 8, bot.colorLeft.blue() * 8, hsvValues);
                    if(hsvValues[1] > .5)return true;
                    return false;
                }
            });
        }else{
            BetaLog.dd(STARTING_BETA_LOG, "Line following backwards left.");
            //Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
            followLineProportionate(LineFollowSide.RIGHT, bot.colorRight, -10, new Predicate() {
               @Override
               public boolean isTrue() {
                   Color.RGBToHSV(bot.colorLeft.red() * 8, bot.colorLeft.green() * 8, bot.colorLeft.blue() * 8, hsvValues);
                   return (hsvValues[1] < .5);
               }
           });
        }

        BetaLog.dd(STARTING_BETA_LOG, "adjust on triangle");

        adjustPosOnTriangle(1000);


        //18.8 shift.
        robotZXPhi = new float[] {30,0,bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        switch (this.cryptoKey){
            case LEFT:
                BetaLog.dd(STARTING_BETA_LOG, "driveDirectionGyro left");
                driveDirectionGyro(10, -90, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return robotZXPhi[1] < -18;
                    }
                });
                break;
            case RIGHT:
                BetaLog.dd(STARTING_BETA_LOG, "driveDirectionGyro right");
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

        BetaLog.dd(STARTING_BETA_LOG, "driveDirectionGyro3");
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
