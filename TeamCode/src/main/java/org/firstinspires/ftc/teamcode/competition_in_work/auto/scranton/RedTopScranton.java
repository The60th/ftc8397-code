package org.firstinspires.ftc.teamcode.competition_in_work.auto.scranton;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotAutonomousScranton;

/**
 * Created by FTC Team 8397 on 3/6/2018.
 */
@Autonomous(name = "Red Top", group = "Auto")
@Disabled
public class RedTopScranton extends MechBotAutonomousScranton {

    final float[] hsvValues = new float[3];

    final boolean BLUE_BOTTOM_START_LOG = true;
    final String RED_BOTTOM_START_TAG = "Red Top start Red Hook:";

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap, 180); //The starting value of the gyro heading comapred to the wall.

        //The starting angle is the gyro heading relative to the crypto box.
        robotZXPhi = new float[3];

        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(RED_BOTTOM_START_TAG, "INITIALIZE AUTO");

        //Contains wait for start.
        initAuto(MechBotAutonomousScranton.TeamColor.RED, VUMARK_KEY_SCAN_TIME, JEWEL_SCAN_TIME); //Knocks the jewel off the stone and finds crypto key.

        //Assume the robot is facing the wall once again still on the balance stone and the wall is a heading of 0.
        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(RED_BOTTOM_START_TAG, "driveDirectionGyro 1");
        //added the 180 to this line of code to keep the robot from turning around.
        driveDirectionGyro(OFF_STONE_SPEED, 180, 180, new MechBotAutonomousScranton.Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);

                if (BLUE_BOTTOM_START_LOG)
                    BetaLog.dd(RED_BOTTOM_START_TAG, "Driving on stone sats: S: %.2f", hsvValues[1]);

                if (hsvValues[1] < HSV_SAT_CUT_OFF_STONE) {
                    //Color sensors are off the stone.
                    return true;
                }
                return false;
            }
        });
        setOdometry(0,0);
        //Robot is now partly off the stone. Just the front color sensors are off, time to drive the rest of the robot off the stone.
        driveDirectionGyro(OFF_STONE_SPEED, 180, 180, new MechBotAutonomousScranton.Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < -5; //Need a constant defined here.
            }
        });

        //Robot is now all the way off the balance stone and ready to turn towards the crypto box.
        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(RED_BOTTOM_START_TAG, "turnToheadingGyro");


        turnToHeadingGyro(0, GLOBAL_STANDERD_TOLERANCE, GLOBAL_STANDERD_LATENCY, RotationDirection.CLOCK); //Turn to face the wall.


        setOdometry(0,0);
        driveDirectionGyro(20, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 10;
            }
        });


        driveDirectionGyro(DRIVE_TOWARDS_TRIANGLE_SPEED, -90, new Predicate() {
            @Override
            public boolean isTrue() { // I just changed the speed of this to see if we can get back over the balancing stone it used to be 20. Also if this works change all the other auto programs.
                Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                if (BLUE_BOTTOM_START_LOG)
                    BetaLog.dd(RED_BOTTOM_START_TAG, "Driving to line sats: S: %.2f", hsvValues[1]);
                if (hsvValues[1] > HSV_SAT_CUT_OFF) {
                    return true;
                }

                return false;
            }
        });

        //while (opModeIsActive()){
        //This stops the robot with both color sensors on the line.
        //}


        handleTriangle(TriangleApproachSide.RIGHT, TriangleMode.OUTSIDE);

//        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
//        driveDirectionGyro(10, 180, new Predicate() {
//            @Override
//            public boolean isTrue() {
//                return robotZXPhi[0] < -5f;
//            }
//        });
//
//
//        //Drive towards the box till the colored tape is detected.
//        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(RED_BOTTOM_START_TAG, "driveDirectionGyro 2");
//
//        driveDirectionGyro(DRIVE_TOWARDS_TRIANGLE_SPEED, -90, new MechBotAutonomousScranton.Predicate() {
//            @Override
//            public boolean isTrue() { // I just changed the speed of this to see if we can get back over the balancing stone it used to be 20. Also if this works change all the other auto programs.
//                Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
//                if (BLUE_BOTTOM_START_LOG)
//                    BetaLog.dd(RED_BOTTOM_START_TAG, "Driving to line sats: S: %.2f", hsvValues[1]);
//                if (hsvValues[1] > HSV_SAT_CUT_OFF) {
//                    return true;
//                }
//
//                return false;
//            }
//        });
//
//
//        driveDirectionGyro(DRIVE_TOWARDS_TRIANGLE_SPEED, -90, new MechBotAutonomousScranton.Predicate() {
//            @Override
//            public boolean isTrue() { // I just changed the speed of this to see if we can get back over the balancing stone it used to be 20. Also if this works change all the other auto programs.
//                Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
//                if (BLUE_BOTTOM_START_LOG)
//                    BetaLog.dd(RED_BOTTOM_START_TAG, "Driving to line sats: S: %.2f", hsvValues[1]);
//                if (hsvValues[1] < HSV_SAT_CUT_OFF) {
//                    return true;
//                }
//
//                return false;
//            }
//        });
//
//        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(RED_BOTTOM_START_TAG, "Checking pre line follow.");
//
//        Color.RGBToHSV(bot.colorLeft.red() * 8, bot.colorLeft.green() * 8, bot.colorLeft.blue() * 8, hsvValues);
//        //Follow the line depending on how many times it has already been seen.
//        if (hsvValues[1] < HSV_SAT_CUT_OFF) {
//            quickTelemetry("FOLLOW LINE REVERSE");
//            followLineProportionate(LineFollowSide.LEFT, -INNER_TAPE_ANGLE_RADS, bot.colorRight, -LINE_FOLLOW_SPEED, new Predicate() {
//                @Override
//                public boolean isTrue() {
//                    Color.RGBToHSV(bot.colorLeft.red() * 8, bot.colorLeft.green() * 8, bot.colorLeft.blue() * 8, hsvValues);
//                    return hsvValues[1] > HSV_SAT_CUT_OFF;
//                }
//            });
//
//
//        } else {
//            quickTelemetry("FOLLOW LINE FORWARD");
//            followLineProportionate(LineFollowSide.LEFT, -INNER_TAPE_ANGLE_RADS, bot.colorRight, LINE_FOLLOW_SPEED, new Predicate() {
//                @Override
//                public boolean isTrue() {
//                    Color.RGBToHSV(bot.colorLeft.red() * 8, bot.colorLeft.green() * 8, bot.colorLeft.blue() * 8, hsvValues);
//                    return hsvValues[1] < HSV_SAT_CUT_OFF;
//                }
//            });
//        }
//        adjustPosInsideTriangle(ADJUST_POS_TIMEOUT);
//
//        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
//
//        driveDirectionGyro(DRIVE_TOWARDS_TRIANGLE_SPEED, 0, new Predicate() {
//            @Override
//            public boolean isTrue() { // I just changed the speed of this to see if we can get back over the balancing stone it used to be 20. Also if this works change all the other auto programs.
//                return robotZXPhi[0] > 8;
//            }
//        });
//
//        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(RED_BOTTOM_START_TAG, "adjust on triangle");

        scoreGlyph(this.cryptoKey);

    }
}
