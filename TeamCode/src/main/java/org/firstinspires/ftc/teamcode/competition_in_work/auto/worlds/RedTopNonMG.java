package org.firstinspires.ftc.teamcode.competition_in_work.auto.worlds;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotAutonomousScranton;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

/**
 * Created by FTC Team 8397 on 4/22/2018.
 */
@Autonomous(name = "Red Top Non MG",group = "No MG")
public class RedTopNonMG extends MechBotAutonomousScranton {
    final float[] hsvValues = new float[3];

    final boolean BLUE_BOTTOM_START_LOG = true;
    final String BLUE_BOTTOM_START_TAG = "Blue bottom start Red Hook:";
    final float OFF_STONE_DISTANCE = -58.5f;
    final float MULTI_GLYPH_X_SHIFT = -14f * 2.54f;
    final float MULTI_GLYPH_Z_SHIFT = 31f * 2.54f;
    final float MULTI_GLYPH_DIR = (float)Math.atan(MULTI_GLYPH_X_SHIFT/MULTI_GLYPH_Z_SHIFT);


    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap, 90); //The starting value of the gyro heading comapred to the wall.

        //The starting angle is the gyro heading relative to the crypto box.
        robotZXPhi = new float[3];

        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(BLUE_BOTTOM_START_TAG, "INITIALIZE AUTO");

        //Contains wait for start.
        initAuto(TeamColor.RED, VUMARK_KEY_SCAN_TIME, JEWEL_SCAN_TIME); //Knocks the jewel off the stone and finds crypto key.
        telemetry.update();

        setOdometry(0,0);
        driveDirectionGyro(OFF_STONE_SPEED, 180, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < OFF_STONE_DISTANCE;
            }
        });

        setOdometry(0,0);

        turnToHeadingGyroQuick(180,GLOBAL_STANDERD_TOLERANCE * 2.0f,GLOBAL_STANDERD_LATENCY * 0.5f);

        setOdometry(0,0);
        driveDirectionGyro(50, 0, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 5;
            }
        });

        driveDirectionGyro(DRIVE_TOWARDS_TRIANGLE_SPEED, -90,180, new Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(bot.backColorRight.red() * 8, bot.backColorRight.green() * 8, bot.backColorRight.blue() * 8, hsvValues);
                return hsvValues[1] > HSV_SAT_CUT_OFF;
            }
        });


        handleTriangle(TriangleApproachSide.RIGHT,LINE_FOLLOW_SPEED,20,180,bot.backColorLeft,bot.backColorRight,0);

        freeFlipPlate();

        scoreGlyph(this.cryptoKey);

        VuMarkNavigator.deactivate();

    }
}
