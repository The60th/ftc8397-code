package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;

/**
 * Created by JimLori on 2/5/2018.
 */

@Autonomous(name="TestNudgeGlyph2", group="Test")
@Disabled

public class TestNudgeGlyph2 extends MechBotAutonomous {

    private enum Side{LEFT, RIGHT, UNKNOWN};

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap,0); //Init the hardware map with a starting angle of 0.
        //The starting angle is the gyro heading relative to the crypto box.
        robotZXPhi = new float[3];

        robotZXPhi = new float[] { 0,0,bot.getOdomHeadingFromGyroHeading(bot.getInitGyroHeadingRadians())};
        bot.updateOdometry();

        telemetry.addData("Press Start To Nudge Glyph", "");
        telemetry.update();

        waitForStart();

        robotZXPhi = new float[] { 0,0,bot.getOdomHeadingFromGyroHeading(bot.getInitGyroHeadingRadians())};
        driveDirectionGyro(20, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 1;
            }
        });

        robotZXPhi = new float[] { 0,0,bot.getOdomHeadingFromGyroHeading(bot.getInitGyroHeadingRadians())};
        driveDirectionGyro(20, 0, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[1] > 3;
            }
        });

        robotZXPhi = new float[] { 0,0,bot.getOdomHeadingFromGyroHeading(bot.getInitGyroHeadingRadians())};
        driveDirectionGyro(20, 0, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < -4;
            }
        });

        robotZXPhi = new float[] { 0,0,bot.getOdomHeadingFromGyroHeading(bot.getInitGyroHeadingRadians())};
        driveDirectionGyro(20, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 2;
                }
            });

        robotZXPhi = new float[] { 0,0,bot.getOdomHeadingFromGyroHeading(bot.getInitGyroHeadingRadians())};
        driveDirectionGyro(20, 0, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[1] < -6;
            }
        });

        robotZXPhi = new float[] { 0,0,bot.getOdomHeadingFromGyroHeading(bot.getInitGyroHeadingRadians())};
        driveDirectionGyro(20, 0, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < -6;
            }
        });

        robotZXPhi = new float[] { 0,0,bot.getOdomHeadingFromGyroHeading(bot.getInitGyroHeadingRadians())};
        driveDirectionGyro(20, 0, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 4;
            }
        });

        telemetry.addData("FINISHED","");
        telemetry.update();

    }

    private void nudgeGlyph(Side side, float speed, long millisec){

        if (side == Side.UNKNOWN) return;

        final float ROBOT_CENTER_TO_GLYPH_EDGE = (9.0f + 6.0f) * 2.54f; //cm from robot center to front edge of glyph
        final float HALF_GLYPH_WIDTH = 6.0f * 2.54f; //half of glyph width, in cm
        //cm from robot center to one of the leading edges (left or right) of the glyph
        final float ROBOT_CENTER_TO_GLYPH_CORNER =
                (float)Math.sqrt(ROBOT_CENTER_TO_GLYPH_EDGE*ROBOT_CENTER_TO_GLYPH_EDGE + HALF_GLYPH_WIDTH*HALF_GLYPH_WIDTH);

        float vx = Math.abs(speed) * HALF_GLYPH_WIDTH / ROBOT_CENTER_TO_GLYPH_CORNER;

        float vy, va;

        if (side == Side.RIGHT){
            vy = -Math.abs(speed) * ROBOT_CENTER_TO_GLYPH_EDGE / ROBOT_CENTER_TO_GLYPH_CORNER;
            va = Math.abs(speed) / ROBOT_CENTER_TO_GLYPH_CORNER;
        }
        else{   //side is Side.LEFT
            vy = Math.abs(speed) * ROBOT_CENTER_TO_GLYPH_EDGE / ROBOT_CENTER_TO_GLYPH_CORNER;
            va = -Math.abs(speed) / ROBOT_CENTER_TO_GLYPH_CORNER;
        }

        bot.setDriveSpeed(vx, vy, va);
        sleep(millisec);
        bot.setDriveSpeed(0,0,0);
    }

}
