package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;
import org.firstinspires.ftc.teamcode.mechbot.MechBotRedHook;

/**
 * Created by JimLori on 2/5/2018.
 */

@Autonomous(name="TestTuckInGlyph", group="Test")
@Disabled


public class TestTuckInGlyph extends MechBotAutonomous {


    @Override
    public void runLoggingOpmode() throws InterruptedException {

        bot.init(hardwareMap);


        telemetry.addData("Press Start To Nudge Glyph", "");
        telemetry.update();

        waitForStart();

        tuckInGlyph();

        telemetry.addData("FINISHED","");
        telemetry.update();

    }


    private void tuckInGlyph(){
        final float TUCK_SPEED = 20;
        final float BACKUP_1 = 1;
        final float RIGHT = 5;
        final float FORWARD_1 = 3;
        final float BACKUP_2 = 5;
        final float LEFT = 10;
        final float FORWARD_2 = 8;

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        driveDirectionGyro(TUCK_SPEED, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > BACKUP_1;
            }
        });

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        driveDirectionGyro(TUCK_SPEED, 90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[1] > RIGHT;
            }
        });

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        driveDirectionGyro(TUCK_SPEED, 180, 20, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < -FORWARD_1;
            }
        });

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        driveDirectionGyro(TUCK_SPEED, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > BACKUP_2;
            }
        });

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        driveDirectionGyro(TUCK_SPEED, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[1] < -LEFT;
            }
        });

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        driveDirectionGyro(TUCK_SPEED, 180, -20, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < -FORWARD_2;
            }
        });

    }

}
