package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;

/**
 * Created by FTC Team 8397 on 11/30/2017.
 */
//@Autonomous(name="encoder test debug",group = "rev")
public class TestEncoder extends MechBotAutonomous {
    @Override
    public void runLoggingOpmode() throws InterruptedException {
        //The robot Z encoder ticks do not seem to count up at all here.

        bot.init(hardwareMap,90); //Init the hardware map with a starting angle of 0.
        //The starting angle is the gyro heading relative to the crypto box.
        robotZXPhi = new float[3];
        waitForStart();
        bot.updateOdometry();
        robotZXPhi = new float[]{0,0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        driveDirectionGyro(-20, 90+bot.getInitGyroHeadingDegrees(),bot.getInitGyroHeadingDegrees(), new Predicate() {
            ElapsedTime et = new ElapsedTime();
            @Override
            public boolean isTrue() {
                telemetry.addData("","Z " + robotZXPhi[0] + " X" +robotZXPhi[1]);
                telemetry.update();
                return robotZXPhi[1] < -40;

                // return et.milliseconds() > 500;
            }
        });



        //The robot Z encoder ticks count up from here.
        bot.init(hardwareMap);
        robotZXPhi = new float[3];
        bot.updateOdometry();
        driveDirectionGyro(-20, 90+bot.getInitGyroHeadingDegrees(),bot.getInitGyroHeadingDegrees(), new Predicate() {
            ElapsedTime et = new ElapsedTime();
            @Override
            public boolean isTrue() {
                telemetry.addData("","Z " + robotZXPhi[0] + " X" +robotZXPhi[1]);
                telemetry.update();
                return robotZXPhi[1] < -40;

                // return et.milliseconds() > 500;
            }
        });
    }
}
