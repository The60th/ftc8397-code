package org.firstinspires.ftc.teamcode.competition_in_work.auto.worlds;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotAutonomousScranton;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

/**
 * Created by FTC Team 8397 on 4/17/2018.
 */
@Autonomous(name = "Red Bottom MG Test", group = "Test")
public class RedBottomMGTest extends MechBotAutonomousScranton {
    final float[] hsvValues = new float[3];


    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap, 180); //The starting value of the gyro heading comapred to the wall.

        //The starting angle is the gyro heading relative to the crypto box.
        robotZXPhi = new float[3];

        waitForStart();

        setOdometry(0,0);

        driveDirectionGyro(5000, 0, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 60;
            }
        });

        driveDirectionGyro(50, 180, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < 55;
            }
        });

        bot.setIntakeOn();

        driveDirectionGyro(10, 0, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 65;
            }
        });

        driveDirectionGyro(50, 180, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < 60;
            }
        });

        sleep(500);

        bot.setIntakeReverse();

        sleep(500);

        final ElapsedTime et = new ElapsedTime();
        bot.setIntakeOn();

        driveDirectionGyro(5, 0, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                if(et.milliseconds() > 1500){
                    bot.setIntakeOff();
                    return true;
                }
                return false;
            }
        });

        VuMarkNavigator.deactivate();

    }
}
