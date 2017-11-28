package org.firstinspires.ftc.teamcode.debug_programs;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;
import org.firstinspires.ftc.teamcode.mechbot.MechBotSensor;

/**
 * Created by FTC Team 8397 on 10/27/2017.
 */
@Autonomous( name= "Do Almost Nothing line follow", group = "Test")
public class DoAlmostNothingLineFollow extends MechBotAutonomous {
    @Override
    public void runLoggingOpmode(){
        bot.init(hardwareMap);
        waitForStart();

        followLineProportionate(LineFollowSide.LEFT, bot.sensorMRColor, new Predicate() {
            @Override
            public boolean isTrue() {
                float[] hsvValues = new float[3];
                Color.RGBToHSV(bot.sensorMRColor2.red() * 8, bot.sensorMRColor2.green() * 8, bot.sensorMRColor2.blue() * 8, hsvValues);
                if(hsvValues[1] > 0.5){
                    return true;
                }
                return false;
            }
        });
    }
}

