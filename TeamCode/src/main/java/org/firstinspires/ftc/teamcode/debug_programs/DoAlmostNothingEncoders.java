package org.firstinspires.ftc.teamcode.debug_programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;

/**
 * Created by FTC Team 8397 on 10/27/2017.
 */
@Autonomous( name= "Do Almost Nothing Encoders", group = "Test")
public class DoAlmostNothingEncoders extends MechBotAutonomous {
    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            bot.updateOdometry();
            telemetry.addData("Encoder Ticks"," %.0f %.0f %.0f %.0f", bot.last_Wheel_Ticks.get(0), bot.last_Wheel_Ticks.get(1),
                    bot.last_Wheel_Ticks.get(2), bot.last_Wheel_Ticks.get(3));
            telemetry.update();
        }
    }
}
