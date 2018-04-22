package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechbot.presupers_bot.MechBotRedHook;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotScranton;
import org.firstinspires.ftc.teamcode.third_party_libs.UTILToggle;

/**
 * Created by FTC Team 8397 on 1/26/2018.
 */
@TeleOp(name = "Flipper Tester", group = "Tester")
//@Disabled
public class FlipperTester extends LinearOpMode {
    private MechBotScranton bot = new MechBotScranton();
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        waitForStart();

        ElapsedTime et = new ElapsedTime();

        while (opModeIsActive()){
           if (et.milliseconds() < 50){
               continue;
           }
           et.reset();

            /*if (gamepad1.y){
                bot.setFlipPosition(bot.flipPlateUpticks);
            }
            else if (gamepad1.a){
                bot.setFlipPosition(bot.flipPlateDownticks);
            }*/
        }
    }
}
