package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotScranton;

/**
 * Created by FTC Team 8397 on 2/28/2018.
 */
@Disabled
@TeleOp(name = "New Jewel Tester", group = "Rev")
public class NewJewelArmTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MechBotScranton bot = new MechBotScranton();

        bot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){

            if(gamepad1.y){
                bot.setArmCube();
            }else if(gamepad1.a){
                bot.setArmJewel();
            }

            if(gamepad1.x){
                bot.setPivotStart();
            }else if(gamepad1.b){
                bot.setPivotEnd();
            }else if(gamepad1.dpad_left){
                bot.knockPivotLeft();
            }else if(gamepad1.dpad_right){
                bot.knockPivotRight();
            }
        }
    }
}
