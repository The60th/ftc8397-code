package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.mechbot.MechBotRedHook;

/**
 * Created by FTC Team 8397 on 1/10/2018.
 */
@TeleOp(group = "ServoFunctions", name = "ServoFunctions")
@Disabled
public class ServoFunctions extends LinearOpMode {
    MechBotRedHook bot = new MechBotRedHook();
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()){
            if(gamepad1.a){
                bot.leftUpperClamp.setPosition(.70); //Left upper all the way back. 1
            }else if(gamepad1.b){
                bot.rightUpperClamp.setPosition(.53);  //Right upper all the way in. 1
            }else if(gamepad1.x){
                bot.leftUpperClamp.setPosition(.37); //Left upper all the way in. 0
            }else if(gamepad1.y){
                bot.rightUpperClamp.setPosition(.20); //Right upper all the way out. 0
            }else if(gamepad1.dpad_up){
                bot.rightLowerClamp.setPosition(1); //Right lower all the way out. 1
            }else if(gamepad1.dpad_right) {
                bot.leftLowerClamp.setPosition(.90);  //Left lower all the way in. 1
            }else if(gamepad1.dpad_down){
                bot.rightLowerClamp.setPosition(.10); //Right lower all the way in. 0
            }else if(gamepad1.dpad_left){
                bot.leftLowerClamp.setPosition(0); //Left lower all the way out. 0
            }

            if (gamepad1.right_trigger > .05){
                bot.liftArmUp();
            }else if (gamepad1.left_trigger > .05){
                bot.liftArmDown();
            }else {
                bot.liftArmStop();
            }

        }
    }
}
