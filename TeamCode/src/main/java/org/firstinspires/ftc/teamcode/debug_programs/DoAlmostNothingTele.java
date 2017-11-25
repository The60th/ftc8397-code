package org.firstinspires.ftc.teamcode.debug_programs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechbot.MechBotSensor;

/**
 * Created by FTC Team 8397 on 10/27/2017.
 */
@TeleOp(name = "Do Almost Nothing Tele", group = "Test")
public class DoAlmostNothingTele extends LinearOpMode {
    MechBotSensor bot = new MechBotSensor();

    @Override
    public void runOpMode() throws InterruptedException {
        //Right left is considered from looking down at the robot with the rev mods on the right side of it facing forwards.
        bot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.y) { //Drives right.
                bot.setDriveSpeed(30, 0, 0);
            } else if (gamepad1.a) { //Drives left.
                bot.setDriveSpeed(-30, 0, 0);
            } else if (gamepad1.b) { //Drives forwards
                bot.setDriveSpeed(0, 30, 0);
            } else if (gamepad1.x) { //Drives backwards
                bot.setDriveSpeed(0, -30, 0);
            }else if(gamepad1.right_bumper){//Spins right.
                bot.setDriveSpeed(0, 0, 10);
            }else if(gamepad1.left_bumper){ //Spins left
                bot.setDriveSpeed(0, 0, -10);
            }else{
                bot.setDriveSpeed(0,0,0);
            }

        }
    }
}
