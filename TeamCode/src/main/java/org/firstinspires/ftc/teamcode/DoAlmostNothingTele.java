package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by FTC Team 8397 on 10/27/2017.
 */
@TeleOp(name = "Do Almost Nothing Tele", group = "Test")
public class DoAlmostNothingTele extends LinearOpMode {
    MechBotSensor bot = new MechBotSensor();

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                bot.one.setPower(.4);
            } else {
                bot.one.setPower(0);
            }

            if (gamepad1.x) {
                bot.two.setPower(.4);
            } else {
                bot.two.setPower(0);
            }

            if (gamepad1.y) {
                bot.three.setPower(.4);
            } else {
                bot.three.setPower(0);
            }

            if (gamepad1.b) {
                bot.four.setPower(.4);
            } else {
                bot.four.setPower(0);
            }

        }
    }
}
