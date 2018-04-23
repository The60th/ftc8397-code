package org.firstinspires.ftc.teamcode.debug_programs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotScranton;

/**
 * Created by FTC Team 8397 on 4/13/2018.
 */
@TeleOp (name = "TestFLipMotor", group = "Test")
@Disabled
public class TestFlipMotor extends LoggingLinearOpMode {

    MechBotScranton bot = new MechBotScranton();

    final float COEFF = 1.0f/420.0f;

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap);

        bot.flipMotor.setPower(0);
        waitForStart();

        float targetTicks = 0;
        ElapsedTime et = new ElapsedTime();
        while(opModeIsActive()){

            if (et.milliseconds() < 30) continue;
            et.reset();

            if(gamepad1.dpad_up){
                targetTicks = targetTicks + 10;
            }
            else if (gamepad1.dpad_down){
                targetTicks = targetTicks - 10;
            }
            float ticks = bot.flipMotor.getCurrentPosition();
            float tickError = ticks - targetTicks;

            float power = -tickError * COEFF;
            power = Math.min(power,1.0f);
            power = Math.max(power,-1.0f);

            bot.flipMotor.setPower(power);

            telemetry.addData("ticks", ticks);
            telemetry.addData("targetTicks",targetTicks);
            telemetry.addData("power",power);
            telemetry.update();
            // 0 - 610 is working range
        }
    }
}

