package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.MechBotRedHook;

/**
 * Created by FTC Team 8397 on 1/23/2018.
 */
@TeleOp(name = "TestLinearSlideMotors", group = "Test")
public class TestLinearSlideMotors extends LoggingLinearOpMode {

    MechBotRedHook bot = new MechBotRedHook();

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            float speed = -gamepad1.left_stick_y;

           if (gamepad1.a){
               bot.leftLinearSlide.setPower(speed);
               bot.rightLinearSlide.setPower(0);
               telemetry.addData("Left Power = ", speed);
           }
           else if (gamepad1.b){
               bot.leftLinearSlide.setPower(0);
               bot.rightLinearSlide.setPower(speed);
               telemetry.addData("Right Power = ", speed);
           }
           else{
               bot.leftLinearSlide.setPower(0);
               bot.rightLinearSlide.setPower(0);
           }

           telemetry.addData("Encoders", "Left = %d  Right = %d", bot.leftLinearSlide.getCurrentPosition(), bot.rightLinearSlide.getCurrentPosition());

            telemetry.update();
        }
    }

}
