package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.presupers_bot.MechBotRedHook;

/**
 * Created by JimLori on 1/10/2018.
 */
//@TeleOp(name = "MechBotServoTest", group = "Test")
public class MechBotServoTest extends LoggingLinearOpMode {

    MechBotRedHook bot = new MechBotRedHook();
    double leftLowerPos = 0;
    double leftUpperPos = 0;
    double rightLowerPos = 0;
    double rightUpperPos = 0;

    @Override
    public void runLoggingOpmode() throws InterruptedException {

        bot.init(hardwareMap);

        waitForStart();

        telemetry.addData("Wait For Initialization","");

        bot.liftArmUp();
        sleep(1000);
        bot.liftArmStop();

        bot.leftLowerClamp.setPosition(leftLowerPos);
        bot.leftUpperClamp.setPosition(leftUpperPos);
        bot.rightLowerClamp.setPosition(rightLowerPos);
        bot.rightUpperClamp.setPosition(rightUpperPos);

        sleep(1000);

        telemetry.addData("Initialization Complete","");

        while (opModeIsActive()){
            double pos = -gamepad1.left_stick_y;
            if (pos < 0) pos = 0;

            if (gamepad1.a) {
                leftUpperPos = pos;
                bot.leftUpperClamp.setPosition(pos);
            }
            else if (gamepad1.b){
                rightUpperPos = pos;
                bot.rightUpperClamp.setPosition(pos);
            }
            else if (gamepad1.x){
                leftLowerPos = pos;
                bot.leftLowerClamp.setPosition(pos);
            }
            else if (gamepad1.y){
                rightLowerPos = pos;
                bot.rightLowerClamp.setPosition(pos);
            }

            telemetry.addData("Left Upper (a)", leftUpperPos);
            telemetry.addData("Right Upper (b)", rightUpperPos);
            telemetry.addData("Left Lower (x)", leftLowerPos);
            telemetry.addData("Right Lower (y)", rightLowerPos);
            telemetry.update();
        }
    }
}
