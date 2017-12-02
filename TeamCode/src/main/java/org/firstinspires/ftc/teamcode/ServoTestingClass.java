package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.mechbot.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.MechBotNickBot;

/**
 * Created by FTC Team 8397 on 11/30/2017.
 */

//@TeleOp(name="servoTesting", group="Rev")
public class ServoTestingClass extends LoggingLinearOpMode {
    private MechBotNickBot mechBot = new MechBotNickBot();
    //private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,mechBot);
   // private float[] driveHeading = new float[]{0,0,0};
    @Override
    public void runLoggingOpmode() throws InterruptedException {
        mechBot.init(hardwareMap);
        double jewelPos = .5;
        double blockSmackServoPos = .5;
        double blockSLiderSerovPos = .5;
        boolean jewelToggle = true;
        waitForStart();
        while (opModeIsActive()) {
            /*if (gamepad1.y) {
                mechBot.jewelServo.setPosition(.4);
            } else if (gamepad1.a) {
                mechBot.jewelServo.setPosition(.6);
            }*/

            if(gamepad2.b)jewelToggle=true;

            if (gamepad2.dpad_up) {
                mechBot.kickerServo.setPosition(.9);
            } else if (gamepad2.dpad_down) {
                mechBot.kickerServo.setPosition(.3);
            }

            if (gamepad1.dpad_up && jewelToggle) {
                jewelToggle= false;
                blockSLiderSerovPos +=.1;
                //mechBot.slideServo.se(blockSLiderSerovPos);
            } else if (gamepad1.dpad_down && jewelToggle) {
                blockSLiderSerovPos -=.1;
                jewelToggle= false;
                //mechBot.slideServo.setPosition(blockSLiderSerovPos);
            }

            telemetry.addData("My servo data: ", "jewelPos " +/* mechBot.jewelServo.getPosition() +*/ " blockSlie " +/* mechBot.slideServo.getPosition() +*/ " blockSmackPos " + mechBot.kickerServo.getPosition() + " jewelToggle " + jewelToggle);
            telemetry.update();
        }
    }
}