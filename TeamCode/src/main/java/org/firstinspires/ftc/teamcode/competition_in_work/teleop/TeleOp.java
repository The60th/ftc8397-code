package org.firstinspires.ftc.teamcode.competition_in_work.teleop;

import android.graphics.Color;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.MechBotNickBot;

/**
 * Created by FTC Team 8397 on 11/30/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Comp")
public class TeleOp extends LoggingLinearOpMode{

    private MechBotNickBot mechBot = new MechBotNickBot();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,mechBot);

    final double armModify = 0.0006;
    double armPos = .5;
    final double und_arm = 0.2;
    final double ovr_arm = 0.7;

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        mechBot.init(hardwareMap);

        telemetry.addData("Ready to go: ","");
        telemetry.update();

        mechBot.updateOdometry();
        waitForStart();
        mechBot.raiseJewelArm();
        sleep(100);
        mechBot.breakJewelArm();
        telemetry.addData("Starting","");
        telemetry.update();

        while (opModeIsActive()) {
            mechBotDriveControls.refreshGamepads(gamepad1,gamepad2);
            mechBotDriveControls.joyStickMecnumDriveComp();

            //Extend block collector.
            if(gamepad1.dpad_up){
                mechBot.driveArm(-1.0f);
            }else if(gamepad1.dpad_down){
                mechBot.driveArm(1.0f);
            }else{
                mechBot.driveArm(.0f);
            }
            if(gamepad1.right_stick_y > .5){
                mechBot.lowerJewelArm();
            }else if(gamepad1.right_stick_y < -.5){
                mechBot.raiseJewelArm();
            }else{
                mechBot.breakJewelArm();
            }

            //Drive block life. //Lifting the block up.
            if(gamepad2.dpad_up){
                mechBot.blockLiftMotor.setPower(1);
            }else if(gamepad2.dpad_down){
                mechBot.blockLiftMotor.setPower(-.5);
            }
            else{
                mechBot.blockLiftMotor.setPower(0);
            }

            //Swing block pusher. //Servo the pushes block down slide.
            if (gamepad2.right_trigger > .2 || gamepad2.left_trigger > .2) {
                mechBot.kickerServo.setPosition(-.85);
            } else  {
                mechBot.kickerServo.setPosition(.85);
            }

            if(gamepad2.y  && armPos <= ovr_arm) {
                armPos += armModify;
            } else if (gamepad2.a  && armPos >= und_arm) {
                armPos -= armModify;
            }

            mechBot.slideServo.setPosition(armPos);

            telemetry.addData("Gamepad 1: ","Left Stick Values X:"+gamepad1.left_stick_x+" Y: "+gamepad1.left_stick_y + " | Right Stick Values X:"+gamepad1.left_stick_x+" Y: "+gamepad1.left_stick_y);
            telemetry.addData("Gamepad 2: ","Left Stick Values X:"+gamepad2.right_stick_x+" Y: "+gamepad2.right_stick_y + " | Right Stick Values X:"+gamepad2.right_stick_x+" Y: "+gamepad2.right_stick_y);
            telemetry.addData("Gamepad 1: " , gamepad1.toString());
            telemetry.addData("Gamepad 2: " , gamepad2.toString());
            telemetry.update();
        }
    }
}
