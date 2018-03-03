package org.firstinspires.ftc.teamcode.debug_programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotGyro;
import org.firstinspires.ftc.teamcode.mechbot.utill.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotFlip;


/**
 * Created by FTC Team 8397 on 2/20/2018.
 */
@TeleOp(name = "ESR Bot", group = "Tele opmode")
public class TestNewBot extends LoggingLinearOpMode {
    MechBotGyro bot = new MechBotGyro();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,bot);
    private float[] driveData = new float[6];

    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap);
        telemetry.addData("Ready to start TeleOp, waiting for starting button.","");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            mechBotDriveControls.refreshGamepads(gamepad1, gamepad2);
            mechBotDriveControls.joyStickMecnumDriveCompNewBot(driveData);
            telemetry.addData("Joystick input: ", "X: %.2f Y: %.2f A: %.2f", driveData[0], driveData[1], driveData[2]);
            telemetry.addData("Drive speeds input: ", "X: %.2f Y: %.2f A: %.2f", driveData[3], driveData[4], driveData[5]);
            bot.gyroTelemetry(telemetry);
            telemetry.update();
            /*if (gamepad2.right_stick_y > .05){
                bot.rightIntake.setPower(-1);
            }
            else if (gamepad2.right_stick_y < -.05){
                bot.rightIntake.setPower(1);
            }
            else{
                bot.rightIntake.setPower(0);
            }
            if (gamepad2.left_stick_y > .05){
                bot.leftIntake.setPower(-1);
            }
            else if (gamepad2.left_stick_y < -.05){
                bot.leftIntake.setPower(1);
            }
            else{
                bot.leftIntake.setPower(0);
            }*/

         if(gamepad1.right_bumper){
             bot.rightIntake.setPower(1);
             bot.leftIntake.setPower(1);
         }
         else if(gamepad1.left_bumper){
             bot.rightIntake.setPower(-1);
             bot.leftIntake.setPower(-1);
         }
         else{
             bot.rightIntake.setPower(0);
             bot.leftIntake.setPower(0);
         }

         if(gamepad1.y){
             bot.flipPlateUpwards();
         }else if(gamepad1.a){
             bot.flipPlateDownwards();
         }

         if (gamepad2.a){
             bot.pinchGlyph();
         }
         else if (gamepad2.y){
             bot.setGlyph();
         }
         else if (gamepad2.x){
             bot.startPos();
         }
        }


    }

}
