package org.firstinspires.ftc.teamcode.competition_in_work.driver_control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.utill.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.presupers_bot.MechBotRedHook;
import org.firstinspires.ftc.teamcode.third_party_libs.UTILToggle;

/**
 * Created by FTC Team 8397 on 1/18/2018.
 */

public class PaceTele  extends LoggingLinearOpMode {
    //TODO add swag flags commands.
    private MechBotRedHook bot = new MechBotRedHook();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,bot);
    private float[] driveData = new float[6];
    UTILToggle topToggle = new UTILToggle();
    boolean topStatus = false;
    TeleOpAlbany.GrabberState topState = TeleOpAlbany.GrabberState.OPEN;
    UTILToggle bottomToggle = new UTILToggle();
    boolean bottomStatus = false;
    TeleOpAlbany.GrabberState bottomState = TeleOpAlbany.GrabberState.OPEN;

    enum GrabberState{CLOSED,OPEN}
    //Button logic.
    //Bottom can't close till top opens.
    //Top can't close till bottom closes.

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap);


        ElapsedTime et = new ElapsedTime();

        telemetry.addData("Ready to start TeleOp, waiting for starting button.","");
        telemetry.update();
        bot.updateOdometry();

        waitForStart();
        telemetry.addData("Started TeleOp","");
        telemetry.update();
        bot.raiseJewelArm();
        bot.openLowerClamp();
        bot.openUpperClamp();

        bot.leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bot.leftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.rightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //bot.leftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //bot.rightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bot.leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bot.rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bot.leftLowPos = bot.leftLinearSlide.getCurrentPosition();
        bot.rightLowPos = Math.abs(bot.rightLinearSlide.getCurrentPosition());

        float avgStartPos = (bot.leftLowPos + bot.rightLowPos) /2.0f;

        while (opModeIsActive()) {
            mechBotDriveControls.refreshGamepads(gamepad1, gamepad2);
            mechBotDriveControls.joyStickMecnumDriveCompQuadSlow(driveData); //Do an array fill by passing the array in, to prevent recreating the array.

            telemetry.addData("Joystick input: ", "X: %.2f Y: %.2f A: %.2f", driveData[0], driveData[1], driveData[2]);
            telemetry.addData("Drive speeds input: ", "X: %.2f Y: %.2f A: %.2f", driveData[3], driveData[4], driveData[5]);

            telemetry.addData("LeftLowPos: ", bot.leftLowPos);
            telemetry.addData("RightLowPos: ", bot.rightLowPos);

            telemetry.addData("LeftPos: ", bot.leftLinearSlide.getCurrentPosition());
            telemetry.addData("RightPos: ", bot.rightLinearSlide.getCurrentPosition()); // is -
            telemetry.addData("Avg pos: ", avgStartPos);


            //TODO
            //REMAP SERVO CONTROLS
            //Servo controls and servo functions are not reflective of what they really do.
            //Please test them to fix it.

            //Toggle should be: mid-> close
            //And secondary button for open

            if (bottomToggle.status(gamepad1.y) == UTILToggle.Status.COMPLETE) {
                if(!topStatus) {
                    bot.closeUpperClamp();
                    topState = TeleOpAlbany.GrabberState.CLOSED;
                }
                else if(topStatus) {
                    bot.openUpperClamp();
                    topState = TeleOpAlbany.GrabberState.OPEN;
                }
                topStatus = !topStatus;
            }

            if (topToggle.status(gamepad1.a) == UTILToggle.Status.COMPLETE) {
                if(!bottomStatus) {
                    bot.closeLowerClamp();
                    bottomState = TeleOpAlbany.GrabberState.CLOSED;
                }
                else if(bottomStatus) {
                    bot.openLowerClamp();
                    bottomState = TeleOpAlbany.GrabberState.OPEN;
                }
                bottomStatus =!bottomStatus;
            }

            if (gamepad1.right_bumper){
                bot.midPosUpperClamp();
            }
            if(gamepad1.left_bumper){
                bot.midPosLowerClamp();
            }

            if(gamepad1.x){
                bot.leftLowPos = bot.leftLinearSlide.getCurrentPosition();
                bot.rightLowPos = Math.abs(bot.rightLinearSlide.getCurrentPosition());
                avgStartPos = (bot.leftLowPos + bot.rightLowPos) /2.0f;
            }

            float checkerValue = ((Math.abs(bot.leftLinearSlide.getCurrentPosition()) + Math.abs(bot.rightLinearSlide.getCurrentPosition()))/2);
            telemetry.addData("Cond value: ", checkerValue);

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                bot.liftArmUp();
            } else if(gamepad1.b && gamepad1.dpad_down){
                bot.liftArmDown();
            } else if(gamepad2.b && gamepad2.dpad_down){
                bot.liftArmDown();
            }
            else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                if(bot.leftLinearSlide.getCurrentPosition() >= avgStartPos) {
                    bot.liftArmDown();
                }
            }
            else {
                bot.liftArmStop();
            }


            if (gamepad1.right_stick_y > .5) { //up
                bot.lowerJewelArm();
            } else if (gamepad1.right_stick_y < -.5) { //down
                bot.raiseJewelArm();
            }

            if (gamepad2.y) {
                bot.relicArmOut();
            } else if (gamepad2.a) {
                bot.relicArmIn();
            } else {
                bot.relicArmStop();
            }

            if (gamepad2.right_bumper) {
                bot.liftRelicArmUp();
            } else if (gamepad2.right_trigger > .05) {
                bot.liftRelicArmDown();
            } else {
                bot.liftRelicArmStop();
            }
            if (gamepad2.left_bumper) {
                bot.relicClampOpen();
            }
            if (gamepad2.left_trigger > .05) {
                bot.relicClampClose();
            }
            if(gamepad2.x) {
                bot.relicClampMid();
            }
            telemetry.update();

        }
    }
}
