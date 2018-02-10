package org.firstinspires.ftc.teamcode.competition_in_work.driver_control;


import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.MechBotRedHook;
import org.firstinspires.ftc.teamcode.third_party_libs.UTILToggle;

/**
 * Created by FTC Team 8397 on 12/5/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Albany-TeleOp-Slow", group="Comp")
public class TeleOpAlbany extends LoggingLinearOpMode {

    private MechBotRedHook bot = new MechBotRedHook();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,bot);
    private float[] driveData = new float[6];
    UTILToggle topToggle = new UTILToggle();
    boolean topStatus = false;
    GrabberState topState = GrabberState.OPEN;
    UTILToggle bottomToggle = new UTILToggle();
    boolean bottomStatus = false;
    GrabberState bottomState = GrabberState.OPEN;

    UTILToggle armSlowModeToggle = new UTILToggle();
    boolean speedToggle = false;



    enum GrabberState{CLOSED,OPEN}
    //Button logic.
    //Bottom can't close till top opens.
    //Top can't close till bottom closes.

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap);

        //TODO
      //  MediaPlayer mPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.champions);
        //TODO

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
        float avgStopPos = 0;

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

            if (gamepad1.right_stick_y <-.5 || gamepad1.right_stick_y >.5 || gamepad2.dpad_down || gamepad2.dpad_up){
                avgStopPos = (Math.abs(bot.leftLinearSlide.getCurrentPosition()) + Math.abs(bot.rightLinearSlide.getCurrentPosition())) /2.0f;

            }


            //TODO
            //REMAP SERVO CONTROLS
            //Servo controls and servo functions are not reflective of what they really do.
            //Please test them to fix it.

            //Toggle should be: mid-> close
            //And secondary button for open

            if (bottomToggle.status(gamepad1.y) == UTILToggle.Status.COMPLETE) {
                if(!topStatus) {
                    bot.closeUpperClamp();
                    topState = GrabberState.CLOSED;
                }
                else if(topStatus) {
                    bot.openUpperClamp();
                    topState = GrabberState.OPEN;
                }
                topStatus = !topStatus;
            }

            if (topToggle.status(gamepad1.a) == UTILToggle.Status.COMPLETE) {
                if(!bottomStatus) {
                    bot.closeLowerClamp();
                    bottomState = GrabberState.CLOSED;
                }
                else if(bottomStatus) {
                    bot.openLowerClamp();
                    bottomState = GrabberState.OPEN;
                }
                bottomStatus =!bottomStatus;
            }

            if (gamepad1.right_bumper){
                bot.midPosUpperClamp();
            }
            if(gamepad1.left_bumper){
                bot.midPosLowerClamp();
            }
            if(gamepad1.dpad_right){
                bot.fullOpenUpperClamp();
            }

            if(gamepad1.x){
                bot.leftLowPos = bot.leftLinearSlide.getCurrentPosition();
                bot.rightLowPos = Math.abs(bot.rightLinearSlide.getCurrentPosition());
                avgStartPos = (bot.leftLowPos + bot.rightLowPos) /2.0f;
            }

            float checkerValue = ((Math.abs(bot.leftLinearSlide.getCurrentPosition()) + Math.abs(bot.rightLinearSlide.getCurrentPosition()))/2f);
            telemetry.addData("Cond value: ", checkerValue);

            if ( gamepad2.dpad_up) {
                bot.liftArmUp();
            } else if(gamepad2.dpad_down && gamepad2.b){
                bot.liftArmDown();
            }
            else if (gamepad2.dpad_down) {
                if(bot.leftLinearSlide.getCurrentPosition() <= avgStartPos) {
                    bot.liftArmDown();
                }
            }else if (gamepad1.right_stick_y > .5) { //up
                if(bot.leftLinearSlide.getCurrentPosition() <= avgStartPos) {
                    bot.liftArmDown();
                }
            } else if (gamepad1.right_stick_y < -.5) { //down
                bot.liftArmUp();
            }
            else {
                //bot.liftArmStop();
                liftArmHoldPos(avgStopPos);
            }

            if(gamepad1.dpad_up){
                bot.raiseJewelArm();
            }else if(gamepad1.dpad_down){
                bot.lowerJewelArm();
            }

            //TODO
            //if(gamepad1.b){
              //  mPlayer.start();
            //}else if(gamepad2.b){
             //   mPlayer.pause();
            //}
            //TODO

            if (armSlowModeToggle.status(gamepad2.y) == UTILToggle.Status.COMPLETE) {
                if(!speedToggle) {
                    telemetry.addData("Arm Slow Mode: ", true);
                    bot.relicArmModify = 2.0f;
                }
                else if(speedToggle) {
                    telemetry.addData("Arm Slow Mode: ", false);
                    bot.relicArmModify = 1;
                }
                speedToggle =!speedToggle;
            }

            if(gamepad2.right_stick_y < -.05){
                bot.relicArmOut();
            }else if(gamepad2.right_stick_y > .05){
                bot.relicArmIn();
            }else{
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

    public void liftArmHoldPos (float avgStopPos){
        final float C_LIFT = 0.01f;
        float leftLinearSlidePos = Math.abs(bot.leftLinearSlide.getCurrentPosition());
        float rightLinearSlidePos = Math.abs(bot.rightLinearSlide.getCurrentPosition());
        float avgPos = (leftLinearSlidePos+rightLinearSlidePos)/2f;
//        boolean liftStop = (avgPos < avgStopPos-50);
//        while (opModeIsActive() && liftStop){
//            bot.leftLinearSlide.setPower(.50);
//            bot.rightLinearSlide.setPower(.50);
//            avgPos = (Math.abs(bot.leftLinearSlide.getCurrentPosition())+Math.abs(bot.rightLinearSlide.getCurrentPosition()))/2f;
//            liftStop = (avgPos < avgStopPos-50);
//            telemetry.addData("liftTest","avgStopPos"+ avgStopPos);
//            telemetry.addData("liftTest"," avgPos"+ avgPos);
//            telemetry.addData("liftTest","LiftStop" + liftStop);
//        }
//        bot.liftArmStop();
        float error = avgStopPos-avgPos;

        telemetry.addData("SlidePosition","Left: %d  Right: %d",bot.leftLinearSlide.getCurrentPosition(),
                bot.rightLinearSlide.getCurrentPosition());
        telemetry.addData("AvgSlide","Target: %.0f  Actual: %.0f Error: %.0f", avgStopPos, avgPos, error);

        if (error > 0){
            float power = C_LIFT*error;
            bot.leftLinearSlide.setPower(power);
            bot.rightLinearSlide.setPower(power);
        }
        else{
            bot.leftLinearSlide.setPower(0);
            bot.rightLinearSlide.setPower(0);
        }
    }
}




