package org.firstinspires.ftc.teamcode.old_or_test_software;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;
import org.firstinspires.ftc.teamcode.mechbot.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.MechBotRedHook;
import org.firstinspires.ftc.teamcode.third_party_libs.UTILToggle;

/**
 * Created by FTC Team 8397 on 12/5/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="testTurnJewelServo", group="Comp")
public class testTurnJewelServo extends LoggingLinearOpMode {

    private MechBotRedHook bot = new MechBotRedHook();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,bot);
    private float[] driveData = new float[6];
    UTILToggle topToggle = new UTILToggle();
    boolean topStatus = false;
    GrabberState topState = GrabberState.OPEN;
    UTILToggle bottomToggle = new UTILToggle();
    boolean bottomStatus = false;
    GrabberState bottomState = GrabberState.OPEN;

    enum GrabberState{CLOSED,OPEN}
    //Button logic.
    //Bottom can't close till top opens.
    //Top can't close till bottom closes.

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap);

        //TODO
        MediaPlayer mPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.champions);
        //TODO

        ElapsedTime et = new ElapsedTime();

        telemetry.addData("Ready to start TeleOp, waiting for starting button.", "");
        telemetry.update();
        bot.updateOdometry();

        waitForStart();
        telemetry.addData("Started TeleOp", "");
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

        float avgStartPos = (bot.leftLowPos + bot.rightLowPos) / 2.0f;


        while (opModeIsActive()){

            mechBotDriveControls.refreshGamepads(gamepad1, gamepad2);
            mechBotDriveControls.joyStickMecnumDriveCompQuadSlow(driveData); //Do an array fill by passing the array in, to prevent recreating the array.

            telemetry.addData("Joystick input: ", "X: %.2f Y: %.2f A: %.2f", driveData[0], driveData[1], driveData[2]);
            telemetry.addData("Drive speeds input: ", "X: %.2f Y: %.2f A: %.2f", driveData[3], driveData[4], driveData[5]);

            telemetry.addData("LeftLowPos: ", bot.leftLowPos);
            telemetry.addData("RightLowPos: ", bot.rightLowPos);

            telemetry.addData("LeftPos: ", bot.leftLinearSlide.getCurrentPosition());
            telemetry.addData("RightPos: ", bot.rightLinearSlide.getCurrentPosition()); // is -
            telemetry.addData("Avg pos: ", avgStartPos);


            if(gamepad1.x){
                knockJewelLeft();
            }
            else if (gamepad1.b){
                knockJewelRight();
            }
            else if (gamepad1.a){
                jewelArmMidPosition();
            }
            else if (gamepad1.y){
                knockJewelWithTurnServo(MechBotAutonomous.Side.RIGHT);
            }
        }
    }

    public void knockJewelRight(){
        bot.turnJewelArm.setPosition(.5); // enter position for right turn.
    }
    public void knockJewelLeft(){
        bot.turnJewelArm.setPosition(.7); // enter position for left turn.
    }
    public void jewelArmMidPosition(){
        bot.turnJewelArm.setPosition(.6); // enter position for starting mid.
    }
    public boolean knockJewelWithTurnServo(MechBotAutonomous.Side side) {
        {
            if (side == MechBotAutonomous.Side.UNKNOWN) {
                return false;
            }
            lowerJewelArm();
            sleep(1050);
            if (side == MechBotAutonomous.Side.LEFT) {
                knockJewelLeft();
                sleep(500);
                jewelArmMidPosition();
                sleep(500);
                raiseJewelArm();
                sleep(1050);
            } else if (side == MechBotAutonomous.Side.RIGHT) {
                knockJewelRight();
                sleep(500);
                jewelArmMidPosition();
                sleep(500);
                raiseJewelArm();
                sleep(1050);
            }
        }
        return true;
    }
    public void lowerJewelArm(){bot.jewelArm.setPosition(1);}
    public void raiseJewelArm(){bot.jewelArm.setPosition(0);}
}

