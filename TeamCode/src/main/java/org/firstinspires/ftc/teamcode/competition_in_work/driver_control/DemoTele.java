package org.firstinspires.ftc.teamcode.competition_in_work.driver_control;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.MechBotPace;
import org.firstinspires.ftc.teamcode.mechbot.MechBotRedHook;
import org.firstinspires.ftc.teamcode.third_party_libs.UTILToggle;

/**
 * Created by FTC Team 8397 on 1/18/2018.
 */
@TeleOp(name = "SwagFlag Tester", group = "Comp")
@Disabled
public class DemoTele  extends LoggingLinearOpMode {

    private MechBotPace bot = new MechBotPace();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1, gamepad2, bot);
    private float[] driveData = new float[6];
    UTILToggle topToggle = new UTILToggle();
    boolean topStatus = false;
    TeleOpAlbany.GrabberState topState = TeleOpAlbany.GrabberState.OPEN;
    UTILToggle bottomToggle = new UTILToggle();
    boolean bottomStatus = false;
    TeleOpAlbany.GrabberState bottomState = TeleOpAlbany.GrabberState.OPEN;

    enum GrabberState {CLOSED, OPEN}
    //Button logic.
    //Bottom can't close till top opens.
    //Top can't close till bottom closes.

    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap);


        ElapsedTime et = new ElapsedTime();

        telemetry.addData("Ready to start TeleOp, waiting for starting button.", "");
        telemetry.update();
        bot.updateOdometry();

        waitForStart();
        telemetry.addData("Started TeleOp", "");
        telemetry.update();
        //bot.raiseJewelArm();
        //bot.openLowerClamp();
       // bot.openUpperClamp();

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

        while (opModeIsActive()) {
            mechBotDriveControls.refreshGamepads(gamepad1, gamepad2);
            mechBotDriveControls.joyStickMecnumDriveCompQuadSlow(driveData); //Do an array fill by passing the array in, to prevent recreating the array.
            if(gamepad1.x){
                bot.turnJewelArmLeft();
                telemetry.addData("Pos = " , "Left 1");
            }else if(gamepad1.b){
                bot.turnJewelArmRight();
                telemetry.addData("Pos = " , "Right 0");
            }else if(gamepad1.a){
                bot.turnJewelArmCenter();
                telemetry.addData("Pos = " , "Center .5");
            }
            telemetry.update();
        }
    }
}
