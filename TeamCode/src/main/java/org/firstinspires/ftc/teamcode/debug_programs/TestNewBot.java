package org.firstinspires.ftc.teamcode.debug_programs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.mechbot.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.MechBotIntake;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;


/**
 * Created by FTC Team 8397 on 2/20/2018.
 */
@TeleOp(name = "TestNewBot", group = "Tele opmode")
public class TestNewBot extends LoggingLinearOpMode {
    MechBotIntake bot = new MechBotIntake();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,bot);

    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            mechBotDriveControls.refreshGamepads(gamepad1, gamepad2);
            mechBotDriveControls.joyStickMecnumDriveCompQuadSlow(new float[8]);

            if (gamepad2.right_stick_y > .05){
                bot.rightIntake.setPower(-.5);
            }
            else if (gamepad2.right_stick_y < -.05){
                bot.rightIntake.setPower(.5);
            }
            else{
                bot.rightIntake.setPower(0);
            }
            if (gamepad2.left_stick_y > .05){
                bot.leftIntake.setPower(-.5);
            }
            else if (gamepad2.left_stick_y < -.05){
                bot.leftIntake.setPower(.5);
            }
            else{
                bot.leftIntake.setPower(0);
            }
        }
    }
}
