package org.firstinspires.ftc.teamcode.old_or_test_software;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechbot.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.MechBotBackUpBot;

/**
 * Created by FTC Team 8397 on 10/30/2017.
 */
@TeleOp(name="BackupBot", group="Other")
@Disabled
public class BackupBot extends LinearOpMode{
    public MechBotBackUpBot bot = new MechBotBackUpBot();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,bot);
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            mechBotDriveControls.refreshGamepads(gamepad1, gamepad2);
            if (gamepad2.right_trigger > .25) {
                bot.setInTakePower(1);
            } else if (gamepad2.left_trigger > .25) {
                bot.setInTakePower(-1);
            } else if (gamepad2.dpad_up) {
                bot.lift.setPower(1.0);
            } else if (gamepad2.dpad_down) {
                bot.lift.setPower(-.40);
            }
            else if(gamepad2.x){
                bot.clapIn();
            }
            else if(gamepad2.a){
                bot.clapOff();
            }
            else {
                bot.setInTakePower(0);
                bot.lift.setPower(0);
                bot.clapRest();
            }

            mechBotDriveControls.joyStickMecnumDrive();

            if (gamepad1.x) {
                mechBotDriveControls.driveDirectonByPower(MechBotDriveControls.XYZ.negX, 1);
            } else if (gamepad1.b) {
                mechBotDriveControls.driveDirectonByPower(MechBotDriveControls.XYZ.plusX, 1);
            } else if (gamepad1.y) {
                mechBotDriveControls.driveDirectonByPower(MechBotDriveControls.XYZ.plusY, 1);
            } else if (gamepad1.a) {
                mechBotDriveControls.driveDirectonByPower(MechBotDriveControls.XYZ.negY, 1);
            }

        }
    }
}
