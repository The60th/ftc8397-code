package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;

/**
 * Created by FTC Team 8397 on 10/30/2017.
 */
@TeleOp(name="BackupBot", group="Other")
public class BackupBot extends LinearOpMode{
    private MechBotBackUpBot bot = new MechBotBackUpBot();
    private  MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,bot);
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();

        while(opModeIsActive())
        mechBotDriveControls.refreshGamepads(gamepad1,gamepad2);
        if(mechBotDriveControls.isGamepadRefreshed()) {
            if (mechBotDriveControls.joyStickMecnumDrive()) {
            }else if(gamepad1.right_trigger > .25){bot.setInTakePower(1);}
            else if(gamepad1.left_trigger > .25){bot.setInTakePower(-1);}
            else if(gamepad1.dpad_up){bot.setLiftPower(.10f);}
            else if(gamepad1.dpad_down){bot.setLiftPower(-.10f);}
        }else{
            telemetry.addData("", "Calling joyStickMecnumDrive without updating gamepads");
        }
    }
}
