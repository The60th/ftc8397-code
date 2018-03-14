package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.mechbot.utill.MechBotDriveControls;

/**
 * Created by FTC Team 8397 on 2/22/2018.
 */
@TeleOp(name = "Raw mechBot",group = "Rev")
@Disabled
public class mechBotRaw extends LinearOpMode {
    MechBot bot = new MechBot();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,bot);
    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            mechBotDriveControls.refreshGamepads(gamepad1, gamepad2);
        mechBotDriveControls.joyStickMecnumDriveCompNewBot(new float[8]);
    }
    }
}
