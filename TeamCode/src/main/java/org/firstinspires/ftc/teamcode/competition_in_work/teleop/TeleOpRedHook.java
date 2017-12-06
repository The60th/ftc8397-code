package org.firstinspires.ftc.teamcode.competition_in_work.teleop;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.mechbot.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.MechBotRedHook;

/**
 * Created by FTC Team 8397 on 12/5/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Red hook comp", group="Comp")
public class TeleOpRedHook extends LoggingLinearOpMode {

    private MechBotRedHook bot = new MechBotRedHook();
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,bot);
    private float[] driveData = new float[6];

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

        while (opModeIsActive()){
            mechBotDriveControls.refreshGamepads(gamepad1,gamepad2);
            mechBotDriveControls.joyStickMecnumDriveComp(driveData); //Do an array fill by passing the array in, to prevent recreating the array.


            telemetry.addData("Joystick input: ","X: %.2f Y: %.2f A: %.2f", driveData[0],driveData[1],driveData[2]);
            telemetry.addData("Drive speeds input: ","X: %.2f Y: %.2f A: %.2f", driveData[3],driveData[4],driveData[5]);
            telemetry.update();

        }
    }
}
