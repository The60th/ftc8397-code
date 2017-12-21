package org.firstinspires.ftc.teamcode.mechbot;

import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;

/**
 * Created by FTC Team 8397 on 12/19/2017.
 */

public abstract class MechBotTeleop extends LoggingLinearOpMode {
    public MechBotRedHook bot = new MechBotRedHook();
    public enum State{NORMAL,QUAD_DRIVE,CUSTOM_DRIVE,SINGLE_CONTROLLER_DRIVE}
    public State state = State.NORMAL;
    public interface Customdrive{
        void DriveOneCycle();
    }
    public void normalDrive(){
        float x = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
        float y = Math.abs(gamepad1.left_stick_y) > 0.05 ? gamepad1.left_stick_y : 0;
        float a = 0;
        if (gamepad1.left_trigger > .05 || gamepad1.right_trigger > .05) {
            if (gamepad1.left_trigger > gamepad1.right_trigger)
                a = -gamepad1.left_trigger;
            else {
                a = gamepad1.right_trigger;
            }
        }
        bot.setDrivePower((-y), (-x), (-a));
    }



}
