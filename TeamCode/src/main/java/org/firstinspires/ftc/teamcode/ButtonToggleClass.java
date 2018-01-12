package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.third_party_libs.UTILToggle;

/**
 * Created by FTC Team 8397 on 1/3/2018.
 */
//@TeleOp(group = "Demo",name = "Toggle")
public class ButtonToggleClass extends LinearOpMode {
    UTILToggle slowToggle = new UTILToggle();    // Slows down drivetrain when on
    boolean demo = false;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            if (slowToggle.status(gamepad1.a) == UTILToggle.Status.COMPLETE) {
                telemetry.addData("Demo: ", demo);
                telemetry.update();
                demo = !demo;
            }
        }
    }
}
