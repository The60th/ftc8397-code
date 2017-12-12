package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by FTC Team 8397 on 12/11/2017.
 */
@TeleOp(name = "rev",group = "rev")
public class RevServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo;
        servo = hardwareMap.servo.get("servo");


         waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                servo.setPosition(1);
            } else if (gamepad1.dpad_down) {
                servo.setPosition(0);
            } else if (gamepad1.dpad_left) {
                servo.setPosition(.5);
            }

        }
    }
}
