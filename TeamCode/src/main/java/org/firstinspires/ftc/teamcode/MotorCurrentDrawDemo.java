package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.openftc.hardware.rev.OpenRevDcMotorImplEx;

/**
 * Created by FTC Team 8397 on 3/8/2018.
 */
@TeleOp(name = "CurrentDrawTest", group = "Test")
public class MotorCurrentDrawDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpenRevDcMotorImplEx motor1 = new OpenRevDcMotorImplEx((DcMotorImplEx) hardwareMap.dcMotor.get("motor1"));

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor1.setPower(1);

                telemetry.addData("Draw: ", String.format("%.2f", motor1.getCurrentDraw()));
                telemetry.update();
            }

            motor1.setPower(0);
        }
    }
}
