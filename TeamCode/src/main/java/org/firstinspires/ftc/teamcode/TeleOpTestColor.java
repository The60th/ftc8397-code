package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by CanAdirondack on 1/24/2017.
 */
@Disabled
@TeleOp(name = "TeleOP Colors: ", group = "TeleOp_Programs")

public class TeleOpTestColor extends LinearOpMode {
    private OmniBot robot = new OmniBot();
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("", "Colors 1: Red %f Green %f Blue %f", robot.sensorRGB_One.red(), robot.sensorRGB_One.green(), robot.sensorRGB_One.blue());
            telemetry.addData("", "Colors 2: Red %f Green %f Blue %f", robot.sensorRGB_TWO.red(), robot.sensorRGB_TWO.green(), robot.sensorRGB_TWO.blue());
            telemetry.update();
        }
    }

}
