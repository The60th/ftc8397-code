package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by FTC Team 8397 on 3/1/2018.
 */
@TeleOp(name = "Color reader", group = "color")
public class Color_Test extends LinearOpMode {
    ColorSensor cc;
    float[] hsvValues = new float[3];
    @Override
    public void runOpMode() throws InterruptedException {
        cc = hardwareMap.get(ColorSensor.class, "cc");
        waitForStart();
        
        while (opModeIsActive()){
            Color.RGBToHSV(cc.red() * 8, cc.green() * 8, cc.blue() * 8, hsvValues);
            telemetry.addData("CC: "," red: " + cc.red() + "Green: " + cc.green() + " Blue: " + cc.blue());
            telemetry.addData("CC HSV: ", " Hue: " + hsvValues[0] + " Sat " + hsvValues[1] + " Value: " + hsvValues[2]);
            telemetry.update();
        }
    }
}
