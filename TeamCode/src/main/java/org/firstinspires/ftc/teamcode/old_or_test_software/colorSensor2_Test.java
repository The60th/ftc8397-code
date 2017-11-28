package org.firstinspires.ftc.teamcode.old_or_test_software;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by FTC Team 8397 on 11/26/2017.
 */
@TeleOp(name="dd",group = "ddd")
public class colorSensor2_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor colorSensor;
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color2");
        colorSensor.setI2cAddress(I2cAddr.create8bit(0x70));
        waitForStart();
        while (opModeIsActive()){
        float[] hsvValues2 = new float[3];

        Color.RGBToHSV(colorSensor.red() * 8,colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues2);
        telemetry.addData("demo","Sensor Two: Red: %d Green %d Blue %d Hue: %f Sat: %f Value:",colorSensor.red(),colorSensor.green(),colorSensor.blue(),
                hsvValues2[0],hsvValues2[1],hsvValues2[2]);
        telemetry.update();
        }
    }
}
