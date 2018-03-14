package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.i2c.BNO055EnhancedImpl;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotSensorScranton;

import java.text.DecimalFormat;

/**
 * Created by FTC Team 8397 on 2/28/2018.
 */
@TeleOp(name = "Sensor debuging", group = "Rev")
@Disabled
public class GyroTest extends LinearOpMode {
    MechBotSensorScranton bot = new MechBotSensorScranton();
    private static DecimalFormat df2 = new DecimalFormat(".##");
    final float[] hsvValues = new float[3];
    final float[] hsvValues2 = new float[3];

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            Orientation angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("ZYX: ","Z: " + String.format("%.2f",angles.firstAngle) + " Y " + String.format("%.2f",angles.secondAngle) + " X " + String.format("%.2f",angles.thirdAngle));

            Color.RGBToHSV(bot.colorLeft.red() * 8, bot.colorLeft.green() * 8, bot.colorLeft.blue() * 8, hsvValues);
            telemetry.addData("ColorLeft: "," red: " + bot.colorLeft.red() + "Green: " + bot.colorLeft.green() + " Blue: " + bot.colorLeft.blue());
            telemetry.addData("ColorLeft HSV: ", " Hue: " + hsvValues[0] + " Sat " + hsvValues[1] + " Value: " + hsvValues[2]);

            Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues2);
            telemetry.addData("ColorRight: "," red: " + bot.colorRight.red() + "Green: " + bot.colorRight.green() + " Blue: " + bot.colorRight.blue());
            telemetry.addData("ColorRight HSV: ", " Hue: " + hsvValues2[0] + " Sat " + hsvValues2[1] + " Value: " + hsvValues2[2]);
            telemetry.update();


        }
    }
}
