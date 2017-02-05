package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.graphics.Color;

/**
 * Created by JimLori on 2/3/2017.
 *
 * Repeatedly read both color sensors; report RGB, hue, and RED/BLUE/UNKNOWN
 */

@Autonomous(name = "Test Color Sensors", group = "Test Op Modes")
public class TestColorSensors extends OmniBotAutonomous {


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        ElapsedTime et = new ElapsedTime();

        while (opModeIsActive()){

            if (et.milliseconds() < 1000) continue;

            int rightRed = robot.sensorRGB_One.red();
            int rightGreen = robot.sensorRGB_One.green();
            int rightBlue = robot.sensorRGB_One.blue();

            int leftRed = robot.sensorRGB_TWO.red();
            int leftGreen = robot.sensorRGB_TWO.green();
            int leftBlue = robot.sensorRGB_TWO.blue();

            float[] rightHSV = new float[]{0,0,0};
            float[] leftHSV = new float[]{0,0,0};

            Color.RGBToHSV(rightRed, rightGreen, rightBlue, rightHSV);
            Color.RGBToHSV(leftRed, leftGreen, leftBlue, leftHSV);

            BeaconColor rightColor = robot.getRightColor();
            BeaconColor leftColor = robot.getLeftColor();

            telemetry.addData("LEFT "," R%d  G%d  B%d  H%.0f", leftRed, leftGreen, leftBlue, leftHSV[0]);
            telemetry.addData("LEFT "," COLOR = %s", leftColor.toString());
            telemetry.addData("RIGHT"," R%d  G%d  B%d  H%.0f", rightRed, rightGreen, rightBlue, rightHSV[0]);
            telemetry.addData("RIGHT"," COLOR = %s", rightColor.toString());
            telemetry.update();

            et.reset();
        }
    }

}
