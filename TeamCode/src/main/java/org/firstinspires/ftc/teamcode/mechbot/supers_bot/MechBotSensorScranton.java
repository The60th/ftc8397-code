package org.firstinspires.ftc.teamcode.mechbot.supers_bot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.i2c.BNO055EnhancedImpl;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

/**
 * Created by FTC Team 8397 on 3/1/2018.
 */

public class MechBotSensorScranton extends MechBot {

    public ColorSensor colorLeft, colorRight, backColorRight, backColorLeft;
    //Change to our custom BNO055 class.
    public BNO055EnhancedImpl imu;

    public float initGyroHeading = 0;

    public float getInitGyroHeadingDegrees() {
        return this.initGyroHeading * 180.0f / (float) Math.PI;
    }

    public float getInitGyroHeadingRadians() {
        return this.initGyroHeading;
    }

    /**
     * Initialize default Hardware interfaces.
     *
     * @param ahwMap passed HardwareMap
     */
    public void init(HardwareMap ahwMap) {

        BetaLog.dd("MechBotSensorScranton: ", "Init this.init HW");

        /**
         * Save passed HardwareMap to local class variable HardwareMap.
         */
        super.init(ahwMap);

        /**
         * Saving all drive wheel motor values to their own string values inside the hardwareMap.
         * Each motor is saved as "M" followed by its number in inger form so Motor One is now "M1".
         */
        // Change made after robot remodle.
        //colorLeft = hardwareMap.get(ColorSensor.class, "sensor_color");
        //colorRight = hardwareMap.get(ColorSensor.class, "sensor_color2");

        //colorRight.setI2cAddress(I2cAddr.create8bit(0x70));

        colorRight = hardwareMap.get(ColorSensor.class, "colorRight");
        colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");

        backColorRight = hardwareMap.get(ColorSensor.class, "backColorRight");
        backColorLeft = hardwareMap.get(ColorSensor.class, "backColorLeft");

        colorLeft.setI2cAddress(I2cAddr.create8bit(0x70));

        imu = hardwareMap.get(BNO055EnhancedImpl.class, "imu");
        BNO055Enhanced.Parameters parameters = new BNO055Enhanced.Parameters();
        parameters.angleUnit = BNO055Enhanced.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055Enhanced.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BN055Cali.json"; // see the calibration sample opmode
        parameters.loggingTag = "IMU";
        parameters.axesMap = BNO055Enhanced.AxesMap.XYZ;
        parameters.axesSign = BNO055Enhanced.AxesSign.PPP;
        imu.initialize(parameters);
    }

    public void init(HardwareMap ahwMap, float initGyroHeadingDegrees) {
        BetaLog.dd("MechBotSensorScranton: ", "Init");
        this.init(ahwMap);
        this.initGyroHeading = initGyroHeadingDegrees * (float) Math.PI / 180.0f;

    }

    public float getHeadingRadians() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        float heading = angles.firstAngle + this.initGyroHeading;
        return (float) VuMarkNavigator.NormalizeAngle(heading);
    }

    public void gyroTelemetry(Telemetry telemetry) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("ZYX: ", "Z: " + String.format("%.2f", angles.firstAngle) + " Y " + String.format("%.2f", angles.secondAngle) + " X " + String.format("%.2f", angles.thirdAngle));
    }

    @Override
    public float getOdomHeadingFromGyroHeading(float gyroHeading) {
        return (float) VuMarkNavigator.NormalizeAngle(gyroHeading - (float) Math.PI);
    }

    @Override
    public float getGyroHeadingFromOdomHeading(float odomHeading) {
        return (float) VuMarkNavigator.NormalizeAngle(odomHeading + (float) Math.PI);
    }
}
