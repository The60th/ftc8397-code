package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.i2c.BNO055EnhancedImpl;



@SuppressWarnings("all")

public class MechBotSensor extends MechBot
{
    public final double MAX_SENSOR_VALUES = 255.0;
    public float last_Gyro_Theta = 0;


    public ColorSensor sensorMRColor;
    public ColorSensor sensorREVColor;
    public DistanceSensor sensorRevDistance;


    //Change to our custom BNO055 class.
    BNO055EnhancedImpl imu;

    private float initGyroHeading = 0;

    /**
     * Initialize default Hardware interfaces.
     * @param ahwMap passed HardwareMap
     */
    public void init(HardwareMap ahwMap) {


        /**
         * Save passed HardwareMap to local class variable HardwareMap.
         */
        super.init(ahwMap);

        /**
         * Saving all drive wheel motor values to their own string values inside the hardwareMap.
         * Each motor is saved as "M" followed by its number in inger form so Motor One is now "M1".
         */

        sensorMRColor = hardwareMap.get(ColorSensor.class, "sensor_color");

        imu = hardwareMap.get(BNO055EnhancedImpl.class, "imu");
        BNO055Enhanced.Parameters parameters = new BNO055Enhanced.Parameters();
        parameters.angleUnit = BNO055Enhanced.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055Enhanced.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BN055Cali.json"; // see the calibration sample opmode
        parameters.loggingTag = "IMU";
        parameters.axesMap = BNO055Enhanced.AxesMap.ZYX;  //Swap X and Z axe
        parameters.axesSign = BNO055Enhanced.AxesSign.PPN;
        imu.initialize(parameters);
    }
    public void init(HardwareMap ahwMap, float initGyroHeading){
        this.init(ahwMap);
        this.initGyroHeading = initGyroHeading;
    }

    public float getHeadingRadians(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,AngleUnit.RADIANS);
        float heading = angles.firstAngle + this.initGyroHeading;
        return (float)VuMarkNavigator.NormalizeAngle(heading);
    }
}

