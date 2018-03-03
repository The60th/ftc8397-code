package org.firstinspires.ftc.teamcode.mechbot.supers_bot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.i2c.BNO055Enhanced;
import org.firstinspires.ftc.teamcode.i2c.BNO055EnhancedImpl;

/**
 * Created by FTC Team 8397 on 3/1/2018.
 */

public class MechBotGyro extends MechBotJewelArm {
    public BNO055EnhancedImpl imu;

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
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

    public void gyroTelemetry(Telemetry telemetry){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("ZYX: ","Z: " + String.format("%.2f",angles.firstAngle) + " Y " + String.format("%.2f",angles.secondAngle) + " X " + String.format("%.2f",angles.thirdAngle));
    }
}
