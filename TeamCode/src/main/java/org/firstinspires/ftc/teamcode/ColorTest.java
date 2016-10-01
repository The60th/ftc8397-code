

package org.firstinspires.ftc.teamcode;
        import android.content.Loader;
        import android.graphics.Color;
        import android.media.AudioManager;
        import android.media.SoundPool;

        import com.qualcomm.hardware.ams.AMSColorSensor;
        import com.qualcomm.robotcore.eventloop.opmode.*;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.I2cAddr;
        import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
        import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name=" ColorTest : Double Sensor ", group="Test")
public class ColorTest extends OpMode {
    OpticalDistanceSensor odsSensor;
    ColorSensor sensorRGB;
    ColorSensor sensorRGB2;
    TouchSensor touchSensor;  // Hardware Device Object
    public SoundPool mySound;
    public int beepID;
    public double baseLine;
    public double odcValue;


    public void init() {
        hardwareMap.logDevices();
        sensorRGB = hardwareMap.colorSensor.get("mr");
        sensorRGB2 = hardwareMap.colorSensor.get("mr2");
        touchSensor=hardwareMap.touchSensor.get("mrT");
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");

        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        //mysound2 = new SoundPool()
        beepID = mySound.load(hardwareMap.appContext, R.raw.nxtstartupsound, 1); // PSM
       // I2cAddr test = 0x03;
        sensorRGB2.setI2cAddress(I2cAddr.create8bit(0x70));

        sensorRGB.enableLed(true);
        sensorRGB2.enableLed(true);
        baseLine = odsSensor.getRawLightDetected();

    }
    @Override
    public void loop() {

        //sensorRGB.setI2cAddress(t, 0x70);
        String colorfound = "none";
        double blue = sensorRGB.blue();
        double red = sensorRGB.red();
        double clear = sensorRGB.alpha();
        double green = sensorRGB.green();

        double blue2 = sensorRGB2.blue();
        double red2 = sensorRGB2.red();
        double clear2 = sensorRGB2.alpha();
        double green2 = sensorRGB2.green();

        odcValue = odsSensor.getRawLightDetected();

        float[] HSVTest = {0F, 0F, 0F};
        float[] HSVTest2 = {0F, 0F, 0F};
        Color.RGBToHSV(sensorRGB.red() * 8, sensorRGB.green() * 8, sensorRGB.blue() * 8, HSVTest);
        Color.RGBToHSV(sensorRGB2.red() * 8, sensorRGB2.green() * 8, sensorRGB2.blue() * 8, HSVTest2);;

        telemetry.addData("Clear", sensorRGB.alpha());
        telemetry.addData("Red  ", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue ", blue);
        telemetry.addData("Hue", HSVTest[0]);
        telemetry.addData("Saturation", HSVTest[1]);
        telemetry.addData("Value", HSVTest[2]);

        telemetry.addData("Clear2", sensorRGB2.alpha());
        telemetry.addData("Red2  ", red2);
        telemetry.addData("Green2", green2);
        telemetry.addData("Blue2 ", blue2);
        telemetry.addData("Hue2", HSVTest2[0]);
        telemetry.addData("Saturation2", HSVTest2[1]);
        telemetry.addData("Value2", HSVTest2[2]);

        telemetry.addData("Base line values of ods is ",baseLine);
        telemetry.addData("Current value of ods is ",odcValue);

        if (touchSensor.isPressed()) {
            sensorRGB2.enableLed(false);
            telemetry.addData("Touch", "Is Pressed");
            sensorRGB.enableLed(true);
            for (int i =0; i<=2; ){
            mySound.play(beepID,1,1,1,0,1);
                i++;
             }
        }
        else {
            sensorRGB2.enableLed(true);
            telemetry.addData("Touch", "Is Not Pressed");
            sensorRGB.enableLed(false);






        }



        this.updateTelemetry(telemetry);

    }
}
