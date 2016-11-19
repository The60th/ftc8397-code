

package org.firstinspires.ftc.teamcode;
        import android.graphics.Color;
        import android.media.SoundPool;

        import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
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
    ModernRoboticsI2cGyro gyro;
    public SoundPool mySound;
    public int beepID;
    public double baseLine;
    public double odsValue;
    int xVal, yVal, zVal = 0;     // Gyro rate Values
    int heading = 0;              // Gyro integrated heading
    int angleZ = 0;


    public void init() {
        hardwareMap.logDevices();
        sensorRGB = hardwareMap.colorSensor.get("mr");
        sensorRGB2 = hardwareMap.colorSensor.get("mr2");
        touchSensor=hardwareMap.touchSensor.get("mrT");
        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        sensorRGB2.setI2cAddress(I2cAddr.create8bit(0x70));

        sensorRGB.enableLed(true);
        sensorRGB2.enableLed(true);
        baseLine = odsSensor.getRawLightDetected();
        

        telemetry.addData(">", "Gyro Calibrating. Do not move!");
        telemetry.update();
        //gyro.calibrate();

        // make sure the gyro is calibrated.
       /* while (gyro.isCalibrating())  {
            //Thread.sleep(50);
            telemetry.addData(">", "Gyro Calibrating. Do Not move!");
            telemetry.update();
            //idle();
        }*/

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();
    }
    @Override
    public void loop() {

        String colorfound = "none";

        double blue = sensorRGB.blue();
        double red = sensorRGB.red();
        int clear = sensorRGB.alpha();
        double green = sensorRGB.green();
        //sensorRGB.
        double blue2 = sensorRGB2.blue();
        double red2 = sensorRGB2.red();
        double clear2 = sensorRGB2.alpha();
        double green2 = sensorRGB2.green();
        odsValue = odsSensor.getRawLightDetected();
        double odsValueScaled = odsSensor.getLightDetected();

        int test;
        float[] HSVTest = {0, 0, 0};
        float[] HSVTest2 = {0F, 0F, 0F};
        Color.RGBToHSV(sensorRGB.red(), sensorRGB.green(), sensorRGB.blue(), HSVTest);
        Color.RGBToHSV(sensorRGB2.red() * 8, sensorRGB2.green() * 8, sensorRGB2.blue() * 8, HSVTest2);

        test = Color.HSVToColor(sensorRGB.alpha(),HSVTest);
        int test2 = 170;
        int alpha = (test);
        //test = Color.HSVToColor(clear,HSVTest);

        String taco = Integer.toHexString(test);
        telemetry.addData("HexString?: ", Integer.toHexString(test2));

        telemetry.addData("Clear", sensorRGB.alpha());
        telemetry.addData("Red  ", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue ", blue);
        telemetry.addData("Hue", HSVTest[0]);
        telemetry.addData("Saturation", HSVTest[1]);
        telemetry.addData("Value", HSVTest[2]);

        telemetry.addData("True color value testing: ",test);
        telemetry.addData("True color value testing AA: ",alpha);
        telemetry.addData("Clear2", sensorRGB2.alpha());
        telemetry.addData("Red2  ", red2);
        telemetry.addData("Green2", green2);
        telemetry.addData("Blue2 ", blue2);
        telemetry.addData("Hue2", HSVTest2[0]);
        telemetry.addData("Saturation2", HSVTest2[1]);
        telemetry.addData("Value2", HSVTest2[2]);

        telemetry.addData("Base line values of ods is ",baseLine);
        telemetry.addData("Current value of ods is ",odsValue);
        telemetry.addData("Current scaled value of ods is:",odsValueScaled);


        xVal = gyro.rawX();
        yVal = gyro.rawY();
        zVal = gyro.rawZ();

        // get the heading info.
        // the Modern Robotics' gyro sensor keeps
        // track of the current heading for the Z axis only.
        heading = gyro.getHeading();
        angleZ  = gyro.getIntegratedZValue();

        telemetry.addData(">", "Press A & B to reset Heading.");
        telemetry.addData("0", "Heading %03d", heading);
        telemetry.addData("1", "Int. Ang. %03d", angleZ);
        telemetry.addData("2", "X av. %03d", xVal);
        telemetry.addData("3", "Y av. %03d", yVal);
        telemetry.addData("4", "Z av. %03d", zVal);



        if (touchSensor.isPressed()) {
            sensorRGB2.enableLed(false);
            telemetry.addData("Touch", "Is Pressed");
            sensorRGB.enableLed(false);
            /*for (int i =0; i<=2; ){
            mySound.play(beepID,1,1,1,0,1);
                i++;
             }*/
            if(!( (heading >= 0&&heading <= 10) || (heading <= 360 && heading>=350) )){
                telemetry.addData("Uh oh!","Looks like we got turned around! Try and turn is back!");
                telemetry.update();
            }
            else{
                telemetry.addData("We holding steady at a heading of:", heading);
            }
        }
        else {
            sensorRGB2.enableLed(true);
            telemetry.addData("Touch", "Is Not Pressed");
            sensorRGB.enableLed(true);
        }
        //White Line    //was .601, min lowered to .551 for better chances
        if((odsValue >= .551) && (odsValue <= .757)){ //Within the max and min values for the whiteline we have found in testing.
            telemetry.addData("White line has been found."," ");

        }




        this.updateTelemetry(telemetry);

    }
}

