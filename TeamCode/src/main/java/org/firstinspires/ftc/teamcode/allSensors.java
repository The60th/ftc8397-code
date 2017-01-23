package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

import java.lang.reflect.Array;

/**
 * Created by Justin on 11/4/2016.
 */

public class allSensors {
    public String colorSensorOne(ColorSensor yourSensor, boolean enabled){
        //Should  return a color in a string or color variable
        if(enabled){
            yourSensor.enableLed(true);
        }
        else{
            yourSensor.enableLed(false);
        }
        double blue = yourSensor.blue();
        double red = yourSensor.red();
        double clear = yourSensor.alpha();
        double green = yourSensor.green();
        double argb = yourSensor.argb();
        return null;
    }

    public String colorSensorTwo(ColorSensor yourSensor, boolean enabled){
        //Should return a color in a string or color variable
        if(enabled){
            yourSensor.enableLed(true);
        }
        else{
            yourSensor.enableLed(false);
        }
        double blue = yourSensor.blue();
        double red = yourSensor.red();
        double clear = yourSensor.alpha();
        double green = yourSensor.green();
        double argb = yourSensor.argb();
        return null;
    }
    public int[] colorSensorOneRawValues(ColorSensor yourSensor, boolean enabled){
        //Order is argb
        if(enabled){
            yourSensor.enableLed(true);
        }
        else{
            yourSensor.enableLed(false);
        }
        int returnValues[] = new int[5];
        returnValues[0] = yourSensor.alpha();
        returnValues[1] = yourSensor.red();
        returnValues[2] = yourSensor.green();
        returnValues[3] = yourSensor.blue();
        returnValues[4] = yourSensor.argb();
        return  returnValues;
    }

}
