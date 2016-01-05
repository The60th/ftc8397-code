/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.util.Log;
import android.view.View;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;


public class ColorTest extends LinearOpMode {

    ColorSensor sensorRGB;
    DcMotor rightRearMotor;
    DcMotor leftRearMotor;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor UpLeftMotor;
    DcMotor UpRightMotor;


    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap.logDevices();
        sensorRGB = hardwareMap.colorSensor.get("mr");
        sensorRGB.enableLed(false);
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        UpLeftMotor =hardwareMap.dcMotor.get("midLeftMotor");
        UpRightMotor = hardwareMap.dcMotor.get("midRightMotor");
        leftRearMotor =hardwareMap.dcMotor.get("leftRearMotor");
        rightRearMotor = hardwareMap.dcMotor.get("rightRearMotor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        UpRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        waitOneFullHardwareCycle();
        waitForStart();

        //float []ColorValue =  {0F,0F,0F};
        //final float []values2 = ColorValue;

        //float []hsvValues = {0F,0F,0F};
        //final float []values = hsvValues;

        /*while (opModeIsActive()) {
            Color.RGBToHSV(sensorRGB.red() * 8, sensorRGB.green() * 8, sensorRGB.blue() * 8, hsvValues);
            telemetry.addData("Clear", sensorRGB.alpha());
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("Hue", hsvValues[0]);


            Log.d("Test", "This is my test");
            telemetry.addData("Test", "Test"); //Shows message if values ==?

            if(values2 == values)
            {
                Log.w("Test","This is my test");
            telemetry.addData("Test","Test"); //Shows message if values ==?
            }
            waitOneFullHardwareCycle();

            float []HSVTest ={1,1,1};
            Color.RGBToHSV(sensorRGB.red(),sensorRGB.green(),sensorRGB.blue(), HSVTest);

        }*/
        while (opModeIsActive()) {
            double leftY = -gamepad1.left_stick_y;
            double rightY = -gamepad1.right_stick_y;
            double UpPower = -.50;
            double Stop =0.0;
            float []HSVTest ={0F,0F,0F};
            int OutPut = 1; //does nothing just for testing ints in telemetry outputs.
            Color.RGBToHSV(sensorRGB.red()*8, sensorRGB.green()*8, sensorRGB.blue()*8, HSVTest);
            telemetry.addData("Clear", sensorRGB.alpha()); //Is just out puting what the sensor picks up for the color value?
            telemetry.addData("Red  ", sensorRGB.red());   // ^
            telemetry.addData("Green", sensorRGB.green()); // ^
            telemetry.addData("Blue ", sensorRGB.blue()); //^
            telemetry.addData("Hue", HSVTest[0]);      //  Is giving off what the HSV values are now? Like its just converting the RGB stuff
            telemetry.addData("Saturation", HSVTest[1]); // This should return the saturation value
            telemetry.addData("Value", HSVTest[2]);    // This should output the value value
            telemetry.addData("TestOutPut", OutPut);     // to HSV then outputing it in the array HSVTest?
                                                        // Sooo you take the hsv and need to compare whatever you test to that
                                                        // to for using in if statements ect?
            //Notes:
            //Might use just hue from HSV to tell if a color is red/blue because of a range of 0-360? test it later.
            //Need notes of what the sensor picks up in diffrent colors the RBG, HSV ect.
            //
            leftMotor.setPower(rightY / 75);
            leftRearMotor.setPower(rightY / 75);
            rightMotor.setPower(leftY / 75);
            rightRearMotor.setPower(leftY/75);
            telemetry.addData("Both left motors are running at the power of", leftY);
            telemetry.addData("Both right morors are running at the power of", rightY);

            if (gamepad1.left_bumper){
                UpLeftMotor.setPower(UpPower);
                UpRightMotor.setPower(UpPower);
                telemetry.addData("Up left motor running at power",UpPower);
                telemetry.addData("Up right motor running at power",UpPower);
            }
            else{
                UpLeftMotor.setPower(Stop);
                UpRightMotor.setPower(Stop);
                telemetry.addData("Up left motor is currently","not running");
                telemetry.addData("Up right motor is currently","not running");
            }

        }




    }
}
