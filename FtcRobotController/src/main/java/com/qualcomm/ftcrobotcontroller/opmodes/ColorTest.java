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
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

import java.util.Random;


public class ColorTest extends LinearOpMode {

    ColorSensor sensorRGB;
    //DcMotor rightRearMotor;
    //DcMotor leftRearMotor;
    DcMotor leftMotor;
    DcMotor rightMotor;
                                           //NOTICE THIS SHOULD WORK AND OUTPUTS THE VALUES OF RED AS: HSV 0,1,1 AND BLUE AS 240,1,1

    @Override

    public void runOpMode() throws InterruptedException {
        hardwareMap.logDevices();
        sensorRGB = hardwareMap.colorSensor.get("mr");
        sensorRGB.enableLed(false);
        //leftRearMotor =hardwareMap.dcMotor.get("leftRearMotor");
        //rightRearMotor = hardwareMap.dcMotor.get("rightRearMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        rightMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightMotor.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        String colorfound = "none";

        waitOneFullHardwareCycle();
        waitForStart();


        while (opModeIsActive()) {
            /*double leftY = -gamepad1.left_stick_y;
            double rightY = -gamepad1.right_stick_y;
            double UpPower = -.50;
            double Stop =0.0; */
            double blue = sensorRGB.blue();
            double red = sensorRGB.red();
            double clear = sensorRGB.alpha();
            double green = sensorRGB.green();
            float []HSVTest ={0F,0F,0F};
            sensorRGB.enableLed(true);
            Color.RGBToHSV(sensorRGB.red() * 8, sensorRGB.green() * 8, sensorRGB.blue() * 8, HSVTest);
            telemetry.addData("Clear", sensorRGB.alpha());
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("Hue", HSVTest[0]);
            telemetry.addData("Saturation", HSVTest[1]);
            telemetry.addData("Value", HSVTest[2]);

            if(HSVTest[0] >= 225 && HSVTest[1] >= 0.5 && HSVTest[2] >= 0.08 && blue >= 5 && green >=1 && clear >=2 && green <= 15 && clear <= 10 && red >= 0 && red <= 2) //Tests for blue
            {
                colorfound = "blue";
                telemetry.addData("Found Blue", "");
                //rightRearMotor.setPower(-1);
                //leftRearMotor.setPower(-1);
                leftMotor.setPower(1);
                rightMotor.setPower(1);
                sleep(1000);
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                telemetry.addData("Motor has been run","");
                sleep(5000);
                telemetry.clearData();
            }
            else if(HSVTest[0] >= 0 && HSVTest[1] >= 1 && HSVTest[2] >= 0.3 && red >= 8 && green >=3 && clear >=10 && green <= 7 && clear <= 21 && blue >= 0 && blue <= 2) //Tests for red
            {
                colorfound = "red";
                telemetry.addData("Found Red","");
                //rightRearMotor.setPower(1);
                //leftRearMotor.setPower(1);
                 leftMotor.setPower(-1);
                 rightMotor.setPower(1);
                 sleep(1000);
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                telemetry.addData("Motor has been run","");
                sleep(5000);
                telemetry.clearData();
            }
            else
            {
                colorfound = "none";
                telemetry.addData("No","Colors");
            }

            telemetry.addData(colorfound,"");
        }
    }
}
