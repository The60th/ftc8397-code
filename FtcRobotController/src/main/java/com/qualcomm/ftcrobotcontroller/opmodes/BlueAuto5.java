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
public class BlueAuto5 extends LinearOpMode {
    ColorSensor sensorRGB;
    DcMotor leftMotor;
    DcMotor rightMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap.logDevices();
        sensorRGB = hardwareMap.colorSensor.get("mr");
        sensorRGB.enableLed(false);
        leftMotor = hardwareMap.dcMotor.get("LM1");
        rightMotor = hardwareMap.dcMotor.get("RM1");
        String colorfound = "none";
        waitForStart();
        int Control = 1;
        while (Control == 1) {
            double blue = sensorRGB.blue();
            double red = sensorRGB.red();
            double clear = sensorRGB.alpha();
            double green = sensorRGB.green();
            float []HSVTest ={0F,0F,0F};
            sensorRGB.enableLed(false);
            Color.RGBToHSV(sensorRGB.red() * 8, sensorRGB.green() * 8, sensorRGB.blue() * 8, HSVTest);
            telemetry.addData("Clear", sensorRGB.alpha());
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("Hue", HSVTest[0]);
            telemetry.addData("Saturation", HSVTest[1]);
            telemetry.addData("Value", HSVTest[2]);

            /*rightMotor.setPower(.5); //foward
            leftMotor.setPower(-.45);
            sleep(2750);


            rightMotor.setPower(-.2); //turn about 85-95°
            leftMotor.setPower(-.2);
            sleep(1050);


            rightMotor.setPower(.5); //foward
            leftMotor.setPower(-.45);
            sleep(3200);


            rightMotor.setPower(0);
            leftMotor.setPower(0);
            sleep(2000);
            telemetry.clearData();*/

            rightMotor.setPower(0);
            leftMotor.setPower(0);
            sleep(2000);
            if(HSVTest[0] >= 180 && HSVTest[1] >= 0.5 && HSVTest[2] >= 0.033  || HSVTest[0] >= 0 && HSVTest[1] >= 1 && HSVTest[2] >= 0.03 && HSVTest[0] <= 50){
                colorfound = "blue or red";
                telemetry.addData("Color found:",colorfound);

            }
            telemetry.addData("Clear", sensorRGB.alpha());
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("Hue", HSVTest[0]);
            telemetry.addData("Saturation", HSVTest[1]);
            telemetry.addData("Value", HSVTest[2]);

            sleep(2000);



            //break the robot is parked about a foot from the color sensor


            if(HSVTest[0] >= 180 && HSVTest[1] >= 0.5 && HSVTest[2] >= 0.033  ||/*tests for red */ HSVTest[0] >= 0 && HSVTest[1] >= 1 && HSVTest[2] >= 0.03 && HSVTest[0] <= 50 ) // && blue >= 5 && green >=1 && clear >=2 && green <= 15 && clear <= 10 && red >= 0 && red <= 2) //Tests for blue
            {
                colorfound = "blue or red";
                rightMotor.setPower(.5); //foward
                leftMotor.setPower(-.5);
                sleep(200);                        //a button press?
                rightMotor.setPower(0); //stop
                leftMotor.setPower(0);
                sleep(1500);
                telemetry.addData("Color found:", colorfound);

                if (/*HSVTest[0] >= 180 && HSVTest[1] >= 0.5 && HSVTest[2] >= 0.033*/HSVTest[0] >= 0 && HSVTest[1] >= 1 && HSVTest[2] >= 0.03 && HSVTest[0] <= 50) {
                    colorfound = "red";
                    rightMotor.setPower(.5);
                    leftMotor.setPower(-.5);
                    sleep(150);
                    telemetry.addData("Color found:", colorfound);
                    sleep(2000);
                    Control = 2;

                }
                //break the robot has found blue and pushed the button
            }

                else{
                //drive to find blue
                    colorfound = "blue"; //No Button Press Wrong Color.
                        telemetry.addData("Color found:",colorfound);
                     //rightMotor.setPower(-.5);
                      //  leftMotor.setPower(.5);
                       // sleep(890);

                        rightMotor.setPower(.2); //turn about 85-95°
                        leftMotor.setPower(.2);
                        sleep(990);

                        rightMotor.setPower(.5);
                        leftMotor.setPower(-.5);
                        sleep(300);

                        rightMotor.setPower(-.2); //turn about 85-95°
                        leftMotor.setPower(-.2);
                        sleep(990);

                        rightMotor.setPower(.5);
                        leftMotor.setPower(-.5);
                        sleep(300);
                    //break parks the robot in front of the color beacon
                            telemetry.addData("Color found:", colorfound);
                            rightMotor.setPower(.5);
                            leftMotor.setPower(-.5);
                            sleep(300);

                            rightMotor.setPower(0);
                            leftMotor.setPower(0);
                            sleep(2000);
                            Control = 4;








                }



            }

        while (Control == 2){
            telemetry.addData("Control is equal to:", Control);
            rightMotor.setPower(0);
            leftMotor.setPower(0);
            sleep(1000);

            rightMotor.setPower(-.5);
            leftMotor.setPower(.5);
            sleep(500);


            rightMotor.setPower(-.2); //turn about 85-95°
            leftMotor.setPower(-.2);
            sleep(1200);

            rightMotor.setPower(.5);
            leftMotor.setPower(-.5);
            sleep(2150);

            rightMotor.setPower(.2); //turn about 85-95°
            leftMotor.setPower(.2);
            sleep(950);

            rightMotor.setPower(0);
            leftMotor.setPower(0);
            sleep(1000);


        } //drive to ramp from first side goes here


        while (Control == 4) {
            telemetry.addData("Control is equal to:", Control);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(1000);

            rightMotor.setPower(-.5);
            leftMotor.setPower(.5);
            sleep(500);


            rightMotor.setPower(-.2); //turn about 85-95°
            leftMotor.setPower(-.2);
            sleep(1200);

            rightMotor.setPower(.5);
            leftMotor.setPower(-.5);
            sleep(2350);

            rightMotor.setPower(.2); //turn about 85-95°
            leftMotor.setPower(.2);
            sleep(900);

            rightMotor.setPower(0);
            leftMotor.setPower(0);
            sleep(1000);


        }

            telemetry.addData(colorfound,"");
            waitOneFullHardwareCycle();
        }
    }
//}
