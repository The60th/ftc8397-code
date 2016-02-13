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


import android.graphics.Color;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
// ^^ check this out? File path is com>qualcomm>robotcore>util for documents on this import to find diffrent commands?

import java.util.Random;

//// TODO: 2/13/2016 Idea on drive slow down? Run off a set double value with a clock that lowers the values per second of run time?
//// TODO: 2/13/2016 Also rewrite all motor commands so the names make sense on how the work, aka leftmotor driving the left not the right and so on.
public class ColorTest extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor upMiddleMotor;
    DcMotor threeArmMotor;
    DcMotor oneArmMotor;
    DcMotor twoArmMotor;
    Servo turnServo;
    Servo dumpServo;
    ColorSensor sensorRGB;

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap.logDevices();
        sensorRGB = hardwareMap.colorSensor.get("mr");
        sensorRGB.enableLed(false);

        leftMotor = hardwareMap.dcMotor.get("LM1");
        rightMotor = hardwareMap.dcMotor.get("RM1");

        upMiddleMotor = hardwareMap.dcMotor.get("MM1");//controller two drive wheels
        oneArmMotor = hardwareMap.dcMotor.get("AM1");//controller two drive wheel

        threeArmMotor = hardwareMap.dcMotor.get("AM3");//Controller three
        twoArmMotor = hardwareMap.dcMotor.get("AM2");//controller three

        turnServo = hardwareMap.servo.get("TS1");//Servo controller one
        dumpServo = hardwareMap.servo.get("DS1"); //Servo controller one


        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        String colorfound = "none";
        int Control = 1;
        waitForStart();
        while (Control == 1) {

            double ArmUp = .25;
            double ArmLift = .25; //Update values to reflect the real world on what is needed

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

            oneArmMotor.setPower(.5); //foward
            upMiddleMotor.setPower(-.5);
            sleep(1850);
            oneArmMotor.setPower(.25);
            upMiddleMotor.setPower(-.25);
            sleep(370);

            telemetry.addData("Turn Servo's current position is:", turnServo.getPosition());
            telemetry.addData("Dump Servo's current position is:", dumpServo.getPosition());
            telemetry.addData("left motor power:", leftMotor.getPower());
            telemetry.addData("MM1 power:", upMiddleMotor.getPower());
            telemetry.addData("right motor power:", rightMotor.getPower());
            telemetry.addData("1AM power:", oneArmMotor.getPower());
            telemetry.addData("2AM power:", twoArmMotor.getPower());
            telemetry.addData("3AM:", threeArmMotor.getPower());
            //sleep(2500);


            oneArmMotor.setPower(-.5); //turn about 85-95°
            upMiddleMotor.setPower(-.5);
            sleep(925);


            oneArmMotor.setPower(.25); //foward
            upMiddleMotor.setPower(-.25);
            sleep(2450);

            oneArmMotor.setPower(.10);
            upMiddleMotor.setPower(-.10);
            sleep(550);

            oneArmMotor.setPower(0);
            upMiddleMotor.setPower(0);
            sleep(250);

            oneArmMotor.setPower(.25);
            upMiddleMotor.setPower(-.25);
            sleep(150);

            oneArmMotor.setPower(0);
            upMiddleMotor.setPower(0);
            sleep(2000);
            telemetry.clearData();

            telemetry.addData("Clear", sensorRGB.alpha());
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("Hue", HSVTest[0]);
            telemetry.addData("Saturation", HSVTest[1]);
            telemetry.addData("Value", HSVTest[2]);

            oneArmMotor.setPower(0);
            upMiddleMotor.setPower(0);
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
                oneArmMotor.setPower(.10); //foward
                upMiddleMotor.setPower(-.10);
                sleep(200);                        //a button press?
                oneArmMotor.setPower(0); //stop
                upMiddleMotor.setPower(0);
                sleep(1500);
                telemetry.addData("Color found:", colorfound);

                if (HSVTest[0] >= 0 && HSVTest[1] >= 1 && HSVTest[2] >= 0.03 && HSVTest[0] <= 50) {
                    colorfound = "red";
                    oneArmMotor.setPower(.20);
                    upMiddleMotor.setPower(-.20);
                    sleep(150);
                    telemetry.addData("Color found:", colorfound);
                    sleep(2000);

                    threeArmMotor.setPower(ArmLift);
                    sleep(250);
                    //upMiddleMotor.setPower(ArmUp);
                    //oneArmMotor.setPower(ArmUp);
                    //sleep(575);
                   // dumpServo.setPosition(0);
                    //sleep(500);
                    /*
                    *
                    *
                    *
                    *
                    *
                    * TODO: 2/9/2016 add the controls to dump from the arm here
                    */
                    Control = 2;


                }

            }




                else
            {
                //drive to find blue
                    colorfound = "blue"; //No Button Press Wrong Color.
                        telemetry.addData("Color found:",colorfound);
                     //rightMotor.setPower(-.5);
                      //  leftMotor.setPower(.5);
                       // sleep(890);

                //// TODO: 2/13/2016 currently running right here
                        oneArmMotor.setPower(.2); //turn about 85-95°
                        upMiddleMotor.setPower(.2);
                        sleep(990);

                        rightMotor.setPower(.5);
                upMiddleMotor.setPower(-.5);
                        sleep(300);

                oneArmMotor.setPower(-.2); //turn about 85-95°
                upMiddleMotor.setPower(-.2);
                        sleep(990);

                oneArmMotor.setPower(.5);
                upMiddleMotor.setPower(-.5);
                        sleep(300);
                    //break parks the robot in front of the color beacon
                            telemetry.addData("Color found:", colorfound);
                oneArmMotor.setPower(.5);
                upMiddleMotor.setPower(-.5);
                            sleep(300);

                oneArmMotor.setPower(0);
                upMiddleMotor.setPower(0);
                            sleep(2000);
                // TODO: 2/9/2016 add the controls to dump from the arm here
                            Control = 3;
                }
            }

        while (Control == 2){
            telemetry.addData("Control is equal to:", Control);
            oneArmMotor.setPower(0);
            upMiddleMotor.setPower(0);
            sleep(1000);

            oneArmMotor.setPower(-.5);
            upMiddleMotor.setPower(.5);
            sleep(500);


            oneArmMotor.setPower(-.2); //turn about 85-95°
            upMiddleMotor.setPower(-.2);
            sleep(1200);

            oneArmMotor.setPower(.5);
            upMiddleMotor.setPower(-.5);
            sleep(2150);

            oneArmMotor.setPower(.2); //turn about 85-95°
            upMiddleMotor.setPower(.2);
            sleep(950);

            oneArmMotor.setPower(0);
            upMiddleMotor.setPower(0);
            sleep(1000);


        } //drive to ramp from first side goes here


        while (Control == 3) {
            telemetry.addData("Control is equal to:", Control);
            upMiddleMotor.setPower(0);
            oneArmMotor.setPower(0);
            sleep(1000);

            oneArmMotor.setPower(-.5);
            upMiddleMotor.setPower(.5);
            sleep(500);


            oneArmMotor.setPower(-.2); //turn about 85-95°
            upMiddleMotor.setPower(-.2);
            sleep(1200);

            oneArmMotor.setPower(.5);
            upMiddleMotor.setPower(-.5);
            sleep(2350);

            oneArmMotor.setPower(.2); //turn about 85-95°
            upMiddleMotor.setPower(.2);
            sleep(900);

            oneArmMotor.setPower(0);
            upMiddleMotor.setPower(0);
            sleep(1000);

            Control = 5; //updated just to end the loop to stop forever looping.
        }
            telemetry.addData(colorfound,"");
            waitOneFullHardwareCycle();
        }
    }
//}
