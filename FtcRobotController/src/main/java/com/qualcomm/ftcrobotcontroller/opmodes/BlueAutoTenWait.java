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
public class BlueAutoTenWait extends LinearOpMode {
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

//106 148 Drive the robot parking it in front of the beacon new update

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        String colorfound = "none";
        int Control = 1;
        waitForStart();
        while (Control == 1) {

            double ArmUp = .3;
            double ArmLift = .3;

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

            sleep(10000);

            oneArmMotor.setPower(.48); //foward
            upMiddleMotor.setPower(-.5);
            sleep(1975);

            oneArmMotor.setPower(.10);
            upMiddleMotor.setPower(-.10);
            sleep(50);

            oneArmMotor.setPower(.23);
            upMiddleMotor.setPower(-.25);
            sleep(460);

            oneArmMotor.setPower(-.7); //turn about 85-95°
            upMiddleMotor.setPower(-.7); //// TODO: 2/21/2016 Always check motor wheel coneection if not checked it will effect turn if loose!!!!
            sleep(790);

            oneArmMotor.setPower(.23); //foward
            upMiddleMotor.setPower(-.25);
            sleep(2270);

            oneArmMotor.setPower(.08);
            upMiddleMotor.setPower(-.10);
            sleep(750);

            oneArmMotor.setPower(0);
            upMiddleMotor.setPower(0);
            sleep(100);

            oneArmMotor.setPower(.23);
            upMiddleMotor.setPower(-.25);
            sleep(260);

            oneArmMotor.setPower(0);
            upMiddleMotor.setPower(0);
            sleep(100);
            telemetry.clearData();

            // TODO: 2/19/2016 Break this is the end of the drive the robot is now parked in front of the color beacon and is ready to press the button and dump thy climbers.
            twoArmMotor.setPower(ArmLift);
            sleep(300);
            twoArmMotor.setPower(0);

            rightMotor.setPower(ArmUp);
            leftMotor.setPower(ArmUp);
            sleep(650);
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            twoArmMotor.setPower(ArmLift);
            sleep(450);
            twoArmMotor.setPower(0);

            rightMotor.setPower(ArmUp);
            leftMotor.setPower(ArmUp);
            sleep(1000);
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            rightMotor.setPower(ArmUp);
            leftMotor.setPower(ArmUp / 2);
            sleep(1600);
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            twoArmMotor.setPower(ArmLift);
            sleep(425);
            twoArmMotor.setPower(0);

            rightMotor.setPower(ArmUp);
            leftMotor.setPower(ArmUp);
            sleep(1200);
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            turnServo.setPosition(1);
            dumpServo.setPosition(1);
            sleep(900);
            turnServo.setPosition(.5);

            twoArmMotor.setPower(-ArmLift);
            sleep(150);
            twoArmMotor.setPower(0);

            rightMotor.setPower(ArmUp/2);
            leftMotor.setPower(ArmUp);
            sleep(300);
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            dumpServo.setPosition(0);
            sleep(500);

            telemetry.addData("Dump finished", "");
            sleep(100);
            Control = 4;

            while(Control == 4){
                DbgLog.msg("The robot is currently stoped and just running debuging info.");
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                upMiddleMotor.setPower(0);
                oneArmMotor.setPower(0);
                twoArmMotor.setPower(0);
                threeArmMotor.setPower(0);

                turnServo.setPosition(.5);

                telemetry.addData("Clear", sensorRGB.alpha());
                telemetry.addData("Red  ", sensorRGB.red());
                telemetry.addData("Green", sensorRGB.green());
                telemetry.addData("Blue ", sensorRGB.blue());
                telemetry.addData("Hue", HSVTest[0]);
                telemetry.addData("Saturation", HSVTest[1]);
                telemetry.addData("Value", HSVTest[2]);

                telemetry.addData(colorfound, "");
                waitOneFullHardwareCycle();
            }

            /*
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


            if(HSVTest[0] >= 180 && HSVTest[1] >= 0.5 && HSVTest[2] >= 0.033  ||/tests for red / HSVTest[0] >= 0 && HSVTest[1] >= 1 && HSVTest[2] >= 0.03 && HSVTest[0] <= 50) && blue >= 5 && green >=1 && clear >=2 && green <= 15 && clear <= 10 && red >= 0 && red <= 2) //Tests for blue
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
            //  Control = 2;

//
            //    }

            //  }




            //   else
         /*   {
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

      /*  while (Control == 2){
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

/*
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
                    while(Control == 4){
                DbgLog.msg("Robot has stopped and is now in debug mode");
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                upMiddleMotor.setPower(0);
                oneArmMotor.setPower(0);
                twoArmMotor.setPower(0);
                threeArmMotor.setPower(0);
                telemetry.addData("Clear", sensorRGB.alpha());
                telemetry.addData("Red  ", sensorRGB.red());
                telemetry.addData("Green", sensorRGB.green());
                telemetry.addData("Blue ", sensorRGB.blue());
                telemetry.addData("Hue", HSVTest[0]);
                telemetry.addData("Saturation", HSVTest[1]);
                telemetry.addData("Value", HSVTest[2]);
            }






        */
            telemetry.addData(colorfound, "");
            waitOneFullHardwareCycle();
        }
    }
}
//