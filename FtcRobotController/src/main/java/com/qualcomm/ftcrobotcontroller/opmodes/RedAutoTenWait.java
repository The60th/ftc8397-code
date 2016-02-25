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

//This program has been made by members of the FTC Team, Beta8397 for the 2016 competition year and the challenge First-Resq this is only one of many
//programs team Beta's software development team worked on through the year, this one in practicality is the program we will use for competition autonomous.


public class RedAutoTenWait extends LinearOpMode {

    //Here we first have a set of variable deculations for the different DC and Servo motors we will be using thought out the program.

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor upMiddleMotor;
    DcMotor threeArmMotor;
    DcMotor oneArmMotor;
    DcMotor twoArmMotor;
    Servo turnServo;
    Servo dumpServo;

    //Something new to are autonomous programs not seen in teleop is a color sensor. Here we have where we are first declaring that we will be using
    //a color sensor, the one we ended up using is the Modern Robotics Color Sensor.
    ColorSensor sensorRGB;

    @Override
    public void runOpMode() throws InterruptedException {
        //Here we have a large list of commands the are naming the motors(Servo and DC) and color sensors we plan to use through out this program.
        //This also sets the name for the different motors that are check for in the configuration files.

        hardwareMap.logDevices();
        //sensorRGB = hardwareMap.colorSensor.get("mr");

        //The line right below this is setting the LED light on our color sensor to false(Turned off) this is so the color sensor picks up the color of
        //lights rather then shining a light to find the color of something like tape or paint ect.
        //sensorRGB.enableLed(false);

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

        //Here we are declaring a few more variables such as one of them is the int "Control" and the string Colorfound. The reasons for them are:
        //Color found updates depending on what colors are sensor picks up then prints what it finds back to use so we know what we are looking at.
        //The control variable is used for controlling while loops through the program, while this is not does as well as it could be it was fast and
        //easy and meet the demands we needed at the time.
        //Also here is a waitForStart line what this does is it will not let the program contuine past this till it is given the command to start.

        //String colorfound = "none";
        int Control = 1;
        waitForStart();
        while (Control == 1) {
            //Here we have a few variables that will be used later on in the program to control the power of different arms and wheels.
            double ArmUp = .3;
            double ArmLift = .3;
            //Now we have a few variables that update in real-time with the values the color sensor sees for each one. A example of this is if you were
            //looking at a red piece of tape the double red would go very high as the red light output from the color sensor is high.
            //We also have here a float array which is being used to contain the HSV(Hue Saturation Values) from the color sensor.
            //double blue = sensorRGB.blue();
            //double red = sensorRGB.red();
            //double clear = sensorRGB.alpha();
            //double green = sensorRGB.green();
            //float []HSVTest ={0F,0F,0F};

            //Again here we are just making sure the color sensor light is off.
            //sensorRGB.enableLed(false);

            //Now we are converting the values from the color sensor from RGB(Red Green Blue) to HSV to be better used latter in the program.
            //Also we have here a set of telemetry data prints, which what they do is print data from the program to the FTC Driver Station app
            //so that we can watch the values of the color sensor in real time.
            //Color.RGBToHSV(sensorRGB.red() * 8, sensorRGB.green() * 8, sensorRGB.blue() * 8, HSVTest);
            //telemetry.addData("Clear", sensorRGB.alpha());
            //telemetry.addData("Red  ", sensorRGB.red());
            //telemetry.addData("Green", sensorRGB.green());
            //telemetry.addData("Blue ", sensorRGB.blue());
            //telemetry.addData("Hue", HSVTest[0]);
            //telemetry.addData("Saturation", HSVTest[1]);
            //telemetry.addData("Value", HSVTest[2]);

            //We are now moving out of most of the set up for the program and variable declaring and are moving in to the commands for the wheels and
            //arms of the robot. First we can tell you a bit more about are plan for the automouse peroid, are plan is to drive the robot forward to about
            //the middle of the map then make a 90 degree turn and drive forward again being lined up with the color beacon. Which we then inspect the color
            //of the beacon and push the correct button, after pushing the button we park and start moving are arm. The arm will move most of the way out
            //with the turn table spinning then the arm will move out more then dump are climbers in to the goal then is right behind the color beacon.


            //The next few lines(138-165 control the robot driving it forward to about the middle of the map. A key point of this you might notice
            //and ask about is why do we have more then one motor forward command. The reason for this is to give us more control over the robot and also
            //to start to phase down the speed as we move closer to are target so we dont jerk around as much and maybe cause damage to the robot.
            //Another thing you might see is at the end of the move forward we then drive back a bit before turning, the reason for this is so that
            //we can clear any balls or blocks from in front of the robot that may stop it from driving correctly.
            oneArmMotor.setPower(.25);
            upMiddleMotor.setPower(-.27);
            sleep(2850);

            oneArmMotor.setPower(.10);
            upMiddleMotor.setPower(-.10);
            sleep(200);

            oneArmMotor.setPower(.23);
            upMiddleMotor.setPower(-.25);
            sleep(590);

            oneArmMotor.setPower(0);
            upMiddleMotor.setPower(0);
            sleep(250);

            oneArmMotor.setPower(-.18);
            upMiddleMotor.setPower(.20);
            sleep(500);

            oneArmMotor.setPower(0);
            upMiddleMotor.setPower(0);
            sleep(55);

            //Now are robot should be park in about the middle of the mats free from all balls and blocks and ready to turn! The turn we have here works
            //by driving one set up wheels one way at full power at the other way to give is a very fast right turn.

            oneArmMotor.setPower(.7); //turn about 85-95°
            upMiddleMotor.setPower(.7); //// TODO: 2/21/2016 Always check motor wheel coneection if not checked it will effect turn if loose!!!!
            sleep(900);

            //The robot has now driven forward then turned 90degrees and its now ready for its last major step of driving. The robot now drives forward
            //and parks it selfs in front of the color beacon so it can check for colors and dump the climbers. This code here is written the same as the
            //above drive code made to make the robot drive fast at first and slowly bring its speed down to a good stop.
            oneArmMotor.setPower(.23); //foward
            upMiddleMotor.setPower(-.25);
            sleep(2325);

            oneArmMotor.setPower(.08);
            upMiddleMotor.setPower(-.10);
            sleep(800);

            oneArmMotor.setPower(0);
            upMiddleMotor.setPower(0);
            sleep(100);

            oneArmMotor.setPower(.23);
            upMiddleMotor.setPower(-.25);
            sleep(350);

            oneArmMotor.setPower(0);
            upMiddleMotor.setPower(0);
            sleep(100);
            telemetry.clearData();

            //The robot has now finished all major wheel driving for a while it has driven forward, turn then forward again parking its self in front of
            //the color beacon it is now up to the robot to move its arm out and dump the climbers.

            //The arm controls for the robot are a pretty long set of code compared to what we have written so far going from line 200-260.
            //The arm has been one of the harder things for us to program this year with it being so compacted and heavy.
            //The main way the arm works is to first drive the gear box motor to angle the arm so we can move it out without running in to anything.
            //You might notice now that all the powers for the arm are being used as variables not set numbers the reason for this is so that
            //but only changing one line we can change the power for all the arm commands.

            //Lines 207-220 the robot starts by angling the arm upwards a small amount this drive it out more, after is drives out for little over
            //half a second it then brings the angle up even more.
            threeArmMotor.setPower(ArmLift);
            sleep(300);
            threeArmMotor.setPower(0);

            rightMotor.setPower(ArmUp);
            leftMotor.setPower(ArmUp);
            sleep(670);
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            threeArmMotor.setPower(ArmLift);
            sleep(450);
            threeArmMotor.setPower(0);

            //Lines 223-245. This part of the code starts by bringing the arm out even more, and then increasing the angle more so that it can clear
            //the front guards of the robot the robot keeps doing this updating the angle and length of the arm for about 3seconds till it is at
            //the correct spot the robot then stops doing this and moves to the next set of code.
            rightMotor.setPower(ArmUp);
            leftMotor.setPower(ArmUp);
            sleep(1050);
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            rightMotor.setPower(ArmUp);
            leftMotor.setPower(ArmUp / 2);
            sleep(1600);
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            threeArmMotor.setPower(ArmLift);
            sleep(425);
            threeArmMotor.setPower(0);

            rightMotor.setPower(ArmUp);
            leftMotor.setPower(ArmUp);
            sleep(1200);
            rightMotor.setPower(0);
            leftMotor.setPower(0);

            //This covers lines 248-275
            //Here we are now using Servo motors rather then DC motors. A major difference between the two is that servos are self aware of what their current
            //position is while DC motors are not. So to start off we turn the servo motor on the bottom of the robot that is geared and chained to the turn
            //table to give it more power. This then turns the turn table to bring it to the front of the robot so that the arm is now over the scoring
            //bucket for the climbers.
            //After moving the turn table we then drive the arms out just a tad bit more to make sure we are right on top of the bucket, so the climbers
            //always go in, after doing then we then rotate the dump servo which is connected to are bucket to dump the climbers and score them
            //right behind the color beacon.
            turnServo.setPosition(1);
            dumpServo.setPosition(1);
            sleep(900);
            turnServo.setPosition(.5);

            threeArmMotor.setPower(-ArmLift);
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
            //The code here now is just a set of debug info in where we set the power of all motors to 0 and all servos to rest position and then
            //and then bring debugging info to the FTC driver station app.
            while(Control == 4){
                DbgLog.msg("The robot is currently stoped and just running debuging info.");
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                upMiddleMotor.setPower(0);
                oneArmMotor.setPower(0);
                threeArmMotor.setPower(0);
                twoArmMotor.setPower(0);

                turnServo.setPosition(.5);

                //telemetry.addData("Clear", sensorRGB.alpha());
                //telemetry.addData("Red  ", sensorRGB.red());
                //telemetry.addData("Green", sensorRGB.green());
                //telemetry.addData("Blue ", sensorRGB.blue());
                //telemetry.addData("Hue", HSVTest[0]);
                //telemetry.addData("Saturation", HSVTest[1]);
                //telemetry.addData("Value", HSVTest[2]);

                //telemetry.addData(colorfound, "");
                waitOneFullHardwareCycle();
            }
            //Sadly now we have reached the end of the working code we currently have below here you can see many lines of commented non working code, all this
            //was made to control the color sensor to search for different beacon colors and then press then button and hopeful park on the ramp. Sadly because
            //of time and design limit we were unable to get this all working in time for the competition on the 27th. There is a sigh chance that this
            //program may be given a update in time and have all this color sensor code work as it would hopefully scoring us around another 20-40 points.
            //Even while it does not work it has been left as a framework for use to work with and inspect.

            //After reading this all if you have any questions feel free to contact any member of FTC Team Beta8397 for more information on how any of this code
            //all are members would be more then happy to walk you threw on how it works!

            //telemetry.addData(colorfound, "");
            waitOneFullHardwareCycle();
        }
    }
}





//Dead code!



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







            telemetry.addData(colorfound, "");
            waitOneFullHardwareCycle();
        }
    }
} */
//