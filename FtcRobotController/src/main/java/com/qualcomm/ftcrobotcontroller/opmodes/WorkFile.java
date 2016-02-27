
package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class WorkFile extends LinearOpMode {

    //Here we first have a set of variable deculations for the different DC and Servo motors we will be using thought out the program.

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor upMiddleMotor;
    DcMotor threeArmMotor;
    DcMotor oneArmMotor;
    DcMotor twoArmMotor;
    Servo turnServo;
    Servo dumpServo;


    @Override
    public void runOpMode() throws InterruptedException {

        hardwareMap.logDevices();


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

        int Control = 1;
        waitForStart();
        while (Control == 1) {
            //Here we have a few variables that will be used later on in the program to control the power of different arms and wheels.
            double ArmUp = .3;
            double ArmLift = .3;
            Control = 6;
            while (Control == 6) {
                telemetry.addData("In while loop 6", "");

                threeArmMotor.setPower(ArmLift);
                sleep(150);
                threeArmMotor.setPower(0);

                oneArmMotor.setPower(.10);
                upMiddleMotor.setPower(-.10);
                sleep(850);

                oneArmMotor.setPower(0);
                upMiddleMotor.setPower(0);
                sleep(150);

                oneArmMotor.setPower(-.7); //turn about 85-95°
                upMiddleMotor.setPower(-.7); //// TODO: 2/21/2016 Always check motor wheel coneection if not checked it will effect turn if loose!!!!
                sleep(290);

                oneArmMotor.setPower(.25);
                upMiddleMotor.setPower(-.27);
                sleep(2890);

                oneArmMotor.setPower(.10);
                upMiddleMotor.setPower(-.10);
                sleep(250);

                oneArmMotor.setPower(.23);
                upMiddleMotor.setPower(-.25);
                sleep(760);

                oneArmMotor.setPower(0);
                upMiddleMotor.setPower(0);
                sleep(55);


                oneArmMotor.setPower(-.7); //turn about 85-95°
                upMiddleMotor.setPower(-.7); //// TODO: 2/21/2016 Always check motor wheel coneection if not checked it will effect turn if loose!!!!
                sleep(400);

                oneArmMotor.setPower(.15);
                upMiddleMotor.setPower(-.15);
                sleep(1225);
                oneArmMotor.setPower(0);
                upMiddleMotor.setPower(0);


                threeArmMotor.setPower(ArmLift);
                sleep(100);
                threeArmMotor.setPower(0);

                rightMotor.setPower(ArmUp);
                leftMotor.setPower(ArmUp);
                sleep(670);
                rightMotor.setPower(0);
                leftMotor.setPower(0);

                threeArmMotor.setPower(ArmLift);
                sleep(450);
                threeArmMotor.setPower(0);

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

                turnServo.setPosition(1);
                dumpServo.setPosition(1);
                sleep(1350);
                turnServo.setPosition(.5);

                //threeArmMotor.setPower(-ArmLift);
                //sleep(150);
                //twoArmMotor.setPower(0);

                rightMotor.setPower(ArmUp / 2);
                leftMotor.setPower(ArmUp);
                sleep(375);
                rightMotor.setPower(0);
                leftMotor.setPower(0);


                dumpServo.setPosition(0);
                sleep(500);

                telemetry.addData("Dump finished", "");
                sleep(100);




                Control = 4;
                waitOneFullHardwareCycle();

            }
            while (Control == 4) {
                DbgLog.msg("The robot is currently stoped and just running debuging info.");
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                upMiddleMotor.setPower(0);
                oneArmMotor.setPower(0);
                threeArmMotor.setPower(0);
                twoArmMotor.setPower(0);
                turnServo.setPosition(.5);
                dumpServo.setPosition(1);


                waitOneFullHardwareCycle();


            }
        }
    }
}
