package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
//Created by CanAdirondack on 11/25/2015.

public class allterrain extends OpMode
{

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor upMiddleMotor;
    DcMotor threeArmMotor;
    DcMotor oneArmMotor;
    DcMotor twoArmMotor;
    Servo turnServo;
    Servo dumpServo;
    @Override
    public void init()
    {
        leftMotor = hardwareMap.dcMotor.get("LM1"); //controler one
        rightMotor = hardwareMap.dcMotor.get("RM1"); //controller one

        upMiddleMotor = hardwareMap.dcMotor.get("MM1");//controller two
        oneArmMotor = hardwareMap.dcMotor.get("AM1");//controller two

        threeArmMotor = hardwareMap.dcMotor.get("AM3");//Controller three
        twoArmMotor = hardwareMap.dcMotor.get("AM2");//controller three

        turnServo = hardwareMap.servo.get("TS1");//Servo controller one
        dumpServo = hardwareMap.servo.get("DS1"); //Servo controller one


        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

    }
    @Override
    //Notice most comments are out of date currently please ignore.
    public void loop()
    {
        double leftY = gamepad1.left_stick_y/2.5; //2am || 0.4 when joystick at max power
        double rightY = -gamepad1.right_stick_y/2.5;// am
        double JoyOneLeft = gamepad1.left_stick_y/1.25;
        double JoyOneRight = -gamepad1.right_stick_y/1.25;
        double rightX = -gamepad2.left_stick_y/2;  //is driving 1AM          // this power is going to MM1 and rightmotor  RM
        double leftX = -gamepad2.right_stick_y/2;  //is driving MM1         // ^^                                       MM1
        //MM1 and armmotor are comeing in as left and right
         //Left right motor are showing up as two and three for some reason
        leftMotor.setPower(leftX); //runs arm motors as leftmotor
        rightMotor.setPower(rightX); //runs arm motors as right motor
        //Current drive wheel set up.
        //running as MM1
        if(leftY >=.35 && rightY >= -.35  && rightY != 0 || leftY <= -.35 && rightY <= -.35 && rightY != 0){
            leftY = 0;
            rightY = 0;
            oneArmMotor.setPower(JoyOneRight);
            upMiddleMotor.setPower(JoyOneLeft);
        }
        else
        {
            oneArmMotor.setPower(rightY); //runing as 1AM, but on right wheels?
            upMiddleMotor.setPower(leftY);


        }



      //***********************************

      if (gamepad2.dpad_up) //This controls the arm of the robot, lifting it up and down.

        {
            threeArmMotor.setPower(.5);
        }
        else if(gamepad2.dpad_down)
        {
            threeArmMotor.setPower(-.5);
        }
        else
        {
            threeArmMotor.setPower(0.0);
        }
        //tried
        //**********************************

        if (gamepad1.left_bumper) //controls the lifting part of the robot in the front.
        {
            twoArmMotor.setPower(.45);
        }
        else if(gamepad1.right_bumper)
        {
            twoArmMotor.setPower(-.45);
        }
        else
        {
            twoArmMotor.setPower(0.0);
        }

        //**********************************

        if(gamepad2.left_trigger >= .80) //Spins the turn table
        {
            turnServo.setPosition(0);
        }
         else if(gamepad2.right_trigger >= .80)
        {
            turnServo.setPosition(1);
        }
        else
        {
          turnServo.setPosition(.5);
        }

        //********************

        if(gamepad2.x)
        { //dumps the blocks
         dumpServo.setPosition(0);
        }
        else
        {
            dumpServo.setPosition(1);
        }
        //Full set of debug info.
        telemetry.addData("Turn Servo's current position is:",turnServo.getPosition());
        telemetry.addData("Dump Servo's current position is:",dumpServo.getPosition());/*
        telemetry.addData("Left Motor's assigned name is:",leftMotor.getDeviceName());
        telemetry.addData("Left Motor's current connection info is :", leftMotor.getConnectionInfo());
        */telemetry.addData("left motor power:",leftMotor.getPower());/*
        telemetry.addData("MM1 name:",upMiddleMotor.getDeviceName());
        telemetry.addData("MM1: connect info",upMiddleMotor.getConnectionInfo());
        */telemetry.addData("MM1 power:",upMiddleMotor.getPower());/*
        telemetry.addData("Right Motor's assigned name is:", rightMotor.getDeviceName());
        telemetry.addData("Right Motor's current connection info is :", rightMotor.getConnectionInfo());
        */telemetry.addData("right motor power:",rightMotor.getPower());/*
        telemetry.addData("1AM assigned name is:",oneArmMotor.getDeviceName());
        telemetry.addData("1AM current connection info is :", oneArmMotor.getConnectionInfo());
       */ telemetry.addData("1AM power:", oneArmMotor.getPower());/*
        telemetry.addData("2AM assigned name is:",twoArmMotor.getDeviceName());
        telemetry.addData("2AM current connection info is :", twoArmMotor.getConnectionInfo());
       */ telemetry.addData("2AM power:", twoArmMotor.getPower());/*
        telemetry.addData("3AM assigned name is:",threeArmMotor.getDeviceName());
        telemetry.addData("3AM current connection info is :", threeArmMotor.getConnectionInfo());
       */ telemetry.addData("3AM:", threeArmMotor.getPower());/*
        telemetry.addData("TS assigned name is:",turnServo.getDeviceName());
        telemetry.addData("TS current connection info is :", turnServo.getConnectionInfo());
        telemetry.addData("DS assigned name is:",dumpServo.getDeviceName());
        telemetry.addData("DS current connection info is :", dumpServo.getConnectionInfo()); */

        // // TODO: 2/13/2016 Add a turbo mode to the drive controlers using the joystick buttons
        //To Do:
        //Work with switching turnservo over to a joystick control rather then with buttons to make it flow better.
        //Also switch over the main arms over to joystick control and trigersr rather then what the are currently set to.
    }
}







