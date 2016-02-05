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
    }
    @Override
    public void loop()
    {
        double leftY = gamepad1.left_stick_y/2;
        double rightY = gamepad1.right_stick_y/2;
        double leftx = -gamepad2.left_stick_y/2;
        double rightx = -gamepad2.right_stick_y/2;

        leftY = Range.clip(leftY, -1, 1);
        rightY = Range.clip(rightY, -1,1);
        leftx = Range.clip(leftx, -1,1);
        rightx = Range.clip(rightx, -1,1);

        leftMotor.setPower(leftY);
        rightMotor.setPower(rightY);

        oneArmMotor.setPower(leftx);
        twoArmMotor.setPower(rightx);


      //***********************************

       if (gamepad2.dpad_up)
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

        if (gamepad1.left_bumper)
        {
            upMiddleMotor.setPower(-.25);
        }
        else if (gamepad1.right_bumper)
        {
            upMiddleMotor.setPower(.25);
        }
        else
        {
            upMiddleMotor.setPower(0.0);
        }

        //**********************************

        if(gamepad2.left_trigger >= .80)
        {
            turnServo.setPosition(0);
        }
        else
        {
            turnServo.setPosition(.5);
        }

        if(gamepad2.right_trigger >= .80)
        {
            turnServo.setPosition(1);
        }
        else
        {
          turnServo.setPosition(.5);
        }

        //********************

        if(gamepad2.x){
         dumpServo.setPosition(0);
        }
        else{
            dumpServo.setPosition(1);
        }

        telemetry.addData("Turn Servo's current position is:",turnServo.getPosition());


        telemetry.addData("Dump Servo's current position is:",dumpServo.getPosition());

                         // -1
                       //.5   .5
                         //  1


        //To Do:
        //Work with switching turnservo over to a joystick control rather then with buttons to make it flow better.
        //Also switch over the main arms over to joystick control and trigersr rather then what the are currently set to.
    }
}







