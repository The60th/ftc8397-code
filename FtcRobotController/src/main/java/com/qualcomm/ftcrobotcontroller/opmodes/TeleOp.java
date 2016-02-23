package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


//Created by CanAdirondack on 11/25/2015.

//intro comment

public class TeleOp extends OpMode
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
        leftMotor = hardwareMap.dcMotor.get("LM1"); //controller one.
        rightMotor = hardwareMap.dcMotor.get("RM1"); //controller one.

        upMiddleMotor = hardwareMap.dcMotor.get("MM1");//controller two Drive wheels.
        oneArmMotor = hardwareMap.dcMotor.get("AM1");//controller two Drive wheels.

        threeArmMotor = hardwareMap.dcMotor.get("AM3");//Controller three.
        twoArmMotor = hardwareMap.dcMotor.get("AM2");//controller three.

        turnServo = hardwareMap.servo.get("TS1");//Servo controller one.
        dumpServo = hardwareMap.servo.get("DS1"); //Servo controller one.


        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

    }
    @Override
//Driver control!
    public void loop()
    {
        double LeftDrive = gamepad1.left_stick_y/2.5;
        double RightDrive = -gamepad1.right_stick_y/2.5;

        double JoyOneLeft = gamepad1.left_stick_y/1.25;
        double JoyOneRight = -gamepad1.right_stick_y/1.25;

        double rightX = -gamepad2.left_stick_y/2;
        double leftX = -gamepad2.right_stick_y/2;

        double JoyTwoLeft = - gamepad2.left_stick_y/1.25;
        double JoyTwoRight = -gamepad2.right_stick_y/1.25;

//comment this
        leftMotor.setPower(leftX);
        rightMotor.setPower(rightX);

        if(LeftDrive >=.35 && RightDrive >= -.35  && RightDrive != 0 && !gamepad1.right_stick_button && !gamepad1.left_stick_button
                || LeftDrive <= -.35 && RightDrive <= -.35 && RightDrive != 0 && !gamepad1.right_stick_button && !gamepad1.left_stick_button)
        {
            LeftDrive = 0;
            RightDrive = 0;
            oneArmMotor.setPower(JoyOneRight);
            upMiddleMotor.setPower(JoyOneLeft);
        }
        else if(gamepad1.right_stick_button && gamepad1.left_stick_button){
            oneArmMotor.setPower(JoyOneRight);
            upMiddleMotor.setPower(JoyOneLeft);
        }
        else{
            oneArmMotor.setPower(RightDrive);
            upMiddleMotor.setPower(LeftDrive);
        }

        if(gamepad2.left_stick_button && !gamepad2.right_stick_button)
        {
           leftMotor.setPower(JoyTwoLeft);
        }

        else if(gamepad2.right_stick_button && !gamepad2.left_stick_button)
        {
            rightMotor.setPower(JoyTwoRight);
        }
        else
        {
            leftMotor.setPower(leftX);
            rightMotor.setPower(rightX);
        }

//This controls the arm of the robot, lifting it up and down.
        //comment this
      if (gamepad2.right_bumper)

        {
            threeArmMotor.setPower(.5);
        }
        else if(gamepad2.left_bumper)
        {
            threeArmMotor.setPower(-.5);
        }
        else
        {
            threeArmMotor.setPower(0.0);
        }

//controls the lifting part of the robot in the front. comment

        if (gamepad1.left_bumper)
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

//Spins the turn table comment

        if(gamepad2.left_trigger >= .80)
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

//dumps the blocks comment

        if(gamepad2.x)
        {
         dumpServo.setPosition(0);
        }
        else
        {
            dumpServo.setPosition(1);
        }


        //Full set of debug info. comment

        telemetry.addData("Turn Servo's current position is:",turnServo.getPosition());
        telemetry.addData("Dump Servo's current position is:",dumpServo.getPosition());
        telemetry.addData("left motor power:",leftMotor.getPower());
        telemetry.addData("MM1 power:",upMiddleMotor.getPower());
        telemetry.addData("right motor power:",rightMotor.getPower());
        telemetry.addData("1AM power:", oneArmMotor.getPower());
        telemetry.addData("2AM power:", twoArmMotor.getPower());
        telemetry.addData("3AM:", threeArmMotor.getPower());
    }
}







