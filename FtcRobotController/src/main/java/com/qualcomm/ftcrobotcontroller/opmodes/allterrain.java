package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.UUID;
//Created by CanAdirondack on 11/25/2015.

public class allterrain extends OpMode{
    DcMotor rightRearMotor;
    DcMotor leftRearMotor;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor upMiddleMotor;
    DcMotor threeArmMotor;
    //DcMotor oneArmMotor;
    //DcMotor twoArmMotor;
    Servo turnServo;
    @Override
    public void init() {
        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("LM1"); //L_M1 was leftMotor
        rightMotor = hardwareMap.dcMotor.get("RM1"); //R_M1 was rightMotor
        upMiddleMotor = hardwareMap.dcMotor.get("MM1"); //M_M1 was middlemotor
        //threeArmMotor = hardwareMap.dcMotor.get("A_M3"); //A_M3 arm motor 3
        leftRearMotor = hardwareMap.dcMotor.get("LM2"); //L_M2 was leftRearMotor
        rightRearMotor = hardwareMap.dcMotor.get("RM2"); //R_M2 was rightRearMotor
        //oneArmMotor = hardwareMap.dcMotor.get("oneArmMotor");
        //twoArmMotor = hardwareMap.dcMotor.get("twoArmMotor");
        turnServo = hardwareMap.servo.get("turnServo");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void loop() {
        //Updates values geting rid of the old *100 and /275 and working with just /75
        double leftY = -gamepad1.left_stick_y;
        double rightY = -gamepad1.right_stick_y;
        leftMotor.setPower(rightY/75);
        //midLeftMotor.setPower(leftY/75);
        leftRearMotor.setPower(rightY/75);
        rightMotor.setPower(leftY/75);
       // midRightMotor.setPower(rightY/75);
        rightRearMotor.setPower(leftY/75);
        double leftx = -gamepad2.left_stick_y;
        double rightx = -gamepad2.right_stick_y;
        double UpPower = -.50;
        //oneArmMotor.setPower(rightx/275);
        //twoArmMotor.setPower(leftx/275);

        if (gamepad2.right_bumper)
        {
            threeArmMotor.setPower(.5);
        }
        else
        {
            threeArmMotor.setPower(0.0);
        }
        if (gamepad1.left_bumper)
        {
            upMiddleMotor.setPower(.25);
            if (gamepad1.right_bumper)
            {
                upMiddleMotor.setPower(-.25);
            }
        }
        else
        {
            upMiddleMotor.setPower(0.0);
        }

        if(gamepad2.left_bumper)
        {
            turnServo.setPosition(0);
        }
        else{
          turnServo.setPosition(.5);
        }
        //To Do:
        //Work with switching turnservo over to a joystick control rather then with buttons to make it flow better.
        //Also switch over the main arms over to joystick control and trigersr rather then what the are currently set to.
    }}







