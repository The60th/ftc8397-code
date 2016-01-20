package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//Created by CanAdirondack on 11/25/2015.

public class allterrain extends OpMode{
    DcMotor rightRearMotor;
    DcMotor leftRearMotor;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor UpMidleMotor;
    DcMotor threeArmMotor;
    DcMotor oneArmMotor;
    DcMotor twoArmMotor;
    Servo turnServo;
    @Override

    public void init() {
        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        UpMidleMotor =hardwareMap.dcMotor.get("UpMidleMotor");
        threeArmMotor = hardwareMap.dcMotor.get("threeArmMotor");
        leftRearMotor =hardwareMap.dcMotor.get("leftRearMotor");
        rightRearMotor = hardwareMap.dcMotor.get("rightRearMotor");
        oneArmMotor = hardwareMap.dcMotor.get("oneArmMotor");
        twoArmMotor = hardwareMap.dcMotor.get("twoAreMotor");
        turnServo = hardwareMap.servo.get("turnServo");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        //UpRightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override

    public void loop() {
        //set the power of the motors with the gamepad values
        double leftY = -gamepad1.left_stick_y * 100;
        double rightY = -gamepad1.right_stick_y * 100;
        //double UpPower = -.50;
        leftMotor.setPower(rightY/275);
        //midLeftMotor.setPower(leftY/275);
        leftRearMotor.setPower(rightY/275);

        rightMotor.setPower(leftY/275);
       // midRightMotor.setPower(rightY/275);
        rightRearMotor.setPower(leftY/275);

        double leftx = -gamepad2.left_stick_y * 100;
        double rightx = -gamepad2.right_stick_y * 100;
        //double UpPower = -.50;
        oneArmMotor.setPower(rightx/275);

        twoArmMotor.setPower(leftx/275);

        if (gamepad2.right_bumper){
            threeArmMotor.setPower(.5);


        }
        else{
            threeArmMotor.setPower(0.0);
        }


        if (gamepad1.left_bumper){
            UpMidleMotor.setPower(.25);


            if (gamepad1.right_bumper){
                UpMidleMotor.setPower(-.25);


            }
        }

        else
        {
            UpMidleMotor.setPower(0.0);

        }

        if(gamepad2.left_bumper)
        {
            turnServo.setPosition(6);
        }
        else{
          turnServo.setPosition(0);
        }

        //To Do:
        //Work with switching turnservo over to a joystick control rather then with buttons to make it flow better.
        //Also switch over the main arms over to joystick control and trigersr rather then what the are currently set to.

    }}







