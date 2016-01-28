package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.util.UUID;
//Created by CanAdirondack on 11/25/2015.

public class allterrain extends OpMode{
    DcMotor rightRearMotor;
    DcMotor leftRearMotor;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor upMiddleMotor;
    //   DcMotor threeArmMotor;
   // DcMotor oneArmMotor;
   // DcMotor twoArmMotor;
    Servo turnServo;
    Servo dumpServo;
    @Override
    public void init() {
        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("LM1"); //L_M1 was leftMotor
        rightMotor = hardwareMap.dcMotor.get("RM1"); //R_M1 was rightMotor
       upMiddleMotor = hardwareMap.dcMotor.get("MM1"); //M_M1 was middlemotor
        //threeArmMotor = hardwareMap.dcMotor.get("A_M3"); //A_M3 arm motor 3
        leftRearMotor = hardwareMap.dcMotor.get("LM2"); //L_M2 was leftRearMotor
        rightRearMotor = hardwareMap.dcMotor.get("RM2"); //R_M2 was rightRearMotor
       // oneArmMotor = hardwareMap.dcMotor.get("AM1");
       // twoArmMotor = hardwareMap.dcMotor.get("AM2");
        turnServo = hardwareMap.servo.get("TS1");
        dumpServo = hardwareMap.servo.get("DS");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void loop() {
        double leftY = gamepad1.left_stick_y/2;
        double rightY = gamepad1.right_stick_y/2;
        leftY = Range.clip(leftY, -1, 1);
        rightY = Range.clip(rightY, -1,1);

            leftMotor.setPower(leftY);
            leftRearMotor.setPower(leftY);

            rightMotor.setPower(rightY);
            rightRearMotor.setPower(rightY);

        //double leftx = -gamepad2.left_stick_y;
       // double rightx = -gamepad2.right_stick_y;
       // leftx = Range.clip(leftx, -1,1);
        //rightx = Range.clip(rightx, -1,1);

       // oneArmMotor.setPower(leftx);
        //twoArmMotor.setPower(rightx);

       // double leftx = -gamepad2.left_stick_y;
       // double rightx = -gamepad2.right_stick_y;
       // double UpPower = -.50;
        //oneArmMotor.setPower(rightx/275);
        //twoArmMotor.setPower(leftx/275);

       // if (gamepad2.right_bumper)
       // {
       //   //  threeArmMotor.setPower(.5);
       // }
       // else
       // {
       //    // threeArmMotor.setPower(0.0);
       // }


        if (gamepad1.left_bumper)
        {
            upMiddleMotor.setPower(-.25);
            if (gamepad1.right_bumper)
            {
                upMiddleMotor.setPower(.25);
            }
        }
        else
        {
            upMiddleMotor.setPower(0.0);
        }



        if(gamepad2.left_bumper)
        {
            turnServo.setPosition(0);
            if(gamepad2.right_bumper)
            {
                turnServo.setPosition(1);
            }


        }
        else
        {
          turnServo.setPosition(.5);
        }

        if(gamepad2.x){
         dumpServo.setPosition(0);
        }
        else{
            dumpServo.setPosition(1);
        }




        //To Do:
        //Work with switching turnservo over to a joystick control rather then with buttons to make it flow better.
        //Also switch over the main arms over to joystick control and trigersr rather then what the are currently set to.
    }}







