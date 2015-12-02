package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//Created by CanAdirondack on 11/25/2015.

public class allterrain extends OpMode{


/*final double Left_Spin = 1.0;
final double Left_Spin_Stop = 0.5;

final double Right_Spin = 0.0;
final double Right_Spin_Stop = 0.5;*/

    float leftY = -gamepad1.left_stick_y * 100;
    float rightY = -gamepad1.right_stick_y * 100;

DcMotor leftMotor;
DcMotor rightMotor;
DcMotor midLeftMotor;
DcMotor midRightMotor;
//Servo Servo1;
//Servo Servo2;
    @Override
    public void init() {
        //get references to the motors from the hardware map
        //leftMotor = hardwareMap.dcMotor.get("left_drive");
        //rightMotor = hardwareMap.dcMotor.get("right_drive");
       // midLeftMotor =hardwareMap.dcMotor.get("Arm1");
        //midrightMotor = hardwareMap.dcMotor.get("Arm2");
        //Servo1 = hardwareMap.servo.get("Servo1");
       // Servo2 = hardwareMap.servo.get("Servo2");
        //reverse the right motor
       // rightMotor.setDirection(DcMotor.Direction.REVERSE);

        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        midLeftMotor =hardwareMap.dcMotor.get("midLeftMotor");
        midRightMotor = hardwareMap.dcMotor.get("midRightMotor");
        //Servo1 = hardwareMap.servo.get("Servo1");
        //jServo2 = hardwareMap.servo.get("Servo2");
        //reverse the right motor
        //rightMotor.setDirection(DcMotor.Direction.REVERSE);
        //midRightMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        //get the values from the gamepads
        //note: pushing the stick all the way up returns -1,
        // so we need to reverse the values
       // float leftY = -gamepad1.left_stick_y * 100;
        //float rightX = -gamepad1.left_stick_x * 100;
        //float arm1 = -gamepad2.left_stick_y * 100;
        //float arm2 = -gamepad2.right_stick_y * 100;

        //set the power of the motors with the gamepad values

        leftMotor.setPower(leftY/200);
        midLeftMotor.setPower(leftY/200);
        rightMotor.setPower(rightY/200);
        midRightMotor.setPower(rightY/200);



        /*if(gamepad1.x) {
            Servo1.setPosition(Left_Spin);
            Servo2.setPosition(Right_Spin);
        }
        else {
            Servo1.setPosition(Left_Spin_Stop);
            Servo2.setPosition(Right_Spin_Stop);
        }*/




    }
}







