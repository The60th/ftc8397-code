package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class TeleOpTankMode  extends OpMode {

    final double Left_Spin = 1.0;
    final double Left_Spin_Stop = 0.5;

    final double Right_Spin = 0.0;
    final double Right_Spin_Stop = 0.5;

    boolean turboOn = false; //a boolean that is used in the loop function to enable/disable turbo mode
    float turboValue = 150; //initing a variable that can be flip/flopped between the turbomode value and standard speed

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor Arm1;
    DcMotor Arm2;
    Servo   Servo1;
    Servo   Servo2;
    @Override
    public void init() {
        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        Arm1 =hardwareMap.dcMotor.get("Arm1");
        Arm2 = hardwareMap.dcMotor.get("Arm2");
        Servo1 = hardwareMap.servo.get("Servo1");
        Servo2 = hardwareMap.servo.get("Servo2");
        //reverse the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        //get the values from the gamepads
        //note: pushing the stick all the way up returns -1,
        // so we need to reverse the values
        float leftY = -gamepad1.left_stick_y * 100;
        float rightY = -gamepad1.right_stick_y * 100;
        float arm1 = -gamepad2.left_stick_y * 100;
        float arm2 = -gamepad2.right_stick_y * 100;
        //We need two different power settings for moving the robot around, they are defined here
        //They are used as a dividing constant in the setpower argument
        float nonTurboPower = 200;
        float turboPower = 150;


        //set the power of the motors with the gamepad values
        leftMotor.setPower(leftY/turboValue);
        rightMotor.setPower(rightY/turboValue);
        Arm2.setPower(arm2/350);
        Arm1.setPower(arm1/350);

        if(gamepad1.a) {
            //If turbomode is on, turboOn should be true, since it's on and we pressed the button
            //we want to shut the function off. First we set turboValue to the non turbo mode
            //division constant, then we set turboOn to false to indicate that the mode is off to
            //the next iteration of the program
            if(turboOn) {
                turboValue = nonTurboPower;
                turboOn = false;
            }
            //This is similar to the above, except it does the inverse function
            else {
                turboValue = turboPower;
                turboOn = true;
            }
        }

        if(gamepad1.x) {
            Servo1.setPosition(Left_Spin);
            Servo2.setPosition(Right_Spin);
        }
        else {
            Servo1.setPosition(Left_Spin_Stop);
            Servo2.setPosition(Right_Spin_Stop);
        }




    }
}