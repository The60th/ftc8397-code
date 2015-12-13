package com.qualcomm.ftcrobotcontroller.opmodes;


import android.os.Handler;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.Delayed;


public class TeleOpTankMode  extends OpMode {

    ReUseAbleTest MyReUseTest;

    final double Left_Spin = 1.0;
    final double Left_Spin_Stop = 0.5;

    final double Right_Spin = 0.0;
    final double Right_Spin_Stop = 0.5; //new

    final double Up_Spin = 1.0;
    final double Down_Spin = 0.0;

    final double Up_Spin2 = 0.0;
    final double Down_Spin2 = 1.0;

    boolean Servo_On = false;

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor Arm1;
    DcMotor Arm2;
    Servo   Servo1;
    Servo   Servo2;
    Servo Servo3;
    Servo Servo4;



    @Override
    public void init() {
        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        Arm1 =hardwareMap.dcMotor.get("Arm1");
        Arm2 = hardwareMap.dcMotor.get("Arm2");
        Servo1 = hardwareMap.servo.get("Servo1");
        Servo2 = hardwareMap.servo.get("Servo2");
        Servo3 = hardwareMap.servo.get("Servo3");
        Servo4 = hardwareMap.servo.get("Servo4");

        //reverse the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        MyReUseTest = new ReUseAbleTest(
                hardwareMap.dcMotor.get("left"),
                hardwareMap.dcMotor.get("right"),
                hardwareMap.servo.get("Servo_Left"),
                hardwareMap.servo.get("Servo_Right")
        );



    }



    @Override
    public void loop()
    {
        //get the values from the gamepads
        //note: pushing the stick all the way up returns -1,
        // so we need to reverse the values
        float leftY = -gamepad1.left_stick_y * 100;
        float rightY = -gamepad1.right_stick_y * 100; //new
        float arm1 = -gamepad2.left_stick_y * 100;
        float arm2 = -gamepad2.right_stick_y * 100;

        //set the power of the motors with the gamepad values
        leftMotor.setPower(leftY/200);
        rightMotor.setPower(rightY/200);
        Arm2.setPower(arm2/350);
        Arm1.setPower(arm1/350);
        if(gamepad1.x) //new
        {
            Servo1.setPosition(Left_Spin);
            Servo4.setPosition(Right_Spin);  //Servo2.setPosition(Right_Spin);
        }
        else
        {
            Servo1.setPosition(Left_Spin_Stop);
            Servo4.setPosition(Right_Spin_Stop);  // Servo2.setPosition(Right_Spin_Stop);
        }


        Handler handler = new Handler(); //All new stuff have no idea if this works please test Asap.
        handler.postDelayed(new Runnable()
        {
            @Override
            public void run()
            {
                if (gamepad1.y)
               { //new
                    //If turbomode is on, turboOn should be true, since it's on and we pressed the button
                    //we want to shut the function off. First we set turboValue to the non turbo mode
                    //division constant, then we set turboOn to false to indicate that the mode is off to
                    //the next iteration of the program

                    if (Servo_On) {
                        Servo2.setPosition(Up_Spin);
                        Servo3.setPosition(Up_Spin2);
                        Servo_On = false;
                        //Delayed.class
                    }
                    //This is similar to the above, except it does the inverse function
                    else {
                        Servo2.setPosition(Down_Spin);
                        Servo3.setPosition(Down_Spin2);
                        Servo_On = true;
                    }}
                }

            }, 1000);
        }


       /* if(gamepad1.y) { //new
            //If turbomode is on, turboOn should be true, since it's on and we pressed the button
            //we want to shut the function off. First we set turboValue to the non turbo mode
            //division constant, then we set turboOn to false to indicate that the mode is off to
            //the next iteration of the program

            if(Servo_On)
            {
                Servo2.setPosition(Up_Spin);
                Servo3.setPosition(Up_Spin2);
                Servo_On = false;
                //Delayed.class
            }
            //This is similar to the above, except it does the inverse function
            else
            {
                Servo2.setPosition(Down_Spin);
                Servo3.setPosition(Down_Spin2);
                Servo_On = true;
            }
        }*/







    }










