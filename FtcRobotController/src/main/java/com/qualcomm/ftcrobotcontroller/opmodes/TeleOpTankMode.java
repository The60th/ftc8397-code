package com.qualcomm.ftcrobotcontroller.opmodes;

import android.os.Handler;
import android.os.Looper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
/*
All code currently using the MotorsServoClass is currently
work in progress do not uncomment till finished. When finished
it should let us use the Motors and Servos using a single class
line instead of what is currently in use.
 */

public class TeleOpTankMode  extends OpMode
{
    //MotorServoClass My_Servos_Motors;

    /*private final double Left_Spin = 1.0;
    private final double Left_Spin_Stop = 0.5;

    private final double Right_Spin = 0.0;
    private final double Right_Spin_Stop = 0.5; //new */

    private final double Up_Spin = 1.0;
    private final double Down_Spin = 0.0;

    private final double Up_Spin2 = 0.0;
    private final double Down_Spin2 = 1.0;

    private boolean Servo_On = false;

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor Arm1;
    private DcMotor Arm2;
    private Servo Servo1;
    private Servo Servo2;
    private Servo Servo3;
    private Servo Servo4;

    boolean ybuttonpressed;


    @Override
    public void init()
    {
        //get references to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        Arm1 = hardwareMap.dcMotor.get("Arm1");
        Arm2 = hardwareMap.dcMotor.get("Arm2");
        Servo1 = hardwareMap.servo.get("Servo1");
        Servo2 = hardwareMap.servo.get("Servo2");
        Servo3 = hardwareMap.servo.get("Servo3");
        Servo4 = hardwareMap.servo.get("Servo4");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }


    @Override
    public void loop()
    {
       //My_Servos_Motors = MotorServoClass( hardwareMap.dcMotor.get("left"),
       //                                    hardwareMap.dcMotor.get("right"),
       //                                    hardwareMap.servo.get("Servo_Left"),
       //                                    hardwareMap.servo.get("Servo_Right"));



        //Moved here to be a local double rather then whole class wide just makes stuff work better I believe.
         final double Right_Spin = 1.0;
         final double Right_Spin_Stop = 0.5;
         final double Left_Spin = 0.0;
         final double Left_Spin_Stop = 0.5;

        float leftY = -gamepad1.left_stick_y * 100;
        float rightY = -gamepad1.right_stick_y * 100; //
        float arm1 = -gamepad2.left_stick_y * 100;
        float arm2 = -gamepad2.right_stick_y * 100;

        leftMotor.setPower(leftY / 200);
        rightMotor.setPower(rightY / 200);
        Arm2.setPower(arm2 / 350);
        Arm1.setPower(arm1 / 350);


        if (gamepad1.x)
        {
            Servo1.setPosition(Left_Spin);
            Servo4.setPosition(Right_Spin);
        } else
        {
            Servo1.setPosition(Left_Spin_Stop);
            Servo4.setPosition(Right_Spin_Stop);
        }

        //Looper.prepare();
       // Handler handler = new Handler(); //All new stuff have no idea if this works please test Asap.
        //handler.postDelayed(new Runnable()
       // {
            //@Override
           // public void run()
            {
                if (gamepad1.y) {
                    ybuttonpressed = true;
                }
                else {
                    if (ybuttonpressed) {

                        ybuttonpressed = false;

                        if (Servo_On) {
                            Servo2.setPosition(Up_Spin);
                            Servo3.setPosition(Up_Spin2);
                            Servo_On = false;
                            //Delayed.class
                        }

                        else {
                            Servo2.setPosition(Down_Spin);
                            Servo3.setPosition(Down_Spin2);
                            Servo_On = true;
                        }

                    }
                }
            }

        //}, 1000);

    }
}











