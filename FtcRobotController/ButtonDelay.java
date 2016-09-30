//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import android.bluetooth.BluetoothClass;
//import android.graphics.Color;
//
////import com.qualcomm.hardware.ModernRoboticsI2cGyro;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.GyroSensor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.robocol.Telemetry;
////1(518)314-6727 gilly's #y
//import java.util.concurrent.TimeUnit;
//
///**
// * Created by CanAdirondack on 8/12/2016.
// */ //test
//public class ButtonDelay {
//    int Ycount = 1;
//    public float LeftDrive;
//    public float RightDrive;
//    DcMotor Left_Motor, Right_Motor;
//
//    public void ButtonDelay(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1) throws  InterruptedException{
//        LeftDrive = gamepad1.left_stick_y;
//        RightDrive = gamepad1.right_stick_y;
//        hardwareMap.logDevices();
//        Left_Motor = hardwareMap.dcMotor.get("LM");
//        Right_Motor = hardwareMap.dcMotor.get("RM");
//        if(gamepad1.y)
//        {
//            Ycount +=1;
//            try {
//                //Thread.sleep(500);
//                TimeUnit.MILLISECONDS.sleep(500);
//
//
//            }
//            catch (InterruptedException e)
//            {
//                //telemetry.addData("","");
//            }
//            if(Ycount%2 == 0)
//            {
//                Left_Motor.setPower(LeftDrive/4);
//                Right_Motor.setPower(RightDrive/4);
//                telemetry.addData("Driving Mode:", "Slow");
//            }else {
//                Left_Motor.setPower(LeftDrive);
//                Right_Motor.setPower(RightDrive);
//                telemetry.addData("Driving Mode:", "Normal");
//            }
//
//        }
//        return;
//
//    }
//
//
//}
