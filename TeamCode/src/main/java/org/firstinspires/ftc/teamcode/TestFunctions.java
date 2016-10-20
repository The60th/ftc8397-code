package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
/**
 * Created by Justi on 10/12/2016.
 */
public class TestFunctions extends OpMode{
    HardwarePushbot         robot   = new HardwarePushbot();
    public void init(

    ){}
    public void loop(){
    //public static void Hello()
        {
        DcMotor Left_Motor;
        DcMotor Right_Motor;
        DcMotor Arm1;
        DcMotor Arm2;
        Servo LeftSweep;
        Servo RightSweep;


        hardwareMap.logDevices();

        Left_Motor = hardwareMap.dcMotor.get("LM");
        Right_Motor = hardwareMap.dcMotor.get("RM");
        Arm1 = hardwareMap.dcMotor.get("A1");
        Arm2 = hardwareMap.dcMotor.get("A2");
        LeftSweep = hardwareMap.servo.get("LS");
        RightSweep = hardwareMap.servo.get("RW");

    }
    }
    public static void ArmorStats(int player, String stat) {

    }
    //double test = GamepadPower(1,1);
    //initiMotors
    //initiMotors'


}

