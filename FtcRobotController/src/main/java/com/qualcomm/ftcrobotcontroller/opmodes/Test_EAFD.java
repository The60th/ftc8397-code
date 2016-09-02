package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.ftcrobotcontroller.opmodes.ButtonDelay;
import java.util.concurrent.TimeUnit;


/**
 * Created by CanAdirondack on 4/26/2016.
 */
public class Test_EAFD extends OpMode
{
    DcMotor Left_Motor;
    DcMotor Right_Motor;
    DcMotor Arm1;
    DcMotor Arm2;
    DcMotor Extender;
    Servo LeftSweep;
    Servo RightSweep;
    int Ycount = 1;

    public void init(){
        hardwareMap.logDevices();

        Left_Motor = hardwareMap.dcMotor.get("LM");
        Right_Motor = hardwareMap.dcMotor.get("RM");
        Arm1 = hardwareMap.dcMotor.get("A1");
        Arm2 = hardwareMap.dcMotor.get("A2");
        Extender = hardwareMap.dcMotor.get("E");
        LeftSweep = hardwareMap.servo.get("LS");
        RightSweep = hardwareMap.servo.get("RW");
        Right_Motor.setDirection(DcMotor.Direction.REVERSE);
        Arm2.setDirection(DcMotor.Direction.REVERSE);


    }

    public void loop()
    {
        double LeftDrive;
        double RightDrive;
        double ArmPower;
        double ExtendPower;
        ButtonDelay delay = new ButtonDelay();
        //int Ystate;


        //Ystate=1;

        //Catapult mode!
       // if(gamepad1.a && gamepad1.b && gamepad2.a && gamepad2.b){
        //    ArmPower = gamepad2.left_stick_y;
        //    LeftDrive = gamepad1.left_stick_y;
        //    RightDrive = gamepad1.right_stick_y;
       // }
        //Default mode with edited power values
       // else{

            LeftDrive = gamepad1.left_stick_y;
            RightDrive = gamepad1.right_stick_y;
            ArmPower = gamepad2.left_stick_y/4;
            ExtendPower = gamepad2.right_stick_y/2;
        // }
        //Left_Motor.setPower(LeftDrive);
        //Right_Motor.setPower(RightDrive);


        //Extender
        if(gamepad2.right_stick_y > (.20)) {
            Extender.setPower(ExtendPower);
        }
        else if(gamepad2.right_stick_y< - (.20)){
            Extender.setPower(ExtendPower);
        }
        else{
            Extender.setPower(0);
        }

        //For arm controls

        if(gamepad2.left_stick_y > (.20)) {
            Arm1.setPower(ArmPower);
            Arm2.setPower(ArmPower);
            telemetry.addData("Running arm forwards", "");
        }
        else if(gamepad2.left_stick_y< - (.20)){
            Arm1.setPower(ArmPower);
            Arm2.setPower(ArmPower);
            telemetry.addData("Running arm backwards","");
        }
        else{
            Arm1.setPower(0);
            Arm2.setPower(0);
        }
        // RightSweep moving code
        if(gamepad1.right_bumper){
            RightSweep.setPosition(1);
        }
        else if(gamepad1.right_trigger>.75){
            RightSweep.setPosition(0);
        }
        else{
            RightSweep.setPosition(.5);
        }
        if (gamepad1.left_bumper) {
            LeftSweep.setPosition(0);
        }
        else if(gamepad1.left_trigger>.75){
            LeftSweep.setPosition(1);
        }
        else{
            LeftSweep.setPosition(.5);
        }
        //Fine Movement
        //Drive controls
        //ButtonDelay(HardwareMap hardwareMap) throws InterruptedException {

       // }

        if(gamepad1.y)
        {
            Ycount +=1;


            try {
                //Thread.sleep(500);
                TimeUnit.MILLISECONDS.sleep(500);


            }
            catch (InterruptedException e)
            {

            }

        }

        //telemetry.addData("Ycount Value:",Ycount);

        if(Ycount%2 == 0)
        {
            Left_Motor.setPower(LeftDrive/4);
            Right_Motor.setPower(RightDrive/4);
            telemetry.addData("Driving Mode:", "Slow");
        }else {
            Left_Motor.setPower(LeftDrive);
            Right_Motor.setPower(RightDrive);
            telemetry.addData("Driving Mode:", "Normal");
        }

//ButtonDelay.class
    }
}
