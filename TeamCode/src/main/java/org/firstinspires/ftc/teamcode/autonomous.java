package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.ftccommon.DbgLog;


import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by CanAdirondack on 10/6/2016.
 */

@Autonomous(name=" Autonomous : Test ", group="Test")

public class autonomous extends LinearOpMode {
    DcMotor one;
    DcMotor two;
    DcMotor three;
    DcMotor four;

    public void runOpMode() throws InterruptedException {
        one = hardwareMap.dcMotor.get("M1");
        two = hardwareMap.dcMotor.get("M2");
        three = hardwareMap.dcMotor.get("M3");
        four = hardwareMap.dcMotor.get("M4");

        two.setDirection(DcMotor.Direction.REVERSE);
        three.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        double angle = 60;
        double power = .5;
        //MoveAtAngle(angle);


    }
    public  void MoveAtAngle(double angle,int time, boolean BreakNotStop)throws InterruptedException{ //angle is in degrees

        double radians = (angle*Math.PI)/180;
        one.setPower(.5*(Math.sin(radians)+Math.cos(radians)));
        two.setPower(.5*(-Math.sin(radians)+Math.cos(radians)));
        three.setPower(.5*(Math.sin(radians)+Math.cos(radians)));
        four.setPower(.5*(-Math.sin(radians)+Math.cos(radians)));
        sleep(time);
    }

    public void MoveStraight(double power, int time, boolean BreakNotStop)throws InterruptedException {
        if(BreakNotStop)
        if(power>1){
            power = 1;
        }
        else if(power<-1){
           power = -1;
        }
        else{
            power = power;
        }
        two.setPower(-power);
        four.setPower(-power);
        one.setPower(power);
        three.setPower(power);
        sleep(time);
    }
}



