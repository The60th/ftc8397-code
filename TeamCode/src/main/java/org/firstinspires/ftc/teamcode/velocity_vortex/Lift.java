//package org.firstinspires.ftc.teamcode;
//import android.widget.Switch;
//
//import com.qualcomm.robotcore.eventloop.opmode.*;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.ftccommon.DbgLog;
//
//
///**
// * Created by CanAdirondack on 9/30/2016.
// */
//@TeleOp(name=" Lift : Test ", group="Test")
//@Disabled
//
//public class Lift  extends OpMode {
//
//    DcMotor Lift;
//    DcMotor Grabber;
//    DcMotor BallGunR;
//    DcMotor BallGunL;
//    CRServo lifterServo;
//
//    @Override
//    public void init() {
//        hardwareMap.logDevices();
//        Lift = hardwareMap.dcMotor.get("SL");
//        Grabber = hardwareMap.dcMotor.get("SG");
//        BallGunR = hardwareMap.dcMotor.get("BR");
//        BallGunL = hardwareMap.dcMotor.get("BL");
//        lifterServo = hardwareMap.crservo.get("S");
//
//
//    }
//    public void loop(){
//        //Servo type now maters.
//    if (gamepad1.left_trigger >= .5){
//
//       Lift.setPower(1);
//
//    }
//    else if(gamepad1.right_trigger >= .5){
//
//        Grabber.setPower(-1);
//    }
//    else{
//
//    Lift.setPower(0);
//    Grabber.setPower(0);
//
//    }
//        if (gamepad1.a){
//            BallGunR.setPower(1);
//            BallGunL.setPower(-1);
//        }
//        else{
//            BallGunL.setPower(0);
//            BallGunR.setPower(0);
//        }
//        if(gamepad1.b){
//            lifterServo.setPower(-1);
//        }
//        else if(gamepad1.x){
//                lifterServo.setPower(1);
//        }
//        else{
//            lifterServo.setPower(0);
//        }
//
//
//
//    }
//}
//
