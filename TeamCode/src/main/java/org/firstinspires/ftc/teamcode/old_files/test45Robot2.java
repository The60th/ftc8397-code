//package org.firstinspires.ftc.teamcode;
//
//import android.app.Activity;
//import android.graphics.Color;
//import android.view.View;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.I2cAddr;
//import com.qualcomm.robotcore.hardware.Servo;
//
///**
// * Created by CanAdirondack on 11/25/2016.
// */
//@TeleOp(name=" Test45Robot3 : Test ", group="Test")
//@Disabled
//public class test45Robot2 extends  LinearOpMode{
//        boolean turnMode = false;
//        allSensors sensors = new allSensors();
//
//        DcMotor one;
//        DcMotor two;
//        DcMotor three;
//        DcMotor four;
//        DcMotor Lift;
//        DcMotor Grabber;
//        DcMotor LeftLaunch;
//        DcMotor RightLaunch;
//        CRServo lifterServo;
//
//        @Override
//        public void runOpMode() throws InterruptedException {
//            float hsvValues[] = {0F,0F,0F};
//            final float values[] = hsvValues;
//
//            //one = hardwareMap.dcMotor.get("M1");
//            //two = hardwareMap.dcMotor.get("M2");
//            //three = hardwareMap.dcMotor.get("M3");
//            //four = hardwareMap.dcMotor.get("M4");
//            Lift = hardwareMap.dcMotor.get("SL");
//            Grabber = hardwareMap.dcMotor.get("SG");
//            lifterServo = hardwareMap.crservo.get("S");
//
//            LeftLaunch = hardwareMap.dcMotor.get("LL");
//            RightLaunch = hardwareMap.dcMotor.get("RL");
//
//            LeftLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            RightLaunch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            RightLaunch.setDirection(DcMotor.Direction.REVERSE);
//           // four.setDirection(DcMotor.Direction.REVERSE);
//            //three.setDirection(DcMotor.Direction.REVERSE);
//            idle();
//            waitForStart();
//            while (opModeIsActive()){
//                double F_B_Drive = gamepad1.right_stick_y;
//                double L_R_Drive = gamepad1.right_stick_x;
//                double TurnDrive = gamepad1.left_stick_x;
//                //boolean robotIsStopped = Math.abs(F_B_Drive) < .05 && Math.abs(L_R_Drive) < .05;
//                telemetry.update();
//
//               // boolean changeMode = (turnMode && Math.abs(TurnDrive) < .05 && !robotIsStopped) || (!turnMode && Math.abs(TurnDrive) > .05 && robotIsStopped);
//                //if (changeMode) {
//                  //  turnMode = !turnMode;
//               // }
//                //if (turnMode) {
//                 //   moveAtTurnMode();
//                //} else {
//                 ///   moveAtJoystickAngle();
//               // }
//
//                if (gamepad1.right_trigger >= .5) {
//
//                    Lift.setPower(1);
//
//                } else if (gamepad1.left_trigger >= .5) {
//
//                    Grabber.setPower(-1);
//                } else {
//                    Lift.setPower(0);
//                    Grabber.setPower(0);
//                }
//                if (gamepad2.left_trigger >= .5) {
//                    LeftLaunch.setPower(-1);
//                    RightLaunch.setPower(-1);
//                } else {
//                    LeftLaunch.setPower(0);
//                    RightLaunch.setPower(0);
//                }
//                if(gamepad2.right_trigger >= .5){
//                    lifterServo.setPower(-1);
//                }
//                else{
//                    lifterServo.setPower(1);
//                }
//                idle();
//            }
//
//        }
//
//        public double moveAtJoystickAngle(){
//
//            double x = gamepad1.right_stick_x;
//            double y = -gamepad1.right_stick_y;
//            if( Math.max(Math.abs(x),Math.abs(y)) <.05){
//               // one.setPower(0);
//                //two.setPower(0);
//                //three.setPower(0);
//                //four.setPower(0);
//                return 0;
//            }
//            double w1 = -x + y;
//            double w2 = x + y;
//            double max = Math.max(Math.abs(w1),Math.abs(w2));
//
//            if(max >1){
//
//                w1=w1/max;
//                w2=w2/max;
//                //one.setPower(w1);
//                //two.setPower(w2);
//                //three.setPower(w1);
//                //four.setPower(w2);
//                telemetry.addData("Max greater then 1: W1 ",w1);
//                telemetry.addData("Max greater then 1: W2 ",w2);
//                this.telemetry.update();
//                return max;
//
//            }
//            //one.setPower(w1);
//            //two.setPower(w2);
//            //three.setPower(w1);
//            //four.setPower(w2);
//
//            telemetry.addData("Max less then 1: W1 ",w1);
//            telemetry.addData("Max less then 1: W2 ",w2);
//            this.telemetry.update();
//            return 1;
//
//        }
//        public void moveAtTurnMode(){
//            double turnDrive = gamepad1.left_stick_x;
//            if(Math.abs(turnDrive) <.05){
//                //one.setPower(0);
//                //two.setPower(0);
//                //three.setPower(0);
//               // four.setPower(0);
//                return;
//            }
//            if (turnDrive >= .05 ){ //Is right on the joystick and spins the robot right
//                //one.setPower(.5);
//                //two.setPower(.5);
//                //three.setPower(-.5);
//                //four.setPower(-.5);
//                telemetry.addData("Running in turn mode:","");
//                this.telemetry.update();
//            }
//            else if (turnDrive <= -.05){//Is left on the joystick and spins the robot left
//                //one.setPower(-.5);
//                //two.setPower(-.5);
//                //three.setPower(.5);
//                //four.setPower(.5);
//                telemetry.addData("Running in turn mode:","");
//                this.telemetry.update();
//            }
//
//        }
//    }
//
//
//
