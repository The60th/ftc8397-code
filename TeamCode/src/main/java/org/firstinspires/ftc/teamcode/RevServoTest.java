package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by FTC Team 8397 on 12/11/2017.
 */
//@TeleOp(name = "rev",group = "rev")
public class RevServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo leftLow, leftTop;
        CRServo rightLow, rightTop;
        DcMotor right, left;

        leftLow = hardwareMap.crservo.get("leftLow");
        leftTop = hardwareMap.crservo.get("leftTop");

        rightLow = hardwareMap.crservo.get("rightLow");
        rightTop = hardwareMap.crservo.get("rightTop");

        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");

        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLow.setDirection(DcMotorSimple.Direction.REVERSE);
        leftTop.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.left_trigger > .05){
                leftTop.setPower(-1);
            }
            else if(gamepad1.right_bumper){
                leftLow.setPower(-1);
            }
            else{
                leftLow.setPower(0);
                leftTop.setPower(0);
            }

            if(gamepad1.right_trigger > .05){
                rightTop.setPower(-1);
            }
            else if(gamepad1.left_bumper){
                rightLow.setPower(-1);
            }
            else{
                rightTop.setPower(0);
                rightLow.setPower(0);
            }
            if(gamepad1.right_stick_y > .05){
                right.setPower(-.5);
            }
            else if(gamepad1.right_stick_y < -.05){
                right.setPower(.5);
            }
            else{
                right.setPower(0);
            }
            if(gamepad1.left_stick_y > .05){
                left.setPower(.5);
            }
            else if(gamepad1.left_stick_y < -.05){
                left.setPower(-.5);
            }
            else{
                left.setPower(0);
            }
        }
    }
}
