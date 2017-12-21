package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by FTC Team 8397 on 12/11/2017.
 */
@TeleOp(name = "rev",group = "rev")
public class RevServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo leftLow, leftTop;
        CRServo rightLow, rightTop;

        leftLow = hardwareMap.crservo.get("leftLow");
        leftTop = hardwareMap.crservo.get("leftTop");

        rightLow = hardwareMap.crservo.get("rightLow");
        rightTop = hardwareMap.crservo.get("rightTop");

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.dpad_left){
                leftTop.setPower(1);
            }
            else if(gamepad1.dpad_down){
                leftLow.setPower(1);
            }
            else{
                leftLow.setPower(0);
                leftTop.setPower(0);
            }

            if(gamepad1.y){
                rightTop.setPower(1);
            }
            else if(gamepad1.a){
                rightLow.setPower(1);
            }
            else{
                rightTop.setPower(0);
                rightLow.setPower(0);
            }
        }
    }
}
