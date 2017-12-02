package org.firstinspires.ftc.teamcode.mechbot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by FTC Team 8397 on 11/17/2017.
 */

public class MechBotNickBot extends MechBotSensor {
    public DcMotor leftSlideExtenderMotor, rightSlideExtenderMotor, blockLiftMotor;
    public Servo kickerServo,slideServo;
    public CRServo jewelServo;
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        leftSlideExtenderMotor = hardwareMap.dcMotor.get("lSE");
        rightSlideExtenderMotor = hardwareMap.dcMotor.get("rSE");
        blockLiftMotor = hardwareMap.dcMotor.get("lift");

        slideServo = hardwareMap.servo.get("BS");
        kickerServo = hardwareMap.servo.get("lA");

        jewelServo = hardwareMap.crservo.get("jSS");

        rightSlideExtenderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlideExtenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideExtenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blockLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blockLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void driveArm(float power){
        leftSlideExtenderMotor.setPower(power);
        rightSlideExtenderMotor.setPower(power);
    }

    public void lowerJewelArm(){
        jewelServo.setPower(.5);
    }
    public void raiseJewelArm(){
        jewelServo.setPower(-.5);

    }
    public void breakJewelArm(){
        jewelServo.setPower(0);
    }
    //public void smackJewel(){
     //   jewelServo.setPosition(.5);
    //}
    //public void holdJewelSmacker(){
      //  jewelServo.setPosition(0);
    //}
}
