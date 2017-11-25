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
    public CRServo blockSmackerKickerCRServo, blockSlideCRServo;
    public Servo jewelSmackerServo;

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        leftSlideExtenderMotor = hardwareMap.dcMotor.get("lSE");
        rightSlideExtenderMotor = hardwareMap.dcMotor.get("rSE");
        blockLiftMotor = hardwareMap.dcMotor.get("lift");

        blockSlideCRServo = hardwareMap.crservo.get("BS");
        blockSmackerKickerCRServo = hardwareMap.crservo.get("lA");

        jewelSmackerServo = hardwareMap.servo.get("jSS");

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

    public void smackJewel(){
        jewelSmackerServo.setPosition(.5);
    }
    public void holdJewelSmacker(){
        jewelSmackerServo.setPosition(0);
    }
}
