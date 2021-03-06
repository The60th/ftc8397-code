package org.firstinspires.ftc.teamcode.mechbot.presupers_bot;

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
    public enum collecterStateValues {OUT,IN,STOP}
    public enum armStateValues {UP,DOWN,STOP}
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

    public void driveCollecter(collecterStateValues collecterStateValuesLocal){
        if(collecterStateValuesLocal == collecterStateValues.IN){
            leftSlideExtenderMotor.setPower(1.0);
            rightSlideExtenderMotor.setPower(1.0);
        }else if(collecterStateValuesLocal == collecterStateValues.OUT){
            leftSlideExtenderMotor.setPower(-1.0);
            rightSlideExtenderMotor.setPower(-1.0);
        }else{
            leftSlideExtenderMotor.setPower(0);
            rightSlideExtenderMotor.setPower(0);
        }

    }

    public void driveArm(armStateValues armStateValuesLocal){
        if(armStateValuesLocal == armStateValues.UP ){
            blockLiftMotor.setPower(1);
        }else if(armStateValuesLocal == armStateValues.DOWN){
            blockLiftMotor.setPower(-.5);
        }else{
            blockLiftMotor.setPower(0);
        }

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
