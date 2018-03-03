package org.firstinspires.ftc.teamcode.mechbot.presupers_bot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechbot.MechBot;

/**
 * Created by FTC Team 8397 on 10/30/2017.
 */

public class MechBotBackUpBot extends MechBot {
    private DcMotor arm1;
    private DcMotor arm2;
    public DcMotor lift;
    private Servo leftServo;
    private Servo rightServo;
    private CRServo liftServo;
    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);
        arm1 = hardwareMap.dcMotor.get("A1");
        arm2 = hardwareMap.dcMotor.get("A2");
        lift = hardwareMap.dcMotor.get("L1");


        leftServo = hardwareMap.servo.get("lServo");
        rightServo = hardwareMap.servo.get("rServo");
        liftServo = hardwareMap.crservo.get("liftServo");

        arm2.setDirection(DcMotorSimple.Direction.FORWARD);
        arm1.setDirection(DcMotorSimple.Direction.FORWARD);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setInTakePower(float power){
        arm1.setPower(power);
        arm2.setPower(-power);
    }
    public void setLiftPower(double power){
        lift.setPower(power);
    }
    public void dropArm(){
        //Set servo pos -> need testing.
    }
    public void clapIn(){
        liftServo.setPower(1);
    }
    public void clapOff(){
        liftServo.setPower(-1);
    }
    public void clapRest(){
        liftServo.setPower(0);
    }


}