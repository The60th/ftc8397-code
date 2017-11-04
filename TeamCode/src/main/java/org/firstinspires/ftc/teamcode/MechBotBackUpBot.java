package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by FTC Team 8397 on 10/30/2017.
 */

class MechBotBackUpBot extends MechBot {
    private DcMotor arm1;
    private DcMotor arm2;
    private DcMotor lift;
    private Servo leftServo;
    private Servo rightServo;
    private Servo liftServo;
    public void init(HardwareMap ahwMap) {

        super.init(ahwMap);
        arm1 = hardwareMap.dcMotor.get("A1");
        arm2 = hardwareMap.dcMotor.get("A2");
        lift = hardwareMap.dcMotor.get("L1");


        leftServo = hardwareMap.servo.get("lServo");
        rightServo = hardwareMap.servo.get("rServo");
        liftServo = hardwareMap.servo.get("liftServo");

        arm2.setDirection(DcMotorSimple.Direction.FORWARD);

        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    void setInTakePower(float power){
        arm1.setPower(power);
        arm2.setPower(power);
    }
    void setLiftPower(float power){
        lift.setPower(power);
    }
    void dropArm(){
        //Set servo pos -> need testing.
    }


}