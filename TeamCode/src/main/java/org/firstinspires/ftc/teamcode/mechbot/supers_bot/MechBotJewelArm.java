package org.firstinspires.ftc.teamcode.mechbot.supers_bot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by FTC Team 8397 on 2/28/2018.
 */

public class MechBotJewelArm extends MechBotFlip {

    public Servo pivot, arm;

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        initHw();
        //Arm max up position is left -- 0
        //Arm downwards position should be around .20 -- testing needed

        //Pivot is left for start position
        //Center position -.10ish for middle position.
    }
    public void init(HardwareMap ahwMap, float initGyroHeadingDegrees){
        super.init(ahwMap, initGyroHeadingDegrees);
        initHw();
    }
    public void initHw(){
        pivot = hardwareMap.servo.get("pivot");
        arm = hardwareMap.servo.get("arm");
    }
    public void setArmUp(){
        arm.setPosition(0.0);
    }
    public void setArmDown(){
        arm.setPosition(.60);
    }

    public void setPivotStart(){
        pivot.setPosition(0.0);
    }
    public void setPivotEnd(){
        pivot.setPosition(0.70);
    }

    public void knockPivotLeft(){
        pivot.setPosition(0.50);
    }
    public void knockPivotRight(){
        pivot.setPosition(0.90);
    }


}
