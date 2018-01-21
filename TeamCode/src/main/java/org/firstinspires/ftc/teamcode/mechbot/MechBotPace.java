package org.firstinspires.ftc.teamcode.mechbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by FTC Team 8397 on 1/16/2018.
 */

public class MechBotPace extends MechBotRedHook {
    public Servo jewelRotate;
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        initHw();
    }
    public void init(HardwareMap ahwMap, float initGyroHeadingDegrees){
        super.init(ahwMap, initGyroHeadingDegrees);
        initHw();
    }
    private void initHw(){
        jewelRotate = hardwareMap.servo.get("jewelRotate");
    }

    public void turnJewelArmLeft(){
        jewelRotate.setPosition(1);
    }
    public void turnJewelArmRight(){
        jewelRotate.setPosition(0);
    }
    public void turnJewelArmCenter(){
        jewelRotate.setPosition(.5);
    }

}
