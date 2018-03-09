package org.firstinspires.ftc.teamcode.mechbot.supers_bot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.beta_log.BetaLog;

/**
 * Created by FTC Team 8397 on 2/23/2018.
 */

public class MechBotFlip extends MechBotIntake {
    Servo leftFlipper, rightFlipper, backStop, pincher, pincher2;
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        initHw();
        //leftFlipper.setDirection();
    }
     public void init(HardwareMap ahwMap, float initGyroHeadingDegrees){
         BetaLog.dd("MechBotFlip: ","Init");
         super.init(ahwMap, initGyroHeadingDegrees);
         initHw();
     }
    public void initHw(){
        BetaLog.dd("MechBotFlip: ","Init HW");
        leftFlipper = hardwareMap.servo.get("leftFlipper");
        rightFlipper = hardwareMap.servo.get("rightFlipper");
        backStop = hardwareMap.servo.get("backStop");
        pincher = hardwareMap.servo.get("pincher");
        pincher2 = hardwareMap.servo.get("pincher2");


    }
     float flipUpPos = .55f;
     float flipDownPos = .05f;
    public void flipPlateDownwards(){
        leftFlipper.setPosition(flipDownPos);
        rightFlipper.setPosition(1-flipDownPos);
        backStop.setPosition(0);
    }

    public void flipPlateUpwards(){
        leftFlipper.setPosition(flipUpPos);
        rightFlipper.setPosition(1-flipUpPos);
        backStop.setPosition(.45);

    }
    public void pinchGlyph(){pincher.setPosition(.01);pincher2.setPosition(.99);} // in pos of each servo
    public void setGlyph(){pincher.setPosition(.15);pincher2.setPosition(.85);} // out pos of each servo
    public void startPos(){pincher.setPosition(.45);pincher2.setPosition(.85);} // crypto position


}
