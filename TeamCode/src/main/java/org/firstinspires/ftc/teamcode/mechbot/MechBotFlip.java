package org.firstinspires.ftc.teamcode.mechbot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by FTC Team 8397 on 2/23/2018.
 */

public class MechBotFlip extends MechBotIntake {
    Servo leftFlipper, rightFlipper, backStop;
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        leftFlipper = hardwareMap.servo.get("leftFlipper");
        rightFlipper = hardwareMap.servo.get("rightFlipper");
        backStop = hardwareMap.servo.get("backStop");


        //leftFlipper.setDirection();
    }
    public void flipPlateUpwards(){
        leftFlipper.setPosition(.40);
        rightFlipper.setPosition(0);
        backStop.setPosition(.45);
    }

    public void flipPlateDownwards(){
        leftFlipper.setPosition(0);
        rightFlipper.setPosition(.50);
        backStop.setPosition(0);

    }
}
