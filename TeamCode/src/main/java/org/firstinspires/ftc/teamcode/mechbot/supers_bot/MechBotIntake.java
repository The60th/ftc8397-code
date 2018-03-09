package org.firstinspires.ftc.teamcode.mechbot.supers_bot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.mechbot.MechBot;

/**
 * Created by FTC Team 8397 on 2/22/2018.
 */

public class MechBotIntake extends MechBotSensorScranton {

    public DcMotor leftIntake, rightIntake;

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        initHw();
    }
    public void init(HardwareMap ahwMap, float initGyroHeadingDegrees){
        BetaLog.dd("MechBotIntake: ","Init");
        super.init(ahwMap, initGyroHeadingDegrees);
        initHw();

    }

    public void initHw(){
        BetaLog.dd("MechBotIntake: ","Init HW");

        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
