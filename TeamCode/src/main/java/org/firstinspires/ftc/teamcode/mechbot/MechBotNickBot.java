package org.firstinspires.ftc.teamcode.mechbot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by FTC Team 8397 on 11/17/2017.
 */

public class MechBotNickBot extends MechBotSensor {
    public DcMotor leftSlideExtender,rightSlideExtender,blockLift;
    public CRServo liftAdjuster;

    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        leftSlideExtender = hardwareMap.dcMotor.get("lSE");
        rightSlideExtender = hardwareMap.dcMotor.get("rSE");
        blockLift = hardwareMap.dcMotor.get("lift");

        liftAdjuster = hardwareMap.crservo.get("lA");

        rightSlideExtender.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlideExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blockLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**
         * Save passed HardwareMap to local class variable HardwareMap.
         */
    }

    public void driveArm(float power){
        leftSlideExtender.setPower(power);
        rightSlideExtender.setPower(power);
    }

}
