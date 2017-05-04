package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="Apple", group="Custom")
@Disabled
public class ScienceFairOpen extends LinearOpMode {



    sciFairBot robot = new sciFairBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("","All system go!");
        telemetry.update();
        waitForStart();
        fetchaDaFruits();

    }
    public void openDaFridge(){
        robot.setDriveSpeed(45,45,0);
        sleep(1400);
        telemetry.addData("reached go Foward","");
        telemetry.update();
        robot.setDriveSpeed(0,0,0);
        sleep(450);
        //robot.setDriveSpeed(-45,-45,0);
       // sleep(200);
        //robot.setDriveSpeed(0,0,-20);
        //sleep(700);
    }
    public void fetchaDaFruits(){
        //robot.servo.setPosition(1);
        //sleep(1000);
        //robot.servo.setPosition(.5);

        robot.bottomArm.setPower(.4);
        robot.topArm.setPower(.4);
        sleep(1000);
        robot.bottomArm.setPower(0);
        sleep(1000);
        robot.topArm.setPower(0);

        robot.servo.setPosition(1);
        sleep(200);
        robot.topArm.setPower(-1);
        sleep(200);
        robot.servo.setPosition(.5);
        sleep(1300);
        robot.topArm.setPower(0);
        robot.setDriveSpeed(0,30,0);
        sleep(1800);
        robot.setDriveSpeed(0,0,30);
        sleep(1300);
        robot.bottomArm.setPower(.4);

    }
}
