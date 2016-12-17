package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by CanAdirondack on 12/16/2016.
 */
@TeleOp(name=" TeleOp Testing: ", group="TeleOp_Testing.")
public class newTeleop extends LinearOpMode {
    OmniBot        robot   = new OmniBot();
    float px ;
    float py;
    float pTheta;
    float newX =0;
    float newY=0;
    float newTheta=0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        robot.updateOdometry();
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive()){
            px = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x: 0;
            py = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y: 0;
            pTheta = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x: 0;
            robot.setDrivePower(px,py,pTheta);
            float pos[] = robot.updateOdometry(newX,newY,newTheta);
            newX = pos[0];
            newY = pos[1];
            newTheta = pos[2];
            if(et.milliseconds()>500){
                et.reset();
                telemetry.addData("Pos:","X = %.0f Y = %.0f Theta = %.0f",newX,newY,newTheta*180.0/Math.PI);
                telemetry.update();
            }
            idle();
    }
    }
}
