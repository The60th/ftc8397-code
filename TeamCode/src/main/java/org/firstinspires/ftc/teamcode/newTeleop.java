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
    boolean awaitingButtonReleaseServo = false;
    boolean awaitingButtonReleaseLift = false;
    boolean awaitingButtonReleasePickup = false;
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

            if(gamepad2.right_trigger > .05){
                robot.setLaunchServo("Up");
            }
            else{
                robot.setLaunchServo("Down");
            }

            if(awaitingButtonReleaseServo && !gamepad2.b && !gamepad2.x){
                robot.setServos("Reset");
                awaitingButtonReleaseServo = false;
            }
            else if(gamepad2.b && !awaitingButtonReleaseServo){
                robot.setServos("Left");
                awaitingButtonReleaseServo = true;
            }
            else if(gamepad2.x && !awaitingButtonReleaseServo){
                robot.setServos("Right");
                awaitingButtonReleaseServo = true;
            }

            //Needs to be rewritten as a true toggle of the boolean with only one toggle per button press. Ie on first press set to true and start the motors
            //Next press set to false and stop the motors.
            //Write toggle inside current if code?
            if(awaitingButtonReleaseLift && (!gamepad1.left_bumper && gamepad1.left_trigger < .05)){
                robot.setLift(0.0);
                awaitingButtonReleaseLift = false;
            }
            else if(gamepad1.left_bumper && !awaitingButtonReleaseLift){
                robot.setLift(1.0);
                awaitingButtonReleaseLift = true;
            }
            else if(gamepad1.left_trigger > .05 && !awaitingButtonReleaseLift){
                robot.setLift(-1.0);
                awaitingButtonReleaseLift = true;
            }

            if(awaitingButtonReleasePickup && (!gamepad1.right_bumper && gamepad1.right_trigger < .05)){
                robot.setSweeper(0.0);
                awaitingButtonReleasePickup = false;
            }
            else if(gamepad1.right_bumper && !awaitingButtonReleaseLift){
                robot.setSweeper(-1.0);
                awaitingButtonReleasePickup = true;
            }
            else if(gamepad1.right_trigger > .05 && !awaitingButtonReleaseLift){
                robot.setSweeper(1.0);
                awaitingButtonReleasePickup = true;
            }

            //Needs Et function with a set time of around 3seconds to lift all the way then lower.
            //Use boolean function to toggle check.
            if(gamepad2.left_trigger > .05) {
                robot.setShooter(1.0);
            }
            else{
                robot.setShooter(0.0);
            }

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
