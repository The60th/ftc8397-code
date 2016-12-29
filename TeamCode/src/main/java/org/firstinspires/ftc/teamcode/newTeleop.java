package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import java.nio.channels.SelectableChannel;

/**
 * Created by CanAdirondack on 12/16/2016.
 */
@TeleOp(name=" TeleOp: ", group="TeleOp_Programs")
public class newTeleop extends LinearOpMode {
    OmniBot        robot   = new OmniBot();
    float px;
    float py;
    float pTheta;
    float newX =0;
    float newY=0;
    float newTheta=0;
    public final float C_PHI = .1f;
    public final float C_X = .1f;
    public String mode = robot.phoneFront;
    boolean awaitingButtonReleaseServo = false;
    boolean awaitingButtonReleaseLift = false;
    boolean awaitingButtonReleasePickup = false;
    VuforiaNav vuforia = new VuforiaNav();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        vuforia.activate();
        waitForStart();
        robot.updateOdometry();
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive()){
            px = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x: 0;
            py = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y: 0;

            pTheta = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x: 0;
            robot.setDrivePower(px,py,pTheta,mode);

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

            if((awaitingButtonReleasePickup && gamepad1.right_bumper) || (gamepad1.right_trigger < .05 && awaitingButtonReleaseLift)){
                if(awaitingButtonReleaseLift) awaitingButtonReleaseLift = false;
                robot.setSweeper(0.0);
                awaitingButtonReleasePickup = false;
            }
            else if(gamepad1.right_bumper && !awaitingButtonReleaseLift){
                robot.setSweeper(-1.0);
                awaitingButtonReleasePickup = true;
            }
            else if(gamepad1.right_trigger > .05 && !awaitingButtonReleaseLift){
                robot.setSweeper(1.0);
                awaitingButtonReleaseLift = true;
            }


            //Needs Et function with a set time of around 3seconds to lift all the way then lower.
            //Use boolean function to toggle check.
            if(gamepad2.left_trigger > .05) {
                robot.setShooter(1.0);
            }
            else{
                robot.setShooter(0.0);
            }


            if(gamepad1.x){
                teleopBeacon();
                while(opModeIsActive() && gamepad1.x) idle();
            }


            if(gamepad2.dpad_down){
                robot.setBigBallLift(1);
            }
            else if(gamepad2.dpad_up){
                robot.setBigBallLift(-1);
            }
            else{
                robot.setBigBallLift(0);
            }


            if(gamepad1.dpad_up){
                mode = robot.phoneFront;
                //Works
            }
            else if (gamepad1.dpad_right)
            {
                mode = robot.sweepFront;
                //Seems to disable all forward drive??
            }
            else if(gamepad1.dpad_left){
                mode = robot.liftFront;
                //Works and does sweeper front currently??
            }
            else if(gamepad1.dpad_down){
                mode = robot.shootFront;
                //Reveres left and right drive but nothing else.
            }


            if(gamepad2.start){
                robot.setServoUp();
            }


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
    public void teleopBeacon() throws InterruptedException{
        float v = 20;
        float[] zxPhi = null;
        OpenGLMatrix robotPosition;
        boolean targetFound = false;
        int targetNumber = 21;
        for(int i = 0;i < 4;i++) {
            if(vuforia.isTargetVisible(i)){
                targetFound = true;
                targetNumber = i;
                break;
            }
        }
        if(!targetFound)return;
        while (opModeIsActive() && gamepad1.x) {
            robotPosition = vuforia.getRobotLocationRelativeToTarget(targetNumber);
            if (robotPosition != null) {
                zxPhi = VuforiaNav.GetZXPH(robotPosition);
            }
            if (zxPhi != null) {
                if (zxPhi[0] <= robot.vuforiaZDistance) {
                    robot.setDriveSpeed(0, 0, 0);
                    return;
                } else {
                    float[] newSpeeds = getCorrectedSpeeds(zxPhi[1], zxPhi[2], v, 0);
                    robot.setDriveSpeed(newSpeeds[0], newSpeeds[1], newSpeeds[2]);
                }
            }
            idle();
        }
        robot.setDriveSpeed(0,0,0);
    }

    public float[] getCorrectedSpeeds(float x,float phi,float v, float x0) {
        float phiPrime = VuforiaNav.remapAngle(phi-(float)Math.PI);
        float va = -phiPrime*Math.abs(v)*C_PHI;
        float vx = -C_X*Math.abs(v)*(x-x0)*(float)Math.cos(phiPrime);
        float vy = v+Math.abs(v)*C_X*(x-x0)*(float)Math.sin(phiPrime);
        return new float[]{vx,vy,va};
    }
}
