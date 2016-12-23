/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;


@Autonomous(name="AutonomousOpMode1", group="Pushbot")
//@Disabled
public class AutonomousOpMode1 extends LinearOpMode {
    public final float C_PHI = .1f;
    public final float C_X = .1f;
    float[] zxPhi;
    VuforiaNav vuforianav = null;

    /* Declare OpMode members. */
    OmniBot        robot   = new OmniBot();
    allSensors sensors = new allSensors();
    @Override
    public void runOpMode() throws InterruptedException {
        float v = 20;
        robot.init(hardwareMap);
        vuforianav = new VuforiaNav();
        robot.sensorGyro.calibrate();
        while(opModeIsActive() && robot.sensorGyro.isCalibrating()){
            telemetry.addData("Gyro Cali","");
            telemetry.update();
            idle();
        }
        telemetry.addData("About to activate Vuforia","");
        ElapsedTime vufTime = new ElapsedTime();
        telemetry.update();
        vuforianav.activate();
        telemetry.addData("Vuforia activated. ","Took %f milliseconds. ",vufTime.milliseconds());
        telemetry.update();
        waitForStart();
        robot.setDriveSpeed(0,-50,0);
        sleep(200);
        robot.setDriveSpeed(0,0,0);
      /* *//* while(opModeIsActive()) {
            float gyroHeading = robot.sensorGyro.getHeading();
            while (opModeIsActive()&&(gyroHeading < 175 || gyroHeading > 185)) {
                if (gyroHeading < 175) {
                    //robot.setDriveSpeed(0, 0, 10 + (175 - gyroHeading));
                } else {
                   // robot.setDriveSpeed(0, 0, -10 - (gyroHeading - 185));

                }
                telemetry.addData("Gyro value:", gyroHeading);
                DbgLog.msg("Robot Debug: Gyro Value: %f", gyroHeading);
                gyroHeading = robot.sensorGyro.getHeading();
                telemetry.update();
                idle();
            }
            idle();
        *//*}*/
        robot.setDriveSpeed(0,43.3,-Math.PI/9);
        sleep(4000);
        robot.setDrivePower(0,0,0);
        OpenGLMatrix robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);
        while ((robotPosition == null )&& opModeIsActive() ){
            idle();
            robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);
            telemetry.addData("In Loop" ,"");
            telemetry.update();
        }
        telemetry.addData("Out Loop","");
        telemetry.update();
        //Once code is here it is now in front of the beacon and has tried to press the button.
        //Now to adjust in the -x to the left to get to the second beacon for a distance of 45.5 inches.
        while(robotPosition == null){
            idle();
            robot.setDrivePower(0,0,0);
            robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);
            telemetry.addData("NullPos loop","");
            telemetry.update();
            // Do a recovery here.
        }
        zxPhi = VuforiaNav.GetZXPH(robotPosition);
        telemetry.addData("Starting Nav","");
        telemetry.update();
        while (opModeIsActive() && zxPhi[0] >= 19) { //Was 15 before changing to 21 for testing.
            float[] newSpeeds = getCorrectedSpeeds(zxPhi[1], zxPhi[2], v, 0);
            robot.setDriveSpeed(newSpeeds[0], newSpeeds[1], newSpeeds[2]);
            idle();
            robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);
            if(robotPosition != null) {
                zxPhi = VuforiaNav.GetZXPH(robotPosition);
            }
        }
        robot.setDrivePower(0, 0, 0);
        float[] colorValues = robot.getColorValues();
        if(colorValues != null) {
            telemetry.addData("Color Values: ", "Red: %.1f Green: %.1f Blue: %.0f", colorValues[0],
                    colorValues[1], colorValues[2]);
            telemetry.update();
        }

        if(robot.isRightBeaconBlue()){
            robot.RightPusher.setPosition(1);
            sleep(1000);
            telemetry.addData("Pushing right beacon","");
            robot.RightPusher.setPosition(0);
        }
        else if((robot.isRightBeaconRed()) && !robot.isRightBeaconBlue()){
            robot.LeftPusher.setPosition(1);
            sleep(1000);
            telemetry.addData("Pushing left beacon","");
            robot.LeftPusher.setPosition(0);
        }
        else{
            telemetry.addData("Unsure what color it is.","");
        }
        telemetry.update();

        //Drive to second color beacon.
        robot.setDrivePower(-50,-30,0); //was -50 -40

        OpenGLMatrix robotPosition2 = vuforianav.getRobotLocationRelativeToTarget(1);
        ElapsedTime driveTime2 = new ElapsedTime();
        while ((robotPosition2 == null )&& driveTime2.milliseconds() < 1000){ //was 1500
            idle();
            robotPosition2 = vuforianav.getRobotLocationRelativeToTarget(1);
        }
        //Once code is here it is now in front of the beacon and has tried to press the button.
        //Now to adjust in the -x to the left to get to the second beacon for a distance of 45.5 inches.
        int counter = 0;
        while(robotPosition2 == null){
            counter = counter+1;
            idle();
            robotPosition2 = vuforianav.getRobotLocationRelativeToTarget(1);
            robot.setDriveSpeed(0,0,Math.PI/12);
            telemetry.addData("Null pos2","");
            DbgLog.msg("Robot Debug: Robot Pos2 Trigger: Number of while loop runs: < %d >", counter);
            telemetry.update();
            // Do a recovery here. WIP
        }
        zxPhi = VuforiaNav.GetZXPH(robotPosition2);
        while (opModeIsActive() && zxPhi[0] >= 19) { //Was 15 before changing to 21 for testing.
            float[] newSpeeds2 = getCorrectedSpeeds(zxPhi[1], zxPhi[2], v, 0);
            robot.setDriveSpeed(newSpeeds2[0], newSpeeds2[1], newSpeeds2[2]);
            idle();
            robotPosition2 = vuforianav.getRobotLocationRelativeToTarget(1);
            if(robotPosition2 != null) {
                zxPhi = VuforiaNav.GetZXPH(robotPosition2);
            }
        }
        robot.setDrivePower(0, 0, 0);
        colorValues = robot.getColorValues();
        if(colorValues != null) {
            telemetry.addData("Color Values: ", "Red: %.1f Green: %.1f Blue: %.0f", colorValues[0],
                    colorValues[1], colorValues[2]);
            telemetry.update();
        }
        if(robot.isRightBeaconBlue()){
            robot.RightPusher.setPosition(1);
            sleep(1000);
            telemetry.addData("Pushing right beacon","");
            robot.RightPusher.setPosition(0);
        }
        else if(robot.isRightBeaconRed() && !robot.isRightBeaconBlue()){
            robot.LeftPusher.setPosition(1);
            sleep(1000);
            telemetry.addData("Pushing left beacon","");
            robot.LeftPusher.setPosition(0);
        }
        else{
            telemetry.addData("Unsure what color it is.","");
        }
        robot.setDrivePower(0,0,0);
        telemetry.addData("","Program has been finished in %f seconds.",getRuntime());
        telemetry.update();
    }

    public float[] getCorrectedSpeeds(float x,float phi,float v, float x0) {
        float phiPrime = VuforiaNav.remapAngle(phi-(float)Math.PI);
        float va = -phiPrime*Math.abs(v)*C_PHI;
        float vx = -C_X*Math.abs(v)*(x-x0)*(float)Math.cos(phiPrime);
        float vy = v+Math.abs(v)*C_X*(x-x0)*(float)Math.sin(phiPrime);
        return new float[]{vx,vy,va};
    }

    public void turnToPosition (float angle, float tolerance, float latency){
        //Tolerance in degrees latency seconds.

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = 90;
        float heading = -robot.sensorGyro.getIntegratedZValue();
        float targetHeading = heading + angle;
        float offset = targetHeading - heading;

        while (opModeIsActive()&& Math.abs(offset) > tolerance){
            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;
            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            robot.setDriveSpeed(0,0,va*Math.PI/180.0);
            heading = -robot.sensorGyro.getIntegratedZValue();
            offset = targetHeading - heading;
        }
        robot.setDrivePower(0,0,0);
    }
    //test


}

