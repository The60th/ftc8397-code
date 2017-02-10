package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Program notes:
 * /** Should be used for final program comments, and code explaining.
 * // or /* Should only be used for notes when working on code, or reversion notes.
 * Current problems should be listed with a To-Do statement.
 * <p>
 * Final version Telemetry data should be ONLY used for important driver data.
 * All current Telemetry data MUST be replaced with "DbgLog.msg()" to reduce run time lag.
 * <p>
 * <p>
 * Program notes should be finished before final ship of code.
 */


/**
 * Notice:
 * This program should be considered locked and as such no changes made without valid reasons.
 * Any changes made should be fully documented with former code left in a comment and reasons for such a change.
 *
 */
@Disabled
@SuppressWarnings("all")
@Autonomous(name="Red_Side", group="Autonomous")
public class AutonomousRedSide extends  LinearOpMode{

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
        ElapsedTime gyroTime = new ElapsedTime();
        while(opModeIsActive() && robot.sensorGyro.isCalibrating()){
            telemetry.addData("Gyro Cali","");
            telemetry.update();
            idle();
        }
        telemetry.addData("Gyro activated. ","Took %f milliseconds. ",gyroTime.milliseconds());
        telemetry.addData("About to activate Vuforia","");
        ElapsedTime vufTime = new ElapsedTime();
        vuforianav.activate();
        telemetry.addData("Vuforia activated. ","Took %f milliseconds. ",vufTime.milliseconds());
        telemetry.update();
        waitForStart();


        robot.setDriveSpeed(0,-40,0); //Drive forward.
        sleep(850);

        robot.setDriveSpeed(0,0,0);

        fireGun();
        sleep(100);


        turnToPosition(-90-robot.sensorGyro.getIntegratedZValue(), 3, .3f);
        //Run ball firing code.
        //null void


       // robot.setDriveSpeed(30,0,0); This was moved to after 35,35,0.
        // sleep(1000);

        //robot.setDriveSpeed(0, 43.3, -Math.PI/6);
        //not - -
        //not + -
        //not - +
        // + +
        robot.setDriveSpeed(35,35,0);
        sleep(2500);
        robot.setDriveSpeed(30,0,0);
        sleep(1000);


        robot.setDrivePower(0,0,0,"");

        //Fetch robotPos
        OpenGLMatrix robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);

        int counter = 0;
        //Null Loop check.
        while ((robotPosition == null) && opModeIsActive()) {
            counter = counter + 1;
            idle();
            robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);
            robot.setDriveSpeed(0, 0, -Math.PI / 12);//changed to -
            DbgLog.msg("Robot Debug: Robot Pos2 Trigger: Number of while loop runs: < %d >", counter);
        }


        //Once code is here it is now in front of the beacon and has tried to press the button.
        //Now to adjust in the -x to the left to get to the second beacon for a distance of 45.5 inches.


        //Null Loop check.

        //Get zxPhi from robotPos
        zxPhi = VuforiaNav.GetZXPH(robotPosition);


        while (opModeIsActive() && zxPhi[0] >= robot.vuforiaZDistance -1) {
            float[] newSpeeds = getCorrectedSpeeds(zxPhi[1], zxPhi[2], v, 0);
            robot.setDriveSpeed(newSpeeds[0], newSpeeds[1], newSpeeds[2]);
            idle();
            robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);
            if(robotPosition != null) {
                zxPhi = VuforiaNav.GetZXPH(robotPosition);
            }
        }

        robot.setDrivePower(0, 0, 0,"");

        /*float[] colorValues = robot.getColorValues();

        No Longer In Use.

        if(colorValues != null) {
            telemetry.addData("Color Values: ", "Red: %.1f Green: %.1f Blue: %.0f", colorValues[0],
                    colorValues[1], colorValues[2]);
            telemetry.update();
        }*/

        if(robot.isRightBeaconRed()){
            robot.RightPusher.setPosition(1);
            sleep(1000);
            robot.RightPusher.setPosition(0);
        }
        else if((robot.isRightBeaconBlue()) && !robot.isRightBeaconRed()){
            robot.LeftPusher.setPosition(1);
            sleep(1000);
            robot.LeftPusher.setPosition(0);
        }

        //Drive to second color beacon.
        //not - +
        //not + +
        // + -
        robot.setDrivePower(50,-50,0,""); //was -50 -40

        OpenGLMatrix robotPosition2 = vuforianav.getRobotLocationRelativeToTarget(1);
        ElapsedTime driveTime2 = new ElapsedTime();

        while ((robotPosition2 == null )&& driveTime2.milliseconds() < 1250){ //was 1500
            idle();
            robotPosition2 = vuforianav.getRobotLocationRelativeToTarget(1);
        }
        //Once code is here it is now in front of the beacon and has tried to press the button.
        //Now to adjust in the -x to the left to get to the second beacon for a distance of 45.5 inches.
         counter = 0;
        while(robotPosition2 == null&& opModeIsActive()){
            counter = counter+1;
            idle();
            robotPosition2 = vuforianav.getRobotLocationRelativeToTarget(1);
            robot.setDriveSpeed(0,0,-Math.PI/12);//changed to -
            DbgLog.msg("Robot Debug: Robot Pos2 Trigger: Number of while loop runs: < %d >", counter);
            // Do a recovery here. WIP
        }

        zxPhi = VuforiaNav.GetZXPH(robotPosition2);
        while (opModeIsActive() && zxPhi[0] >= robot.vuforiaZDistance) { //Was 15 before changing to 21 for testing.
            float[] newSpeeds2 = getCorrectedSpeeds(zxPhi[1], zxPhi[2], v, 0);
            robot.setDriveSpeed(newSpeeds2[0], newSpeeds2[1], newSpeeds2[2]);
            idle();
            robotPosition2 = vuforianav.getRobotLocationRelativeToTarget(1);
            if(robotPosition2 != null) {
                zxPhi = VuforiaNav.GetZXPH(robotPosition2);
            }
        }

        robot.setDrivePower(0, 0, 0,"");

        /*colorValues = robot.getColorValues();

        No Longer In Use

        if(colorValues != null) {
            telemetry.addData("Color Values: ", "Red: %.1f Green: %.1f Blue: %.0f", colorValues[0],
                    colorValues[1], colorValues[2]);
            telemetry.update();
        }*/

        if(robot.isRightBeaconRed()){
            robot.RightPusher.setPosition(1);
            sleep(1000);
            robot.RightPusher.setPosition(0);
        }
        else if(robot.isRightBeaconBlue() && !robot.isRightBeaconRed()){
            robot.LeftPusher.setPosition(1);
            sleep(1000);
            robot.LeftPusher.setPosition(0);
        }

        robot.setDrivePower(0,0,0,"");

        v = -30;

        ElapsedTime nullTime = new ElapsedTime();
        while (opModeIsActive() && zxPhi[0] <= 100) {
            float[] newSpeeds2 = getCorrectedSpeeds(zxPhi[0], zxPhi[1], zxPhi[2], v, 102,127);
            robot.setDriveSpeed(newSpeeds2[0], newSpeeds2[1], newSpeeds2[2]);
            idle();
            robotPosition2 = vuforianav.getRobotLocationRelativeToTarget(1);
            if(robotPosition == null) telemetry.addData("","Null Pos for backup:");
            telemetry.addData("","vX = %f vY = %f vA = %f",-newSpeeds2[0],newSpeeds2[1],-newSpeeds2[2]);
            telemetry.update();
            /**
             * Added ElaspedTime tracker to track total time the robot has gone before a vuforia check could be made, if the robot has gone 1000 ms without a vuforia lock, stop the robot.
             */
            if(robotPosition2 == null){
                if(nullTime.milliseconds() > 1000){
                    robot.setDriveSpeed(0,0,0);
                    break;
                }
            }
            if(robotPosition2 != null) {
                zxPhi = VuforiaNav.GetZXPH(robotPosition2);
            }
        }

        telemetry.addData("", "Program has been finished in %f seconds.", getRuntime());
        telemetry.update();
    }

    public float[] getCorrectedSpeeds(float x,float phi,float v, float x0) {
        float phiPrime = VuforiaNav.remapAngle(phi-(float)Math.PI);
        float va = -phiPrime*Math.abs(v)*C_PHI;
        float vx = -C_X*Math.abs(v)*(x-x0)*(float)Math.cos(phiPrime);
        float vy = v+Math.abs(v)*C_X*(x-x0)*(float)Math.sin(phiPrime);
        return new float[]{vx,vy,va};
    }
    public void fireGun() throws InterruptedException{
        robot.setShooter(1.0);
        sleep(250);
        robot.setLaunchServo("Up");
        sleep(1000);
        robot.setLaunchServo("Down");
        sleep(1250);
        robot.setLaunchServo("Up");
        sleep(1000);
        robot.setLaunchServo("Down");
        robot.setShooter(0.0);
        //wait for second ball
        //lift again.
        //wax vjy
    }

    public float[] getCorrectedSpeeds(float z, float x, float phi, float v, float xT, float zT){
        final float CD = 0.1f;
        final float CPHI = 0.01f;
        float phiPrime = VuforiaNav.remapAngle(phi-(float)Math.PI);
        float errorD = (x * zT - z * xT)/((float)Math.sqrt(xT * xT + zT * zT));
        float phiPrimeT = (float)Math.atan2(xT, zT);
        float errorPhiPrime = phiPrime - phiPrimeT;
        float vABS = Math.abs(v);
        float vX = -CD * vABS * errorD * (float)Math.cos(errorPhiPrime);
        float vY = v + CD * vABS * errorD * (float)Math.sin(errorPhiPrime);
        float vA = -CPHI * Math.abs(v) * errorPhiPrime;
        //if(z < 10 || z> 80) vA = -CPHI * Math.abs(v) * phiPrime;
        if(z > 80) vY-= 10;

        return new float[] {-vX, vY, -vA};//added - to x and a
        //Not +vX -Vy, vA
        //Not -vX,vY,vA
        // -vX,vY,-vA

    }


    public void turnToPosition (float angle, float tolerance, float latency ){
        //Tolerance in degrees latency seconds.

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = 135;
        float heading = robot.sensorGyro.getIntegratedZValue();
        float targetHeading = heading + angle;
        float offset = targetHeading - heading;

        while (opModeIsActive()&& Math.abs(offset) > tolerance){
            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;
            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            DbgLog.msg("Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            robot.setDriveSpeed(0,0,va*Math.PI/180.0);
            heading = robot.sensorGyro.getIntegratedZValue();
            offset = targetHeading - heading;
        }
        robot.setDrivePower(0,0,0,"");
    }
}



