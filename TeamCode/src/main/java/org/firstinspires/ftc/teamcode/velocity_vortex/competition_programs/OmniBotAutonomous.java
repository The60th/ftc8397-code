package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;


/*
 *
 * Abstract base class for all Autonomous OpModes using the OmniBot Hardware class.
 * Provides basic functionality, to avoid having duplicate copies of method and variable
 * declarations in the various OpModes.
 */

public abstract class OmniBotAutonomous extends LinearOpMode {

    /**
     *
     *
     */
    public enum BeaconColor{Blue,Red,Unknown};

    /**
     *
     *
     */
    public enum  ButtonSide{Left,Right};


    //The robot
    protected OmniBot robot = new OmniBot();

    //VuforiaNav instance. Will need to be instantiated in the runOpMode method of the
    //inheriting concrete class
    protected VuforiaNav vuforiaNav = null;

    //Variable to indicate whether Debug Logging is turn on. This can be set in the constructor
    //or runOpMode method of the inheriting concrete class.
    static public boolean DEBUG = true;

    //Distance from target for Vuforia Navigation, in cm
    protected final float Z_TARGET = 20.0f;

    //X-OFFSET for Vuforia Navigation to target, in cm
    protected final float X_TARGET = 20.0f;


    //Using gyro, turns robot the specified number of degrees (angle), with acceptable error
    //of +/- tolerance degrees, assuming a latency between new gyro readings of "latency" seconds
    protected void turnAngleGyro(float angle, float tolerance, float latency) {
        //Tolerance in degrees latency seconds.

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = 135;
        float heading = robot.sensorGyro.getIntegratedZValue();
        float targetHeading = heading + angle;
        float offset = targetHeading - heading;

        while (opModeIsActive() && Math.abs(offset) > tolerance) {
            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;
            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            DbgLog.msg("Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            robot.setDriveSpeed(0, 0, va * Math.PI / 180.0);
            heading = robot.sensorGyro.getIntegratedZValue();
            offset = targetHeading - heading;
        }
        robot.setDrivePower(0, 0, 0, "");
    }

    //Turns robot to a specific integratedZ heading using Gyro, targetHeading in degrees
    protected void turnToHeadingGyro(float targetHeading, float tolerance, float latency){
        //Tolerance in degrees latency seconds.

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = 135;
        float heading = robot.sensorGyro.getIntegratedZValue();
        float offset = targetHeading - heading;

        while (opModeIsActive() && Math.abs(offset) > tolerance) {
            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;
            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            DbgLog.msg("Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            robot.setDriveSpeed(0, 0, va * Math.PI / 180.0);
            heading = robot.sensorGyro.getIntegratedZValue();
            offset = targetHeading - heading;
        }
        robot.setDrivePower(0, 0, 0, "");

    }



    //Use to obtain corrected speeds while navigating along a perpendicular straight line
    //toward the target/beacon. x and phi are the current x-coordinate and heading of the
    //robot in target-zx-space (typically obtained with Vuforia, but could also be obtained
    //through other means, including gyro, odometry). v is the desired nominal forward speed
    //of the robot. x0 is the desired x-offset.
    protected float[] getCorrectedSpeeds(float x, float phi, float v, float x0) {
        return null;
    }


    /**
     * @param targetIndex
     * @param zxPhi
     * @param vNominal
     * @param zTarget
     * @param xTarget
     * @return
     */
    //Navigate robot to specifiec vuforia target (targetIndex).
    //Initial coordinates must be known (argument zxPhi).
    //Returns true if successful, false if not.
    //Final coordinates (best known) will be stored in zxPhi upon return.
    //vNominal is the desired nominal forward speed
    //zTarget is the desired final distance from the target
    //xTarget is the desired final x-direction offset from the z axis.
	//CONSIDER THE FOLLOWING ENHANCEMENTS:
	//Incorporate odometry in case of transient Vuforia failure.
	//Initial slide in the +/- X direction (using Vuforia) to obtain better navigation toward target
    protected boolean vuforiaNavigateToTarget(int targetIndex, float[] zxPhi, float vNominal, /** Note for jim, make a reverse function here as well? You should be able to use the same code but give it a negtivie for x and drive backwards?*/
                                              float zTarget, float xTarget){
        final float C_PHI = 0.05f;
        final float C_X = 0.05f;
        float[] zxPhiLocal = new float[]{zxPhi[0],zxPhi[1],zxPhi[2]};
        robot.updateOdometry();

        //final float maxInitialX = 0.5f * (float)Math.exp(C_X * (zxPhiLocal[0] - zTarget));
        final float maxInitialX = 10;


        while(opModeIsActive() && Math.abs(zxPhiLocal[1]) > maxInitialX){
            OpenGLMatrix robotPos = vuforiaNav.getRobotLocationRelativeToTarget(targetIndex);
            if(robotPos != null){
                robot.updateOdometry();
                if(DEBUG) DbgLog.msg("<Debug> Vuforia x slide loop Successful");
                zxPhiLocal = VuforiaNav.GetZXPH(robotPos);
            }else{
                if(DEBUG) DbgLog.msg("<Debug> Vuforia x slide loop Failed");
                //Odem
                //zxPhiLocal = robot.updateOdometry(zxPhiLocal[0], zxPhiLocal[1], zxPhiLocal[2]);

            }
            float phiPrime = VuforiaNav.remapAngle(zxPhiLocal[2] - (float) Math.PI);
            float va = -phiPrime * vNominal * C_PHI;
            float vx = -Math.signum(zxPhiLocal[1] - xTarget) * vNominal * (float)Math.cos(phiPrime);
            float vy = Math.signum(zxPhiLocal[1] - xTarget) * vNominal * (float)Math.sin(phiPrime);
            robot.setDriveSpeed(vx, vy, va);

        }


        while(opModeIsActive() && zxPhiLocal[0] > zTarget){
            OpenGLMatrix robotPos = vuforiaNav.getRobotLocationRelativeToTarget(targetIndex);
            if(robotPos != null){
                robot.updateOdometry();
                if(DEBUG) DbgLog.msg("<Debug> Vuforia main loop Successful");
                zxPhiLocal = VuforiaNav.GetZXPH(robotPos);
            }else{
                if(DEBUG) DbgLog.msg("<Debug> Vuforia main loop Failed");
                //Odem
                //zxPhiLocal = robot.updateOdometry(zxPhiLocal[0], zxPhiLocal[1], zxPhiLocal[2]);

            }


            float phiPrime = VuforiaNav.remapAngle(zxPhiLocal[2] - (float) Math.PI);
            float va = -phiPrime * vNominal * C_PHI;
            float vx = vNominal * (float) Math.sin(phiPrime) - C_X * vNominal * (zxPhiLocal[1] - xTarget) * (float) Math.cos(phiPrime);
            float vy = vNominal * (float) Math.cos(phiPrime) + C_X * vNominal * (zxPhiLocal[1] - xTarget) * (float) Math.sin(phiPrime);
            if(DEBUG) DbgLog.msg("<Debug> Vuforia Main Loop zxPhiLocal = %.1f %.1f %.1f", zxPhiLocal[0], zxPhiLocal[1], zxPhiLocal[2] * 180.0/Math.PI);
            if(DEBUG) DbgLog.msg("<Debug> Vuforia Main Loop phiPrime = %.1f", phiPrime * 180.0/Math.PI);
            if(DEBUG) DbgLog.msg("<Debug> Vuforia Main Loop vx = %.1f, vy = %.1f, va = %.1f", vx, vy, va * 180.0/Math.PI);

            robot.setDriveSpeed(vx,vy,va);

        }

        zxPhi[0] = zxPhiLocal[0];
        zxPhi[1] = zxPhiLocal[1];
        zxPhi[2] = zxPhiLocal[2];
        robot.setDriveSpeed(0, 0, 0);
        return (zxPhiLocal[0] < zTarget);
    }

    //Starting from a position within or near the Vuforia Lock zone, follows an organized
    //search pattern to obtain Vuforia Lock.
    //Assumes robot is turned directly toward wall at the start.
    //If successful, returns the OpenGLMatrix that contains the robot position relative
    //to the target. If not, returns null.
    protected OpenGLMatrix searchForVuforia(int targetIndex){
        float searchDir;
        final float SCAN_DEGREES = 20f;
        final float SCAN_TOLERANCE = 3f;
        final float SCAN_LATENCY = 0.4f; //0.3 to low, measured around .27 //.6 works
        final float SCAN_DURATION = 800.0f;
        final float SHIFT_SPEED = 50.0f;
        final float X_SHIFT_DURATION = 1200.0f;
        final float Z_SHIFT_DURATION = 1000.0f;
        if(targetIndex % 2 == 0){
            //blue
            searchDir = -1.0f;
        }else{
            //red
            searchDir = 1.0f;
        }
        OpenGLMatrix robotpos = scanForVuforia(targetIndex, SCAN_DURATION, searchDir, SCAN_DEGREES, SCAN_TOLERANCE, SCAN_LATENCY);
        if(robotpos != null)  return robotpos;

        driveStraightGyroTime(SHIFT_SPEED * searchDir, 0, X_SHIFT_DURATION);

        robotpos = scanForVuforia(targetIndex, SCAN_DURATION, searchDir, SCAN_DEGREES, SCAN_TOLERANCE, SCAN_LATENCY);
        if(robotpos != null)  return robotpos;

        driveStraightGyroTime(0, SHIFT_SPEED, Z_SHIFT_DURATION);

        robotpos = scanForVuforia(targetIndex, SCAN_DURATION, searchDir, SCAN_DEGREES, SCAN_TOLERANCE, SCAN_LATENCY);
        if(robotpos != null)  return robotpos;

        driveStraightGyroTime(-SHIFT_SPEED * searchDir, 0, X_SHIFT_DURATION);

        robotpos = scanForVuforia(targetIndex, SCAN_DURATION, searchDir, SCAN_DEGREES, SCAN_TOLERANCE, SCAN_LATENCY);
        if(robotpos != null)  return robotpos;

        return null;
    }

    //Checks for Vuforia with robot in current position, for a maximum of duration milliseconds
    //If found, returns the robotLocationRelativeToTarget matrix. Otherwise, returns null
    protected OpenGLMatrix checkForVuforia(int targetIndex, float duration){
        ElapsedTime et = new ElapsedTime();
        OpenGLMatrix robotPos = vuforiaNav.getRobotLocationRelativeToTarget(targetIndex);
        while (opModeIsActive() && et.milliseconds() < duration && robotPos==null){
            robotPos = vuforiaNav.getRobotLocationRelativeToTarget(targetIndex);
        }
    return robotPos;
    }

    //Checks for Vuforia with robot in current position, AND turned +/-SCAN_DEGREES
    //Returns robotLocationRelativeToTarget matrix immediately if found. If not found, returns null
    //search direction negative = blue
    //search direction positive = red
    protected OpenGLMatrix scanForVuforia(int targetIndex, float duration, float searchDir,
                                          float scanDegrees, float scanTolerance, float scanLatency){
        OpenGLMatrix robotpos = checkForVuforia(targetIndex, duration);
        if(robotpos != null) return robotpos;
        float initialHeading = robot.sensorGyro.getIntegratedZValue();

        turnToHeadingGyro(initialHeading - searchDir * scanDegrees, scanTolerance, scanLatency);
        robotpos = checkForVuforia(targetIndex, duration);
        if(robotpos != null) return  robotpos;

        turnToHeadingGyro(initialHeading + searchDir * scanDegrees, scanTolerance, scanLatency);
        robotpos = checkForVuforia(targetIndex, duration);
        if(robotpos != null) return  robotpos;

        turnToHeadingGyro(initialHeading, scanTolerance, scanLatency);
        //robotpos = checkForVuforia(targetIndex, duration); Removed because of longer wait time.
        return robotpos;
    }

    /**
     * @param vx
     * @param vy
     * @param duration
     */
    //Drive in straight line (vx,vy), using Gyro to keep heading constant. Stop
    //after the specified number of milliseconds; if vx and vy are not possible
    //within the constraint of motor power <=1, adjust time up.
    protected void driveStraightGyroTime(float vx, float vy, float duration) {
        final float C_ANGLE = 2.0f * (float)Math.PI / 180.0f;
        final float initialHeading = robot.sensorGyro.getIntegratedZValue();
        ElapsedTime et = new ElapsedTime();
        double scale = robot.setDriveSpeed(vx, vy, 0);
        if(DEBUG)DbgLog.msg("<Debug> DriveStrGyroTime Pre Scale duration = %.0f vx = %.0f vy = %.0f", duration, vx, vy);

        duration /= scale;
        vx *= scale;
        vy *= scale;
        if(DEBUG)DbgLog.msg("<Debug> DriveStrGyroTime Post Scale( %.2f ) duration = %.0f vx = %.0f vy = %.0f", scale, duration, vx, vy);

        while(opModeIsActive()){
            double etms = et.milliseconds();
            if(etms > duration) break;
            float currentHeading = robot.sensorGyro.getIntegratedZValue();
            float va = (initialHeading - currentHeading) * C_ANGLE;
            if(DEBUG)DbgLog.msg("<Debug> Initial Angle = %.1f Current Angle = %.1f", initialHeading, currentHeading);
            robot.setDriveSpeed(vx, vy, va);
        }
        robot.setDriveSpeed(0, 0, 0);

    }

    //Drive in a straight line (vx, vy), using Gyro to keep heading constant. Stop
    //after the robot has travelled the specified number of centimeters (per odometry).
    //Passed in array xyTheta contains initial robot coordinates and heading for odometry. These
    //values are updated using gyro-assisted odometry.
    /*
    protected void driveStraightGyroDistance(float vx, float vy, float distance, float[] xyTheta) {
    }
    */

    //Added to launch the balls from the launcher.
    protected  void launchBall(){
        robot.setShooter(.60); //70 over shots at 2/9/17
        sleep(250);
        robot.setLaunchServo("Up");
        sleep(1000);
        robot.setLaunchServo("Down");
        sleep(1250);
        robot.setLaunchServo("Up");
        sleep(1000);
        robot.setLaunchServo("Down");
        robot.setShooter(0.0);
    }

    protected boolean pushButton(ButtonSide pushSide, BeaconColor intiLeftColor,BeaconColor intiRightColor){
        boolean colorChange = false;
        boolean leftPusher = false;
        if(pushSide == ButtonSide.Right){robot.RightPusher.setPosition(1); leftPusher = false;}
        else{robot.LeftPusher.setPosition(1);leftPusher = true;}
        sleep(500);

        ElapsedTime et = new ElapsedTime();

        robot.setDriveSpeed(0,10,0);

        while (opModeIsActive() && et.milliseconds() < 1000){
            BeaconColor leftColor = robot.getLeftColor();
            BeaconColor rightColor = robot.getRightColor();

            if(leftColor != intiLeftColor && intiLeftColor != BeaconColor.Unknown && leftColor != BeaconColor.Unknown) {colorChange =true; break;} //Boolean to see if we trigger early from a swap.
            if(rightColor != intiRightColor && intiRightColor != BeaconColor.Unknown && rightColor != BeaconColor.Unknown) {colorChange =true;  break;}
        }

        if(leftPusher)robot.LeftPusher.setPosition(0);
        else{robot.RightPusher.setPosition(0);}

        robot.setDriveSpeed(0,-10,0);

        sleep(200);
        robot.setDriveSpeed(0,0,0);
        return colorChange; //Not sure about the returns here.
    }

    public void handleBeacon(BeaconColor teamColor, int targetIndex){
        if(teamColor == BeaconColor.Unknown) return;
        BeaconColor wrongColor = teamColor == BeaconColor.Blue ? BeaconColor.Red : BeaconColor.Blue;
        int counter=0;
        int pushCount = 0;
        final int maxCounts = 2;

        BeaconColor leftColor = robot.getLeftColor();
        BeaconColor rightColor = robot.getRightColor();
        while (opModeIsActive() && counter < maxCounts){ //override if both wc
            if(leftColor == teamColor && rightColor == teamColor) break;
            else if(leftColor == wrongColor && rightColor == wrongColor){
                if(pushCount % 2 == 0)pushButton(ButtonSide.Right,leftColor,rightColor);
                else pushButton(ButtonSide.Left,leftColor,rightColor);
                pushCount +=1;
            }
            else if(leftColor == wrongColor){
                pushButton(ButtonSide.Right,leftColor,rightColor);
                pushCount+=1;
            }
            else if(rightColor == wrongColor){
                pushButton(ButtonSide.Left,leftColor,rightColor);
                pushCount+=1;
            }
            else if(leftColor == teamColor && rightColor == BeaconColor.Unknown && pushCount == 0){
                pushButton(ButtonSide.Left,leftColor,rightColor);
                pushCount+=1;
            }
            else if(rightColor == teamColor && leftColor == BeaconColor.Unknown && pushCount == 0){
                pushButton(ButtonSide.Right,leftColor,rightColor);
                pushCount+=1;
            }
            else if(leftColor == BeaconColor.Unknown && rightColor == BeaconColor.Unknown){
                //Drive around?
            }

            BeaconColor initLeftColor = leftColor;
            BeaconColor initRightColor = rightColor;

            leftColor = robot.getLeftColor();
            rightColor = robot.getRightColor();

            if((leftColor == teamColor && rightColor == teamColor) || (leftColor == teamColor && initLeftColor == wrongColor)
                    || (rightColor == teamColor && initRightColor == wrongColor)) break;

            if(leftColor != initLeftColor || rightColor != initRightColor){
                sleep(5000);
            }

            if(leftColor != wrongColor || rightColor != wrongColor){
                //move
               // reposRobot(targetIndex);
            }

            counter +=1;

        }

    }

    public void reposRobot(int targetIndex) {
        final float X_TOL = 1;
        final float Z_TOL = 1;
        final float PHI_TOL = .05f;
        final float zTarget = 13.5f;
        final float vNominal = 10;
        final float C_X = 10.0f;
        final float C_Z = 10.0f;
        final float C_PHI = 10.0f;
        final float xTarget = 0;
        final float phiTarget = 0;

        OpenGLMatrix robotPos = vuforiaNav.getRobotLocationRelativeToTarget(targetIndex);
        if (robotPos == null) return;

        driveStraightGyroTime(0, -10, 500);
        robotPos = vuforiaNav.getRobotLocationRelativeToTarget(targetIndex);
        if (robotPos == null) {
            if (DEBUG) DbgLog.msg("<Debug> Repos function failed to find vuforia after backup");
            driveStraightGyroTime(0, 10, 500);
            return;
        }

        float[] zxPhi;

        while (opModeIsActive()){
            robotPos = vuforiaNav.getRobotLocationRelativeToTarget(targetIndex);
            if (robotPos != null) {
                if (DEBUG) DbgLog.msg("<Debug> Vuforia Repos Loop Worked");
                zxPhi = VuforiaNav.GetZXPH(robotPos);
            } else {
                if (DEBUG) DbgLog.msg("<Debug> Vuforia Repos loop Failed");
                break;
            }

            float va;
            float vx;
            float vy;
            float phiPrime = VuforiaNav.remapAngle(zxPhi[2] - (float) Math.PI);
            if((Math.abs(zxPhi[0] - zTarget) < Z_TOL) && (Math.abs(zxPhi[1] - xTarget) < X_TOL) && (Math.abs(phiPrime-phiTarget) < PHI_TOL)) break;
            if(Math.abs(phiPrime) >= PHI_TOL)va = -.3f*Math.signum(phiPrime); else va=0;
            float vxF = Math.abs(zxPhi[1] - xTarget) >= X_TOL ? -5*Math.signum(zxPhi[1]-xTarget) : 0;
            float vzF = Math.abs(zxPhi[0] - zTarget) >= Z_TOL ? -5*Math.signum(zxPhi[0]-zTarget) : 0;

             vx =  -vzF * (float) Math.sin(phiPrime) + vxF * (float) Math.cos(phiPrime);
             vy =   -vzF * (float) Math.cos(phiPrime) - vxF* (float) Math.sin(phiPrime);
            if (DEBUG)
                DbgLog.msg("<Debug> Vuforia Main Repos zxPhiLocal = %.1f %.1f %.1f", zxPhi[0], zxPhi[1], zxPhi[2] * 180.0 / Math.PI);
            if (DEBUG)
                DbgLog.msg("<Debug> Vuforia Main Repos phiPrime = %.1f", phiPrime * 180.0 / Math.PI);
            if (DEBUG)
                DbgLog.msg("<Debug> Vuforia Main Repos vx = %.1f, vy = %.1f, va = %.1f", vx, vy, va * 180.0 / Math.PI);


            robot.setDriveSpeed(vx, vy, va);
        }
        robot.setDriveSpeed(0,0,0);



    }

    protected boolean vuforiaBackWardsDemo(int targetIndex, float[] zxPhi, float vNominal, /** Note for jim, make a reverse function here as well? You should be able to use the same code but give it a negtivie for x and drive backwards?*/
                                           float zTarget, float xTarget){
        final float C_PHI = 0.05f;
        final float C_X = 0.05f;
        float[] zxPhiLocal = new float[]{zxPhi[0],zxPhi[1],zxPhi[2]};
        robot.updateOdometry();

        while(opModeIsActive() && zxPhiLocal[0] < zTarget){
            OpenGLMatrix robotPos = vuforiaNav.getRobotLocationRelativeToTarget(targetIndex);
            if(robotPos != null){
                robot.updateOdometry();
                if(DEBUG) DbgLog.msg("<Debug> Vuforia main loop Successful");
                zxPhiLocal = VuforiaNav.GetZXPH(robotPos);
            }else{
                if(DEBUG) DbgLog.msg("<Debug> Vuforia main loop Failed");
                //Odem
                //zxPhiLocal = robot.updateOdometry(zxPhiLocal[0], zxPhiLocal[1], zxPhiLocal[2]);
            }


            float phiPrime = VuforiaNav.remapAngle(zxPhiLocal[2] - (float) Math.PI);
            float va = -phiPrime * vNominal * C_PHI;
            float vx = -vNominal * (float) Math.sin(phiPrime) - C_X * vNominal * (zxPhiLocal[1] - xTarget) * (float) Math.cos(phiPrime);
            float vy = -vNominal * (float) Math.cos(phiPrime) + C_X * vNominal * (zxPhiLocal[1] - xTarget) * (float) Math.sin(phiPrime);
            if(DEBUG) DbgLog.msg("<Debug> Vuforia Main Loop zxPhiLocal = %.1f %.1f %.1f", zxPhiLocal[0], zxPhiLocal[1], zxPhiLocal[2] * 180.0/Math.PI);
            if(DEBUG) DbgLog.msg("<Debug> Vuforia Main Loop phiPrime = %.1f", phiPrime * 180.0/Math.PI);
            if(DEBUG) DbgLog.msg("<Debug> Vuforia Main Loop vx = %.1f, vy = %.1f, va = %.1f", vx, vy, va * 180.0/Math.PI);

            robot.setDriveSpeed(vx,vy,va);
        }

        zxPhi[0] = zxPhiLocal[0];
        zxPhi[1] = zxPhiLocal[1];
        zxPhi[2] = zxPhiLocal[2];
        robot.setDriveSpeed(0, 0, 0);
        return (zxPhiLocal[0] < zTarget);
    }


}
