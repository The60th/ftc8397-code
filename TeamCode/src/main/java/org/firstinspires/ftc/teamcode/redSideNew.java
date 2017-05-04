package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Created by CanAdirondack on 2/4/2017.
 */

@Autonomous(name = "Red_Side_New", group = "Autonomous")
//@Disabled
public class redSideNew extends OmniBotAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        final int blueOne = OmniBot.blueTargets[0];
        final int blueTwo = OmniBot.blueTargets[1];

        final int redOne = OmniBot.redTargets[0];
        final int redTwo = OmniBot.redTargets[1];
        robot.init(hardwareMap);
        vuforiaNav = new VuforiaNav();
        vuforiaNav.activate();
        robot.sensorGyro.calibrate();
        while (robot.sensorGyro.isCalibrating()) {
            idle();
        }

        telemetry.addData("","All systems ready!");
        telemetry.update();
        if(DEBUG) DbgLog.msg("<Debug> Start of program run");

        waitForStart(); //Wait for start

        if(DEBUG) DbgLog.msg("<Debug> Start of section one. Shoot balls and drive to vuforia range.");

        launchBall(); //Launch ball.

        driveStraightGyroTime(0.0f,-30.0f,1200.0f); //Drive forward for about 10.5 inches with shooters front. //105 inchs //10.5 inches per second. //not 30,0,1 not -30,0,1: maybe 0,30,1?

        turnToHeadingGyro(-90.0f,3.0f,.5f); //Turn so phone is facing wall and vuforia target.

        driveStraightGyroTime(0.0f,30.0f,2500.0f); //Drive forward towards the wall.

        driveStraightGyroTime(34.0f,0,3200.0f); //Slide to the right in front of the robot. 2/6/17 changed to 34 from 3-

        /*
        *
        * Break in-between shooting and first color beacon.
        * Robot is parked in aways from the first beacon ready for vuforia drive.
        *
        *
         */
        if(DEBUG) DbgLog.msg("<Debug> Start of section two. Drive to first Beacon.");
        OpenGLMatrix robotPos = searchForVuforia(redOne);

        if(robotPos == null){
            if(DEBUG)DbgLog.msg("<Debug> !! <Fail> Robot Vuforia critical error, force exited the program. <Fail>");
            robot.setDriveSpeed(0,0,0);
            return;
        }

        float[] zxPhi = VuforiaNav.GetZXPH(robotPos);

        vuforiaNavigateToTarget(redOne,zxPhi,25.0f,13.5f,-5.0f); //OmniBot.bluetargets[0] = first blue target

        handleBeacon(OmniBotAutonomous.BeaconColor.Red,redOne);

        /*
        *
        * Break in-between color beacon one, and color beacon two.
        * Robot is parked in front of first beacon.
        *
         */

        if(DEBUG) DbgLog.msg("<Debug> Start of section three. Drive to second Beacon.");

        robotPos = checkForVuforia(redOne,500);
        if(robotPos != null){
            zxPhi = VuforiaNav.GetZXPH(robotPos);
            vuforiaBackWardsDemo(redOne,zxPhi,25.0f,40.0f,-5.0f);
            telemetry.addData("<Debug> Vuforia drive for back up.","");
        }
        else{
            //Drive backwords with gyro drive.
            driveStraightGyroTime(0.0f,-30.0f,3000.0f); //Backward 20cm
            telemetry.addData("<Debug> Gyro drive for back up.","");
            //Side ways 40cm.
        }
        telemetry.update();
        //Start of using redTwo.

        driveStraightGyroTime(44.0f,0,3000.0f); //Sideways 40cm

        //At second vuforia target?

        robotPos = searchForVuforia(redTwo);

        if(robotPos == null){
            if(DEBUG)DbgLog.msg("<Debug> !! <Fail> Robot Vuforia critical error, force exited the program. <Fail>");
            robot.setDriveSpeed(0,0,0);
            return;
        }

        zxPhi = VuforiaNav.GetZXPH(robotPos);

        vuforiaNavigateToTarget(redTwo,zxPhi,25.0f,13.5f,-5.0f);

        handleBeacon(OmniBotAutonomous.BeaconColor.Red,redTwo);

        robotPos = checkForVuforia(redTwo,100);
        if(robotPos != null){
            zxPhi = VuforiaNav.GetZXPH(robotPos);
            vuforiaBackWardsDemo(redTwo,zxPhi,25.0f,20.0f,-5.0f); //Fixed?
            telemetry.addData("<Debug> Vuforia drive for back up.","");
        }
        else{
            //Drive backwords with gyro drive.
            telemetry.addData("<Debug> Gyro drive for back up.","");
            driveStraightGyroTime(0.0f,-10.0f,500.0f); //Backward 20cm
            //Side ways 40cm.
        }
        telemetry.update();

    }
}

