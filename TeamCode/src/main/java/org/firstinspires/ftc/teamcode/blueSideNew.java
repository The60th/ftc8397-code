package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Created by CanAdirondack on 1/31/2017.
 */
@Autonomous(name = "Blue_Side_New", group = "Autonomous")

public class blueSideNew extends OmniBotAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
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

        launchBall(); //Launch ball.

        driveStraightGyroTime(0.0f,-30.0f,1200.0f); //Drive forward for about 10.5 inches with shooters front. //105 inchs //10.5 inches per second. //not 30,0,1 not -30,0,1: maybe 0,30,1?

        turnToHeadingGyro(90.0f,3.0f,.5f); //Turn so phone is facing wall and vuforia target.

        driveStraightGyroTime(0.0f,30.0f,2300.0f); //Drive forward towards the wall.

        driveStraightGyroTime(-30.0f,0,3200.0f); //Slide to the left in front of the robot.

        OpenGLMatrix robotPos = searchForVuforia(0);

        if(robotPos == null)return;

         float[] zxPhi = VuforiaNav.GetZXPH(robotPos);

        vuforiaNavigateToTarget(0,zxPhi,25.0f,13.5f,-5.0f);

    }
}
