package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Created by CanAdirondack on 1/29/2017.
 */
@Autonomous(name = "TestAutoOpMode", group = "Test Opmodes")

public class NewAutoTestOpMode extends OmniBotAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
       // vuforiaNav = new VuforiaNav();
       // vuforiaNav.activate();
        robot.sensorGyro.calibrate();

        while (robot.sensorGyro.isCalibrating()) {
            idle();
        }

        waitForStart();
        //OpenGLMatrix robotPos = searchForVuforia(0);

        //if(robotPos == null)return;

       // float[] zxPhi = VuforiaNav.GetZXPH(robotPos);

        driveStraightGyroTime(30,0,10000); //105 inchs //10.5 inches per second.
       // boolean success = vuforiaNavigateToTarget(0, zxPhi, 20, 14, -3);

    }
}
