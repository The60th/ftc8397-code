package org.firstinspires.ftc.teamcode;


import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Created by CanAdirondack on 1/31/2017.
 */
@Autonomous(name = "Shoot then Cap", group = "Autonomous")

public class BackUp_Blue extends OmniBotAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        //Init
        robot.init(hardwareMap);
        robot.sensorGyro.calibrate();
        while (robot.sensorGyro.isCalibrating()) {
            idle();
        }
        telemetry.addData("","All systems ready!");
        telemetry.update();
        waitForStart(); //Wait for start
        //Actions
        driveStraightGyroTime(0.0f, -30.0f, 2000.0f);
        launchBall();
        sleep(18000);
        driveStraightGyroTime(0.0f, -40.0f, 5000.0f); //hit ball
    }

}
