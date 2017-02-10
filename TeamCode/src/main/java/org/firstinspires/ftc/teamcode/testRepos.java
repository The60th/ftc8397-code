package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Created by CanAdirondack on 2/4/2017.
 */
@Disabled
@Autonomous(name = "RobotReposDemo", group = "Autonomous")
public class testRepos extends OmniBotAutonomous {
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

        waitForStart();

        OpenGLMatrix robotPos = vuforiaNav.getRobotLocationRelativeToTarget(0);
        while (opModeIsActive() && robotPos == null) robotPos = vuforiaNav.getRobotLocationRelativeToTarget(0);
        reposRobot(0);
    }
}
