package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by CanAdirondack on 2/4/2017.
 */
@Disabled
@Autonomous(name = "Auto Color testing", group = "Auto")

public class colorSensorTest extends OmniBotAutonomous {
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

        handleBeacon(BeaconColor.Blue,0);

    }
}
