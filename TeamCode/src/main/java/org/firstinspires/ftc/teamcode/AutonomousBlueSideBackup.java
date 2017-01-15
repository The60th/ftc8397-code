package org.firstinspires.ftc.teamcode;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Created by CanAdirondack on 1/9/2017.
 */
@Autonomous(name = "Backup_Auto", group = "Autonomous")
public class AutonomousBlueSideBackup extends LinearOpMode {
    OmniBot robot = new OmniBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        sleep(10000);
        //start

        robot.setDriveSpeed(0,-40,0);
        sleep(1750);
        robot.setDriveSpeed(0,0,0);
        sleep(100);

        fireGun();

        sleep(100);
        robot.setDriveSpeed(0,-40,0);
        sleep(3000);




    }
    public void fireGun() throws InterruptedException {
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

    }
}

