package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Created by CanAdirondack on 11/9/2016.
 */
@Autonomous(name=" TestVuforiaNavClass: Test ", group="Test")

public class TestVuforiaNavClass extends LinearOpMode {
    VuforiaNav vuforianav = null;

    @Override
    public void runOpMode() throws InterruptedException {

        vuforianav = new VuforiaNav();
        vuforianav.activate();
        waitForStart();
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive()){
            if (et.milliseconds() > 500){
                OpenGLMatrix robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);
                if ( robotPosition != null){
                    float[] data = robotPosition.getData();
                    float x = data[12];
                    float z = data[14];
                    float phi = (float)Math.atan2(data[4],data[6])*180.0f/(float)Math.PI;
                    telemetry.addData("position ", "x = %.0f y = %.0f phi = %.0f",x,z,phi);
                    telemetry.update();
                }
                et.reset();
            }

            idle();
        }
    }
}
