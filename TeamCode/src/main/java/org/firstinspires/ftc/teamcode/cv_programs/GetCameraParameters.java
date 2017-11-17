package org.firstinspires.ftc.teamcode.cv_programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

/**
 * Created by JimLori on 11/6/2016.
 *
 * Gets and displays some basic camera parameters. Each parameter has two values, one for the
 * x direction (e.g., width), and one for the y direction (e.g., height). These parameters must
 * be used for CryptoNav navigation. The parameters are:
 *
 * fovRad: camera field of view (x and y), in radians; this will be converted to degrees for display.
 *
 * size: camera resolution in pixels (width and height).
 *
 * pp: camera principal point (x,y). This is the position in the image of the center point of the lens.
 *     You'd think it would just be (width/2, height/2), and it is pretty close to that.
 *
 * fl: camera focal length (x,y), in pixels.
 *
 * As a reality check, the "expected X focal length" is also computed: width/(2 * Math.tan(fovRad[0]/2).
 * This should come out pretty close to fl[0]
 *
 *
 */

@Autonomous(name = "GetCameraParameters", group = "Test")
//@Disabled
public class GetCameraParameters extends LinearOpMode {
    @Override
    public void runOpMode() {

        VuMarkNavigator.activate();
        float[] fovRad = CameraDevice.getInstance().getCameraCalibration().getFieldOfViewRads().getData();
        float[] size = CameraDevice.getInstance().getCameraCalibration().getSize().getData();
        float[] pp = CameraDevice.getInstance().getCameraCalibration().getPrincipalPoint().getData();
        float fl[] = CameraDevice.getInstance().getCameraCalibration().getFocalLength().getData();
        double expectedFL = size[0]/(2.0 * Math.tan(fovRad[0] / 2.0));
        telemetry.addData("FOV Degrees"," x: %.1f degrees  y: %.1f degrees", fovRad[0]*180.0/Math.PI, fovRad[1]*180.0/Math.PI);
        telemetry.addData("Size"," width: %.1f pixels  height: %.1f pixels", size[0], size[1]);
        telemetry.addData("Principal Point"," x: %.1f y: %.1f", pp[0], pp[1]);
        telemetry.addData("Focal Length"," x: %.1f y: %.1f", fl[0], fl[1]);
        telemetry.addData("Expected X Focal Length", " %.1f", expectedFL);

        telemetry.update();

        waitForStart();
    }

}
