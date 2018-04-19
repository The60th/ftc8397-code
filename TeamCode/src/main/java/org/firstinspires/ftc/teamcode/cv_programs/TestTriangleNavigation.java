package org.firstinspires.ftc.teamcode.cv_programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

import java.util.concurrent.BlockingQueue;

/**
 * Created by JimLori on 11/6/2016.
 *
 * This demonstrates use of CryptoNav for navigation using the cryptobox rails.
 */

@Autonomous(name = "TestTriangleNavigation", group = "Test")
//@Disabled
public class TestTriangleNavigation extends LinearOpMode {

    final String TAG = "TST_TRIANGLE_NAV";

    /*
    Image parameters used here are based on my Nexus 7 tablet; different ones will be needed for the G4.

    These parameters must be supplied to the TriangleNav class using the initParams(...) method. rawImgWidth,
    rawImgHeight, rawFocalLength, and rawPrincipalX can be obtained by running the GetCameraParameters
    opMode. Alternatively, an overload of initParams(...) is provided that will obtain these automatically.

    rangeX0, rangeY0, rangeWidth, and rangeHeight must be selected by the user of TriangleNav -- they indicate a
    sub-range of the raw image that is actually analyzed for navigation. A smaller subrange will result
    in faster execution. I've noticed that there tends to be a little "noise" at far left edge of the G4
    image, which can result in extraneous blobs. Using a subrange can trim the edges. Vertical trimming
    could also exclude blobs resulting from floor tape.

    sampleRatio also must be selected by the user of TriangleNav. This is how many rows and columns are skipped
    between pixels that are used for finding blobs. Also a big time saver. A sampleRatio of at least 4 is
    recommended; this results in a factor of 16 increased computation speed. Note that sampleRatio must be
    a common denominator of rangeWidth and rangeHeight. For example, if you used rangeWidth=1200,
    rangeHeight=240, then a sampleRatio of 2, 4, 5, 6, 8, or 10 would work. But if you changed rangeWidth to 1210,
    then sampleRatio of 2, 5 or 10 would still work, but 4, 6, and 8 would not.

    The key navigation method is TriangleNav.getCameraLocationZX, which returns a float[] containing the current
    (z,x) coordinates of the camera. These are just like the z and x we used for vuforia targets previously.
    The origin of the (z,x) system is the front-center of the cryptobox. The z axis points straight into the field.
    The x axis points to the right (as viewed from inside the field).

    Unlike vuforia, CryptoNav.updateLocationZX does not give you the heading (phiPrime). Instead, you pass a
    known phiPrime (from the gyro) into this method. This allows it to correct for the effect of heading on
    the location of the rails in the image. For this demonstration, phiPrime is assumed to be zero.

    TriangleNav assumes that the camera is in a horizontal (landscape) orientation. Which way should you rotate
    from the usual portrait orientation? If the phone display is facing back toward you, rotate counterclockwise
    90 degrees.

    Besides tweaks related to the G4 phone, some tweaks may be required to the HSV ranges used for the red and
    blue rails. I've been testing this on a paper image of four red bars. I've noticed (using the BlobTest C#
    program) that the actual red rails have a higher level of saturation (always very close to 1.0) than my
    paper image (closer to 0.6 or 0.7). Increasing the minimum saturation could help improve distinction between
    rails and other extraneous objects. Optimal HSV ranges for blue rails would be obtained by capturing images
    with the TestImageFileOutput op mode and examining with the BlobTest program. Blue objects always seem to be
    darker than red ones, so a lower minimum V will almost certainly be needed.

    To change the HSV ranges for red and blue rails, get into the TriangleNav class and change the RED_RANGE and
    BLUE_RANGE constants.

    Another possibility for tweaks: change the TriangleNav.getTriangleBlobFromRawImage method. Currently, it merges
    together all blobs with number of points greater than some threshhold.
     */

    private int sampleRatio = 4;
    private int rawImgWidth = 1280; //use 1280 for G4 //640
    private int rawImgHeight = 720; //use 720 for G4 //480
    private int rangeX0 = 40; //Consider using 40 for G4 //0
    private int rangeY0 = 0; //Consider using 240 for G4 //0
    private int rangeWidth = 1200; //Consider using 1200 for G4 //640
    private int rangeHeight = 720; //Consider using 240 for G4 //480
    private float rawFocalLength = 1082f; //Correct for g4
    private float rawPrincipalX = 640f;  //Should be roughly 640 for G4 //320f

    //This array of bytes will hold each new raw RGB565 image
    private byte[] imageBytes = new byte[2 * rawImgWidth * rawImgHeight];

    //Camera location on robot: (x,y) coordinates of camera in robot coordinate system
    float[] cameraLocOnRobot = new float[] { 0, - 7.0f * 2.54f };

    @Override
    public void runOpMode()  {

        ElapsedTime etv = new ElapsedTime();

        //FIRST:  Activate VuMarkNavigator
        VuMarkNavigator.activate(true);
        telemetry.addData("Vuforia Activation Time", "%.3f", etv.seconds());
        telemetry.update();

        //SECOND: Pass required parameters in to CryptoNav
        TriangleNav.initParams(TriangleNav.TeamColor.RED, rawImgWidth, rawImgHeight, rangeX0, rangeY0, rangeWidth, rangeHeight,
                rawFocalLength, rawPrincipalX, sampleRatio);


        //THIRD: obtain the frame que from VuMarkNavigator
        BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = VuMarkNavigator.getFrameQueue();

        CameraDevice.getInstance().setFlashTorchMode(true);

        //FOURTH: Wait for start.
        waitForStart();

        //FIFTH: Clear the frame queue so we can be sure the next image obtained is up to date.
        VuMarkNavigator.clearFrameQueue(frameQueue);
        RobotLog.dd(TAG, "FrameQueue Cleared");

        ElapsedTime et = new ElapsedTime();

        //SIXTH: Enter the actual navigation loop. In practice, each iteration of this loop would include adjustments
        //of motor powers, just as we did with Vuforia navigation.
        while (opModeIsActive()) {

            //Get a new image; if no image is a available, keep on trying.
            if (!VuMarkNavigator.getRGB565Array(frameQueue, rawImgWidth, rawImgHeight, imageBytes)) continue;

            //Get the triangle blob
            Blob triangleBlob = TriangleNav.getTriangleBlobFromRawImage(imageBytes);

            //get the  camera zx coordinates; for this demo, assume gyro heading = 0
            float gyroHeading = 0.0f;
            float[] zxCamera = TriangleNav.getCameraLocationZX(triangleBlob, getCameraPhiPrimeFromGyroHeading(gyroHeading));

            //Camera is not at robot center; here's how to get coordinates of robot center from camera coordinates
            float[] zxRobot = getRobotZXfromCameraZX(zxCamera, gyroHeading);

            //Display new coordinates up to 10 times per second.
            if (et.milliseconds() < 100) continue;

            et.reset();

            //Note: if navigation fails, that's usually because only zero or one rails is currently visible.
            //The updateLocationZX method needs to "see" at least two rails. No problem; once two rails become
            //visible again, we should start getting valid locations again.
            if (zxCamera == null) telemetry.addData("Navigation Failed","");
            else {
                telemetry.addData("Navigation Succeeded"," zCam = %.1f  xCam = %.1f", zxCamera[0], zxCamera[1]);
                telemetry.addData("Robot Coords", " zRobot = %.1f  xRobot = %.1f", zxRobot[0], zxRobot[1]);
            }

            telemetry.update();
        }

        CameraDevice.getInstance().setFlashTorchMode(false);
    }

    //In practice, these two methods would be implemented in the robot hardware class, and the
    //implementation would be dependent on robot configuration.
    private float getCameraPhiPrimeFromGyroHeading(float gyroHeading) { return gyroHeading; }
    private float getOdomHeadingFromGyroHeading (float gyroHeading) { return gyroHeading; }

    //This method would have to be added to the op mode
    private float[] getRobotZXfromCameraZX(float[] zxCamera, float gyroHeading ){
        if (zxCamera == null) return null;
        float odomHeading = getOdomHeadingFromGyroHeading(gyroHeading);
        float sin = (float)Math.sin(odomHeading);
        float cos = (float)Math.cos(odomHeading);
        float zRobot = zxCamera[0] - cameraLocOnRobot[0] * sin - cameraLocOnRobot[1] * cos;
        float xRobot = zxCamera[1] + cameraLocOnRobot[0] * cos - cameraLocOnRobot[1] * sin;
        return new float[] {zRobot, xRobot};
    }

}