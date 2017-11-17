package org.firstinspires.ftc.teamcode.cv_programs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

import java.util.concurrent.BlockingQueue;

/**
 * Created by JimLori on 11/6/2016.
 *
 * This demonstrates use of CryptoNav for navigation using the cryptobox rails.
 */

@Autonomous(name = "TestCryptoBoxNavigation", group = "Test")
//@Disabled
public class TestCryptoBoxNavigation extends LinearOpMode {

    final String TAG = "TST_CRYPT";

    /*
    Image parameters used here are based on my Nexus 7 tablet; different ones will be needed for the G4.

    These parameters must be supplied to the CryptoNav class using the initParams(...) method. rawImgWidth,
    rawImgHeight, rawFocalLength, and rawPrincipalX can be obtained by running the GetCameraParameters
    opMode. Alternatively, an overload of initParams(...) is provided that will obtain these automatically.

    rangeX0, rangeY0, rangeWidth, and rangeHeight must be selected by the user of CryptoNav -- they indicate a
    sub-range of the raw image that is actually analyzed for navigation. A smaller subrange will result
    in faster execution. I've noticed that there tends to be a little "noise" at far left edge of the G4
    image, which can result in extraneous blobs. Using a subrange can trim the edges. Vertical trimming
    could also exclude blobs resulting from floor tape.

    sampleRatio also must be selected by the user of CryptoNav. This is how many rows and columns are skipped
    between pixels that are used for finding blobs. Also a big time saver. A sampleRatio of at least 4 is
    recommended; this results in a factor of 16 increased computation speed. Note that sampleRatio must be
    a common denominator of rangeWidth and rangeHeight. For example, if you used rangeWidth=1200,
    rangeHeight=240, then a sampleRatio of 2, 4, 5, 6, 8, or 10 would work. But if you changed rangeWidth to 1210,
    then sampleRatio of 2, 5 or 10 would still work, but 4, 6, and 8 would not.

    The key navigation method is CryptoNav.updateLocationZX, which returns a float[] containing the current
    (z,x) coordinates of the phone. These are just like the z and x we used for vuforia targets previously.
    The origin of the (z,x) system is the front-center of the cryptobox. The z axis points straight into the field.
    The x axis points to the right (as viewed from inside the field).

    Unlike vuforia, CryptoNav.updateLocationZX does not give you the heading (phiPrime). Instead, you pass a
    known phiPrime (from the gyro) into this method. This allows it to correct for the effect of heading on
    the location of the rails in the image. For this demonstration, phiPrime is assumed to be zero.

    CryptoNav assumes that the camera is in a horizontal (landscape) orientation. Which way should you rotate
    from the usual portrait orientation? If the phone display is facing back toward you, rotate counterclockwise
    90 degrees.

    Besides tweaks related to the G4 phone, some tweaks may be required to the HSV ranges used for the red and
    blue rails. I've been testing this on a paper image of four red bars. I've noticed (using the BlobTest C#
    program) that the actual red rails have a higher level of saturation (always very close to 1.0) than my
    paper image (closer to 0.6 or 0.7). Increasing the minimum saturation could help improve distinction between
    rails and other extraneous objects. Optimal HSV ranges for blue rails would be obtained by capturing images
    with the TestImageFileOutput op mode and examining with the BlobTest program. Blue objects always seem to be
    darker than red ones, so a lower minimum V will almost certainly be needed.

    To change the HSV ranges for red and blue rails, get into the CryptoNav class and change the RED_RANGE and
    BLUE_RANGE constants.

    Another possibility for tweaks: the filtering of blobs that is done to get a set of "rails" from an initial set
    of multiple blobs. Because of the white tape across the rails, each rail gets divided into multiple blobs. These
    need to be combined. We also need to exclude the tape on the floor, as well as any tiny extraneous blobs.
    This is handled in the CryptoNav.filterRailBlobs method.
     */

    private int sampleRatio = 4;
    private int rawImgWidth = 1280; //use 1280 for G4 //640
    private int rawImgHeight = 720; //use 720 for G4 //480
    private int rangeX0 = 40; //Consider using 40 for G4 //0
    private int rangeY0 = 240; //Consider using 240 for G4 //0
    private int rangeWidth = 1200; //Consider using 1200 for G4 //640
    private int rangeHeight = 240; //Consider using 240 for G4 //480
    private float rawFocalLength = 548f; //Don't know what this will be for G4-Run GetCameraParameters to find out
    private float rawPrincipalX = 640f;  //Should be roughly 640 for G4 //320f

    //This array of bytes will hold each new raw RGB565 image
    private byte[] imageBytes = new byte[2 * rawImgWidth * rawImgHeight];

    @Override
    public void runOpMode()  {

        ElapsedTime etv = new ElapsedTime();

        //FIRST:  Activate VuMarkNavigator
        VuMarkNavigator.activate();
        telemetry.addData("Vuforia Activation Time", "%.3f", etv.seconds());
        telemetry.update();

        //SECOND: Pass required parameters in to CryptoNav
        CryptoNav.initParams(CryptoNav.TeamColor.RED, rawImgWidth, rawImgHeight, rangeX0, rangeY0, rangeWidth, rangeHeight,
                rawFocalLength, rawPrincipalX, sampleRatio);


        //THIRD: obtain the frame que from VuMarkNavigator
        BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = VuMarkNavigator.getFrameQueue();

        //FOURTH: Wait for start.
        //Before the start button is pressed, it is critical that either the left-most rail (when viewed from inside the
        //field) or the right most rail be visible on the camera image. It is also just fine if all rails are visible.
        //The CryptoNav.initRails(...) method needs to be told whether it is the left-most rail or the right-most rail
        //that will be visible initially. This is done by passing in either CryptoNav.Side.LEFT or CryptoNav.Side.RIGHT.
        //If all rails are visible, then it doesn't matter whether you pass in LEFT vs. RIGHT.
        waitForStart();

        //FIFTH: Clear the frame queue so we can be sure the next image obtained is up to date.
        VuMarkNavigator.clearFrameQueue(frameQueue);
        RobotLog.dd(TAG, "FrameQueue Cleared");

        //SIXTH: Initialize Rails. Why is a loop needed here? Because after clearing the frame queue, it
        //will take a fraction of a second for a new frame to become available.
        boolean cryptoNavInitialized = false;
        while (opModeIsActive()){
            RobotLog.dd(TAG,"Try Initialize");
            //Try to get an image; if image not yet available, loop and try again.
            if (!VuMarkNavigator.getRGB565Array(frameQueue, rawImgWidth, rawImgHeight, imageBytes)) continue;

            //Try to initialize the rails. We're assuming for this demonstration that the left-most
            //rail is visible to the camera before initializeRails is called.
            if(!CryptoNav.initializeRails(imageBytes, CryptoNav.Side.LEFT)) break;
            cryptoNavInitialized = true;
            break;
        }

        if (cryptoNavInitialized) RobotLog.dd(TAG, "CryptoNav Initialization Succeeded.");
        else {
            RobotLog.dd(TAG, "CryptoNav Initialization Failed.");
            while (opModeIsActive()) continue;
        }

        ElapsedTime et = new ElapsedTime();

        //SEVENTH: Enter the actual navigation loop. In practice, each iteration of this loop would include adjustments
        //of motor powers, just as we did with Vuforia navigation.
        while (opModeIsActive()) {

            //Get a new image; if no image is a available, keep on trying.
            if (!VuMarkNavigator.getRGB565Array(frameQueue, rawImgWidth, rawImgHeight, imageBytes)) continue;

            //From the image, use the CryptoNav.updateLocationZX method to obtain new z,x coordinates.
            //Note that this method does not GIVE us the phiPrime heading value. Instead, we need to provide the
            //method with the phiPrime value. In practice, we would obtain this from the GYRO. For this
            //demonstration, it will be assumed that phiPrime is 0 (just keep the phone pointed directly toward the
            //wall.
            float[] zx = CryptoNav.updateLocationZX(imageBytes, 0f);

            //To improve readability of the display, display new coordinates only twice per second.
            if (et.milliseconds() < 500) continue;

            et.reset();

            //Note: if navigation fails, that's usually because only zero or one rails is currently visible.
            //The updateLocationZX method needs to "see" at least two rails. No problem; once two rails become
            //visible again, we should start getting valid locations again.
            if (zx == null) telemetry.addData("Navigation Failed","");
            else telemetry.addData("Navigation Succeeded"," z = %.1f  x = %.1f", zx[0], zx[1]);

            telemetry.update();
        }
    }
}