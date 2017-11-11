package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;

/**
 * Created by FTC Team 8397 on 11/10/2017.
 */

public abstract class MechBotAutonomous extends LinearOpMode {

    protected MechBotSensor bot = new MechBotSensor();
    private enum LineFollowSide {LEFT,RIGHT}
    private final float INNER_TAPE_ANGLE = 33.70f;
    private final float INNER_TAPE_ANGLE_RADS = INNER_TAPE_ANGLE * ((float)Math.PI/180.0f);
    private final float LINE_FOLLOW_SPEED = 10.0f; //10 centimeters per second.
    private final float LINE_FOLLOW_ANGLE_FACTOR = 30.0f * ((float)Math.PI/180.0f); //30.0 Degrees converted to radians.
    private final float HEADING_CORECTION_FACTOR = 2.0f;
    private enum JewlSide{BLUE_LEFT,RED_LEFT,UNKNOWN}

    protected void setFlashOn(){
        CameraDevice.getInstance().setFlashTorchMode(true);
    }
    protected void setFlashOff(){
        CameraDevice.getInstance().setFlashTorchMode(false);
    }

    private void followLineProportionate(LineFollowSide side, ColorSensor colorSensor){
        float[] hsvValues = new float[3];
        final float coeff = 20.0f;
        while (opModeIsActive()){
            float heading = bot.getHeadingRadians();
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            float err = side == LineFollowSide.LEFT? 0.5f - hsvValues[1] : hsvValues[1] - 0.5f;
            if (err < -0.5) err = -0.4f;
            else if (err > 0.5) err = 0.4f;
            float angleDiff = side == LineFollowSide.LEFT? heading - INNER_TAPE_ANGLE_RADS : heading + INNER_TAPE_ANGLE_RADS;
            float vx = LINE_FOLLOW_SPEED * (float)Math.sin(angleDiff) + coeff * err * (float)Math.cos(angleDiff);
            float vy = LINE_FOLLOW_SPEED * (float)Math.cos(angleDiff) - coeff * err * (float)Math.sin(angleDiff);
            float va = -heading * HEADING_CORECTION_FACTOR;
            bot.setDriveSpeed(vx, vy, va);
        }
    }

    //Using gyro, turns robot the specified number of degrees (angle), with acceptable error
    //of +/- tolerance degrees, assuming a latency between new gyro readings of "latency" seconds
    protected void turnAngleGyro(float angle, float tolerance, float latency) {
        //Tolerance in degrees latency seconds.
        //Convert to radians.
        angle = angle * (float)Math.PI/180f;
        tolerance = tolerance * (float)Math.PI/180f;

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = 0.2f * (float)Math.PI;
        float heading = bot.getHeadingRadians();
        float targetHeading = heading + angle;
        float offset = (float)VuMarkNavigator.NormalizeAngle(targetHeading - heading);

        while (opModeIsActive() && Math.abs(offset) > tolerance) {
            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;
            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            RobotLog.a("Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            bot.setDriveSpeed(0, 0, va);
            heading = bot.getHeadingRadians();
            offset = targetHeading - heading;
        }
        bot.setDrivePower(0, 0, 0);
    }

    //Turns robot to a specific integratedZ heading using Gyro, targetHeading in degrees
    protected void turnToHeadingGyro(float targetHeading, float tolerance, float latency){
        //Tolerance in degrees latency seconds.
        tolerance = tolerance * (float)Math.PI/180f;
        targetHeading = targetHeading * (float)Math.PI/180f;

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = 0.75f * (float)Math.PI;
        float heading = bot.getHeadingRadians();
        float offset = (float)VuMarkNavigator.NormalizeAngle(targetHeading - heading);

        while (opModeIsActive() && Math.abs(offset) > tolerance) {
            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;
            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            RobotLog.a("Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            bot.setDriveSpeed(0, 0, va);
            heading = bot.getHeadingRadians();
            offset = targetHeading - heading;
        }
        bot.setDrivePower(0, 0, 0);

    }

    protected void driveStraightGyroTime(float vx, float vy, float duration) {
        final float C_ANGLE = 2.0f;
        final float initialHeading = bot.getHeadingRadians();
        ElapsedTime et = new ElapsedTime();
        double scale = bot.setDriveSpeed(vx, vy, 0);
        RobotLog.a("<Debug> DriveStrGyroTime Pre Scale duration = %.0f vx = %.0f vy = %.0f", duration, vx, vy);

        duration /= scale;
        vx *= scale;
        vy *= scale;
        RobotLog.a("<Debug> DriveStrGyroTime Post Scale( %.2f ) duration = %.0f vx = %.0f vy = %.0f", scale, duration, vx, vy);

        while(opModeIsActive()){
            double etms = et.milliseconds();
            if(etms > duration) break;
            float currentHeading = bot.getHeadingRadians();
            float va = (initialHeading - currentHeading) * C_ANGLE;
            RobotLog.a("<Debug> Initial Angle = %.1f Current Angle = %.1f", initialHeading, currentHeading);
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0, 0);

    }
    //Time out, time in ms.
    public JewlSide findJewl(double timeOut){
        JewlSide returnSide;
        int blobSizeThreshhold = 400; //Blobs smaller than this will be discarded
        int sampleRatio = 5; //Number of rows and columns to skip between raw pixels selected for reduced image

        //Get resolution automatically from camera
        float[] size = CameraDevice.getInstance().getCameraCalibration().getSize().getData();
        int imgWidth = Math.round(size[0]);
        int imgHeight = Math.round(size[1]);
        byte[] imageBytes = new byte[2 * imgWidth * imgHeight];

        //Set up reduced image dimensions
        int reducedImgWidth = imgWidth / sampleRatio;
        int reducedImgHeight = imgHeight / sampleRatio;
        byte[] reducedImageBytes = new byte[2 * reducedImgWidth * reducedImgHeight];

        //int arrays for binary images for identifying the red and blue jewels
        int[] binaryRed = new int[reducedImgWidth * reducedImgHeight];
        int[] binaryBlue = new int[reducedImgWidth * reducedImgHeight];

        //ArrayLists to hold red and blue blobs
        ArrayList<Blob> redBlobs = null;
        ArrayList<Blob> blueBlobs = null;

        //Need a reference to the frame queue to get images
        BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = VuMarkNavigator.getFrameQueue();
        VuMarkNavigator.clearFrameQueue(frameQueue);

        ElapsedTime et = new ElapsedTime();
        et.startTime();

        while (opModeIsActive() && et.milliseconds() < timeOut) {

            //Get array of RGB565 pixels (two bytes per pixel) from the last frame on the frame queue.
            //If no image is available, keep looping
            boolean gotBytes = VuMarkNavigator.getRGB565Array(frameQueue, imgWidth, imgHeight, imageBytes);
            if (!gotBytes){
                continue;
            }

            //First, reduce the imgWidthximgHeight image to reducedImgWidth x reducedImgHeight by skipping rows and columns per sampleRatio
            //From the reduced RGB565 image, obtain the binary images for red and blue blob detection
            ImgProc.getReducedRangeRGB565(imageBytes,imgWidth,imgHeight,0,0,imgWidth,imgHeight,reducedImageBytes,sampleRatio);
            ImgProc.getBinaryImage(reducedImageBytes,345,15,0.7f,1.0f,0.3f,1.0f,binaryRed);
            ImgProc.getBinaryImage(reducedImageBytes,195,235,0.7f,1.0f,0.2f,1.0f,binaryBlue);

            //Get lists of red and blue blobs from the binary images
            redBlobs = Blob.findBlobs(binaryRed, reducedImgWidth, reducedImgHeight);
            blueBlobs = Blob.findBlobs(binaryBlue, reducedImgWidth, reducedImgHeight);

            //Filter out small blobs
            for (int i = redBlobs.size()-1; i >= 0; i--) if (redBlobs.get(i).getNumPts() < blobSizeThreshhold) redBlobs.remove(i);
            for (int i = blueBlobs.size()-1; i >= 0; i--) if (blueBlobs.get(i).getNumPts() < blobSizeThreshhold) blueBlobs.remove(i);

            //Take only the right-most red blob. This is to avoid problems with the VuMark, which may match the red range.
            while (redBlobs.size() > 1) {
                if (redBlobs.get(0).getAvgX() < redBlobs.get(1).getAvgX()) redBlobs.remove(0);
                else redBlobs.remove(1);
            }


            if (redBlobs.size() > 0 && blueBlobs.size() > 0){
                Blob redBlob = redBlobs.get(0); //There is only one blob left in the red list; this is it.

                //Find the largest blue blob; it will be assumed to be the blue jewel.
                Blob blueBlob = blueBlobs.get(0);
                for (int i = 1; i < blueBlobs.size(); i++) if (blueBlobs.get(i).getRectArea() > blueBlob.getRectArea()) blueBlob = blueBlobs.get(i);

                if(blueBlob.getAvgX() < redBlob.getAvgX()) return JewlSide.BLUE_LEFT;
                else return  JewlSide.RED_LEFT;

                //If blueBlob.getAvgX() < redBlob.getAvgX(), the blue blob is on the left
               // telemetry.addData("Arrangement", "%s", blueBlob.getAvgX()<redBlob.getAvgX()? "BLUE LEFT" : "BLUE RIGHT");
                //telemetry.addData("Red ", " x = %.0f y = %.0f w = %.0f h = %.0f", redBlob.getAvgX(), redBlob.getAvgY(),
                     //   redBlob.getWidth(), redBlob.getLength());
                //telemetry.addData("Blue","x = %.0f y = %.0f w = %.0f h = %.0f", blueBlob.getAvgX(), blueBlob.getAvgY(),
                     //   blueBlob.getWidth(), blueBlob.getLength());
            }
            //else telemetry.addData("Could Not Find Both Blobs","");

            //telemetry.update();

        }
        return JewlSide.UNKNOWN;
    }
}
