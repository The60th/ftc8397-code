package org.firstinspires.ftc.teamcode.competition_in_work.driver_control;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.cv_programs.Blob;
import org.firstinspires.ftc.teamcode.cv_programs.ImgProc;
import org.firstinspires.ftc.teamcode.mechbot.presupers_bot.MechBotSensor;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;

/**
 * Created by JimLori on 11/30/2017.
 */

@TeleOp(name = "PreMatchRobotAlignment", group = "Debug")
public class PreMatchRobotAlignment extends LoggingLinearOpMode {

    enum JewelSide {BLUE_LEFT, RED_LEFT, UNKNOWN}
    Orientation orientation;
    @Override
    public void runLoggingOpmode() throws InterruptedException {
        MechBotSensor bot = new MechBotSensor();
        bot.init(hardwareMap);
        telemetry.addData("Starting Vuforia: ", "Wait for flash light to start program.");
        telemetry.update();
        VuMarkNavigator.activate();

        //Turn flashlight on. Found I needed this in my office after dark to get adequate read on blue.
        //May not be necessary under normal competition conditions, especially if value and saturation
        //threshholds are set pretty low for blue.
        CameraDevice.getInstance().setFlashTorchMode(true);

        waitForStart();
        telemetry.addData("Program started. ", "");
        telemetry.update();
        //CameraDevice.getInstance().setFlashTorchMode(true);
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

        JewelSide jewelSide;


        while (opModeIsActive()) {

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

                jewelSide = blueBlob.getAvgX() < redBlob.getAvgX() ? JewelSide.BLUE_LEFT : JewelSide.RED_LEFT;
            }
            else jewelSide = JewelSide.UNKNOWN;

            OpenGLMatrix robotPose = VuMarkNavigator.getRobotPoseRelativeToTarget();

            telemetry.addData("Jewel Side", jewelSide);
            if (robotPose == null) telemetry.addData("","Robot Pose Unknown");
            else{
                float[] poseData = robotPose.getData();
                float heading = (float)Math.atan2( poseData[8], poseData[10]) * 180.0f/(float)Math.PI;
                telemetry.addData("Robot Pose"," Heading = %.1f Z = %.1f X = %.1f", heading, poseData[14]/10.0, poseData[12]/10.0);

                orientation = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //WAS ZYX
                telemetry.addData("Gyro Heading degrees: ",orientation.firstAngle);
                telemetry.addData("Roll degrees: ",orientation.secondAngle);
                telemetry.addData("Pitch degrees: ", orientation.thirdAngle);
            }
            float[] HSV = new float[3];
            Color.RGBToHSV(bot.colorLeft.red(), bot.colorLeft.green(), bot.colorLeft.blue(),HSV);
            telemetry.addData("Color left", "H = %.2f S = %.2f V = %.2f", HSV[0],HSV[1],HSV[2]);
            Color.RGBToHSV(bot.colorRight.red(), bot.colorRight.green(), bot.colorRight.blue(),HSV);
            telemetry.addData("Color right", "H = %.2f S = %.2f V = %.2f", HSV[0],HSV[1],HSV[2]);
            telemetry.update();

        }

        //Turn flashlight back off before exiting.
        CameraDevice.getInstance().setFlashTorchMode(false);

    }
}
