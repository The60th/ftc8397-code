package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;

/**
 * Created by JimLori on 11/6/2016.
 */

@Autonomous(name = "TestJewelDiscrimination", group = "Test")
//@Disabled
public class TestJewelDiscrimination extends LinearOpMode {

//    final OpenGLMatrix CAMERA_LOCATION_ON_ROBOT =
//            OpenGLMatrix.translation(0, 0, 0).multiplied(
//                    Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
//                            AngleUnit.DEGREES, 90, 0, 0));
//    final VuforiaLocalizer.CameraDirection CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.BACK;
//
//    VuforiaNavigator vuforiaNavigator = null;

    @Override
    public void runOpMode()  {

//        vuforiaNavigator = new VuforiaNavigator("FTC_2016-17", FTCField.TARGET_LOCATIONS, FTCField.TARGET_NAMES,
//                CAMERA_LOCATION_ON_ROBOT, CAMERA_DIRECTION);

        ElapsedTime etv = new ElapsedTime();
        VuMarkNavigator.activate();
//        vuforiaNavigator.activate();
        telemetry.addData("Vuforia Activation Time", "%.3f", etv.seconds());
        telemetry.update();

        waitForStart();


        ElapsedTime et = new ElapsedTime();

        double numIter = 0;
        double getImageBytesTime = 0;
        double getReduceTime = 0;
        double getBinaryImageTime = 0;
        double findBlobsTime = 0;
        int nLoops = 0;

        int blobSizeThreshhold = 50;
        int sampleRatio = 4;
        int imgWidth = 640;
        int imgHeight = 480;
        int reducedImgWidth = imgWidth / sampleRatio;
        int reducedImgHeight = imgHeight / sampleRatio;
        byte[] imageBytes = new byte[2 * imgWidth * imgHeight];
        byte[] reducedImageBytes = new byte[2 * reducedImgWidth * reducedImgHeight];
        int[] binaryRed = new int[reducedImgWidth * reducedImgHeight];
        int[] binaryBlue = new int[reducedImgWidth * reducedImgHeight];
        ArrayList<Blob> redBlobs = null;
        ArrayList<Blob> blueBlobs = null;
        BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = VuMarkNavigator.getFrameQueue();




        while (opModeIsActive()) {
            nLoops++;

            //Get array of RGB565 pixels (two bytes per pixel) from the last frame on the frame queue
            et.reset();
            boolean gotBytes = VuMarkNavigator.getRGB565Array(frameQueue, imgWidth, imgHeight, imageBytes);
            if (!gotBytes){
                continue;
            }
            getImageBytesTime += et.milliseconds();

            //First, reduce the imgWidthximgHeight image to reducedImgWidthxreducedImgHeight by selecting every 4th row and every 4th column
            //Then, get reducedImgWidthxreducedImgHeight binary image
            et.reset();
            ImgProc.getReducedRangeRGB565(imageBytes,imgWidth,imgHeight,0,0,imgWidth,imgHeight,reducedImageBytes,sampleRatio);
            ImgProc.getBinaryImage(reducedImageBytes,330,30,0.5f,1.0f,0.1f,1.0f,binaryRed);
            ImgProc.getBinaryImage(reducedImageBytes,200,280,0.5f,1.0f,0.1f,1.0f,binaryBlue);
            getBinaryImageTime += et.milliseconds();

            //Find StatBlobs in the 80x60 float image
            et.reset();
            redBlobs = Blob.findBlobs(binaryRed, reducedImgWidth, reducedImgHeight);
            blueBlobs = Blob.findBlobs(binaryBlue, reducedImgWidth, reducedImgHeight);
            findBlobsTime = et.milliseconds();

            numIter += 1.0;

            //Filter out small blobs
            for (int i = redBlobs.size()-1; i >= 0; i--) if (redBlobs.get(i).getNumPts() < blobSizeThreshhold) redBlobs.remove(i);
            for (int i = blueBlobs.size()-1; i >= 0; i--) if (blueBlobs.get(i).getNumPts() < blobSizeThreshhold) blueBlobs.remove(i);

            //Take only the right-most red blob
            while (redBlobs.size() > 1) {
                if (redBlobs.get(0).getAvgX() < redBlobs.get(1).getAvgX()) redBlobs.remove(0);
                else redBlobs.remove(1);
            }


            if (redBlobs.size() > 0 && blueBlobs.size() > 0){
                Blob redBlob = redBlobs.get(0);
                for (int i = 1; i < redBlobs.size(); i++) if (redBlobs.get(i).getRectArea() > redBlob.getRectArea()) redBlob = redBlobs.get(i);
                Blob blueBlob = blueBlobs.get(0);
                for (int i = 1; i < blueBlobs.size(); i++) if (blueBlobs.get(i).getRectArea() > blueBlob.getRectArea()) blueBlob = blueBlobs.get(i);

                telemetry.addData("Arrangement", "%s", blueBlob.getAvgX()<redBlob.getAvgX()? "BLUE LEFT" : "BLUE RIGHT");
                telemetry.addData("NumBlobs","Num Red = %d  Num Blue = %d", redBlobs.size(), blueBlobs.size());
                telemetry.addData("Red ", " x = %.0f y = %.0f w = %.0f h = %.0f", redBlob.getAvgX(), redBlob.getAvgY(),
                        redBlob.getWidth(), redBlob.getLength());
                telemetry.addData("Blue","x = %.0f y = %.0f w = %.0f h = %.0f", blueBlob.getAvgX(), blueBlob.getAvgY(),
                        blueBlob.getWidth(), blueBlob.getLength());
            }
            else telemetry.addData("Could Not Find Both Blobs","");

            telemetry.update();

            nLoops = 0;

        }
    }
}