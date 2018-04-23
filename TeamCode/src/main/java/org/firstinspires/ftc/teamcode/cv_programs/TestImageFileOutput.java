package org.firstinspires.ftc.teamcode.cv_programs;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.concurrent.BlockingQueue;

/**
 * Created by JimLori on 9/12/2017.
 */

@Autonomous(name = "TestImageFileOutput", group = "Test")
//@Disabled
@Disabled
public class TestImageFileOutput extends LinearOpMode {

    @Override
    public void runOpMode() {

        VuMarkNavigator.activate(true);

        //Automatically determine the resolution of the camera
        float size[] = CameraDevice.getInstance().getCameraCalibration().getSize().getData();
        int imgWidth = Math.round(size[0]);
        int imgHeight = Math.round(size[1]);

        //Need frame queue to capture images
        BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = VuMarkNavigator.getFrameQueue();

        //Array of bytes to hold the RGB565 image
        byte[] imageBytes = new byte[2 * imgWidth * imgHeight];

        telemetry.addData("Press start when ready to save image...","");
        telemetry.update();

        CameraDevice.getInstance().setFlashTorchMode(true);

        waitForStart();

        //Clear the frame queue so we get a brand new image, rather than one that has been
        //waiting in the queue.
        VuMarkNavigator.clearFrameQueue(frameQueue);

        boolean gotImage = false;

        //After clearing frame queue, it may take a while (maybe 50 millisec) for a new image to become
        //available. Loop until a new image is available, then save it.
        while (opModeIsActive() && !gotImage)
            gotImage = VuMarkNavigator.getRGB565Array(frameQueue, imgWidth, imgHeight, imageBytes);

        if (gotImage){

            //The application context is available through the hardware map and is needed for file operations.
            Context context = hardwareMap.appContext;

            FileOutputStream fos = null;

            //This awkward nested try/catch/finally block is needed because fos.close() can itself throw an exception

            try {

                try {
                    fos = new FileOutputStream("/sdcard/DCIM/ImageBytes.dat", true); //"true" means to append if file already exists

                    //this array just holds the image dimensions for writing to the file
                    byte[] bytes = new byte[]{(byte)(imgWidth&0xFF), (byte)(imgWidth>>8), (byte)(imgHeight&0xFF), (byte)(imgHeight>>8)};

                    //Write the image dimensions, then the actual image data to the file
                    fos.write(bytes);
                    fos.write(imageBytes);
                } catch (FileNotFoundException exc) {
                    telemetry.addData("File Not Found", "");
                } catch (java.io.IOException exc) {
                    telemetry.addData("IO Exception", "");
                } finally {
                    if (fos != null) fos.close();
                }
                telemetry.addData("Success!", "");

            } catch (java.io.IOException exc) {
                telemetry.addData("IO Exception on Close", "");
            }
        }

        telemetry.update();

        //Done writing file. Now enter a do-nothing loop until user stops the opmode

        CameraDevice.getInstance().setFlashTorchMode(false);

        while (opModeIsActive()) continue;

    }

}
