package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.concurrent.BlockingQueue;

/**
 * Created by JimLori on 9/12/2017.
 */

@Autonomous(name = "TestImageFileOutput", group = "Test")
public class TestImageFileOutput extends LinearOpMode {
    private int width = 1280;
    private int height = 720;
    private boolean gotImage;
    @Override
    public void runOpMode() {

        VuMarkNavigator.activate();

        BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = VuMarkNavigator.getFrameQueue();

        byte[] imageBytesInitial = new byte[2* width * height];

        byte[] imageBytes = new byte[2 * width * height];

        telemetry.addData("Press start when ready to save image...","");
        telemetry.update();

        waitForStart();

        VuMarkNavigator.getRGB565Array(frameQueue, width, height,imageBytesInitial);

        sleep(500);

        if(VuMarkNavigator.getRGB565Array(frameQueue, width, height, imageBytes)) {

            byte[] widthHeight = new byte[]{(byte) (width % 256), (byte) (width / 256), (byte) (height % 256), (byte) (height / 256)};

            Context context = hardwareMap.appContext;

            FileOutputStream fos = null;

            try {

                try {
                    fos = new FileOutputStream("/sdcard/DCIM/ImageBytes.dat", true);
                    fos.write(widthHeight);
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
         else{
            telemetry.addData("Failed to get image","");
        }
            telemetry.update();


        while (opModeIsActive()) continue;



    }

}
