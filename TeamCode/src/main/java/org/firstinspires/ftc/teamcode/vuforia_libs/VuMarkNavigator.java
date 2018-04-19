package org.firstinspires.ftc.teamcode.vuforia_libs;

/**
 * Created by JimLori on 11/6/2016.
 */


import android.graphics.Bitmap;

import com.qualcomm.ftcrobotcontroller.R;
import com.vuforia.Box3D;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;

//import org.firstinspires.ftc.robotcore.internal.VuforiaLocalizerImpl;

public class VuMarkNavigator {
    public static Boolean isActive = false;
    private static VuforiaLocalizer vuforia;
    private static VuforiaTrackables targets;
    private static VuforiaTrackable target;
    private static final String TARGET_ASSET_NAME = "RelicVuMark";
    private static final OpenGLMatrix TARGET_LOCATION = OpenGLMatrix.translation(0,0,0).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,
            AxesOrder.XYX, AngleUnit.DEGREES, 0, 0, 0));
    private static final OpenGLMatrix PHONE_LOCATION_ON_ROBOT =
            OpenGLMatrix.translation(0,0,0).multiplied(
                    Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES,0,0,0));;
    private static final VuforiaLocalizer.CameraDirection CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.BACK;


    public static void activate(boolean viewOn){

        VuforiaLocalizer.Parameters parameters;
        //Create the VuforiaLocalizer
        if(viewOn){parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);}
        else{parameters = new VuforiaLocalizer.Parameters();}
        parameters.cameraDirection = CAMERA_DIRECTION;
        parameters.vuforiaLicenseKey = "Ae1jMu//////AAAAGeOTvbI/jknrpPvd1Jvqdk4+f9Csy0CC9PVlGo1o3pXA9ZN/v5SYrMDAzxObLuZ5pRWhZ9F6XZH5tkZcgPBMnb5pe9r1MgIPvxEC7IQ9mGWsnzGdGl5ABrOCR/wrog7l5YKJeMuRrlVRib9So+hurF1WchB8nlnuwQe3E7fiC+0M/wtz2qYRBxcH6dM7XKhPHNKpiyH5s82YXwIVMEwFtSQHQI68ghVvj3vnVVf5B0o4aUS8gn2/ygAa4uhEwUf3kmAiZO/kwGpdQGy3moOOdzBDyJ8KmGZIckcsRFGQdtQtgDVCoKxItGMCeH0vWyRmSp4dqLsUVnDQLWZfEP8axZ7UIWO+A8ppclFj7A1qehC6";
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        //Set up the VuforiaLocalizer to allow frame grabs of RGB565 images
        vuforia.setFrameQueueCapacity(3);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        //Obtain the single target and set its location (will use 0,0,0 and do all navigation relative to target position)
        targets = vuforia.loadTrackablesFromAsset(TARGET_ASSET_NAME);
        target = targets.get(0);
        target.setLocation(TARGET_LOCATION);
        ((VuforiaTrackableDefaultListener)target.getListener()).setPhoneInformation(PHONE_LOCATION_ON_ROBOT, CAMERA_DIRECTION);
        targets.activate();
        isActive = true;

    }

    public static void deactivate(){
        targets.deactivate();
        isActive = false;
    }

    //Return the RelicRecoveryVuMark code of the visualized target (RIGHT, CENTER, LEFT); UNKNOWN if no target found.
    public static RelicRecoveryVuMark getRelicRecoveryVumark(){
        return RelicRecoveryVuMark.from(target);
    }

    public static OpenGLMatrix getTargetPoseRelativeToRobot(){
        OpenGLMatrix targetPoseRelativeToCamera =
                ((VuforiaTrackableDefaultListener)target.getListener()).getPose();
        if (targetPoseRelativeToCamera == null) return null;
        return PHONE_LOCATION_ON_ROBOT.multiplied(targetPoseRelativeToCamera);
    }

    public static OpenGLMatrix getRobotPoseRelativeToTarget(){
        OpenGLMatrix targetPoseRelativeToCamera =
                ((VuforiaTrackableDefaultListener)target.getListener()).getPose();
        if (targetPoseRelativeToCamera == null) return null;
        return (PHONE_LOCATION_ON_ROBOT.multiplied(targetPoseRelativeToCamera)).inverted();
    }

    public static void setTargetLocation(OpenGLMatrix targetLocation){
        target.setLocation(targetLocation);
    }

    public static OpenGLMatrix getRobotLocation(){
        return ((VuforiaTrackableDefaultListener)target.getListener()).getRobotLocation();
    }


    //return: Z,X position and Phi, the angle between the Z axis of the coordinate system
    //being transformed to and the projection of the robot Y-axis into the ZX plane of the
    //coordinate system being transformed to.
    public static float[] getRobot_Z_X_Phi_FromLocationTransform(OpenGLMatrix locationTransform)
    {
        float[] locationData = locationTransform.getData();
        float[] returnValue = new float[3];
        returnValue[0] = locationData[14]/10.0f;
        returnValue[1] = locationData[12]/10.0f;
        returnValue[2] = (float)Math.atan2( locationData[4], locationData[6]);
        return returnValue;
    }



    public static double NormalizeAngle(double angle){
        double temp = (angle + Math.PI)/(2.0 * Math.PI);
        return (temp - Math.floor(temp) - 0.5) * 2.0 * Math.PI;
    }


    public static BlockingQueue<VuforiaLocalizer.CloseableFrame> getFrameQueue(){
        return vuforia.getFrameQueue();
    }

    public static void clearFrameQueue(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue){
        VuforiaLocalizer.CloseableFrame tempFrame = null;
        while (true) {
            tempFrame = frameQueue.poll();
            if (tempFrame == null) break;
            tempFrame.close();
        }
    }

    public static boolean getRGB565Array(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue, int width, int height, byte[] dst) {
        if (dst.length != (2 * width * height)) return false;
        VuforiaLocalizer.CloseableFrame frame = null;
        VuforiaLocalizer.CloseableFrame tempFrame = null;
        Image img = null;
        try{
            //We want the most recent available frame, which necessitates this while loop. If no frame is available, return false.
            while (true){
                tempFrame = frameQueue.poll();
                if (tempFrame == null) break;
                if (frame != null) frame.close();
                frame = tempFrame;
            }
            if (frame == null) return false;

            //Iterate through the images in the frame to find one that satisfies the width, height, and pixel format requirements
            long numImages = frame.getNumImages();
            for (int i = 0; i < numImages; i++){
                img = frame.getImage(i);
                if (img.getFormat() == PIXEL_FORMAT.RGB565 && img.getWidth() == width && img.getHeight() == height){
                    ByteBuffer byteBuf = img.getPixels();
                    byteBuf.get(dst);
                    return true;
                }
            }
            return false;
        }
        finally{
            if (frame != null) frame.close();
            if (tempFrame != null) tempFrame.close();
        }
    }

    //Turn camera flashlight on or off
    public static boolean setFlashTorchMode(boolean on){
        return CameraDevice.getInstance().setFlashTorchMode(on);
    }



//
//    public static int[] getRGBfromByteArray(int row, int col, int width, byte[] src){
//        int index = 2 * (row * width + col);
//        byte b1 = src[index];
//        byte b2 = src[index+1];
//        int blue = (b1 & 0x1F) << 3;
//        int red = (b2 & 0xF8);
//        int green = ((b1 & 0xE0) >> 3) + ((b2 & 0x7) << 5);
//        return new int[] {red, green, blue};
//    }
//
//
//    public static Bitmap getBitmap(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue)
//            throws InterruptedException{
//        VuforiaLocalizer.CloseableFrame frame = null;
//        try{
//            frame = frameQueue.poll(10, TimeUnit.MICROSECONDS);
//            if (frame == null) return null;
//            long numImages = frame.getNumImages();
//            Image rgbImage = null;
//            for (int i = 0; i < numImages; i++)
//                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
//                    rgbImage = frame.getImage(i);
//                    break;
//                }
//            if (rgbImage == null) return null;
//            Bitmap bm = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
//            bm.copyPixelsFromBuffer(rgbImage.getPixels());
//            return bm;
//        }
//        catch(InterruptedException exc){
//            throw exc;
//        }
//        finally{
//            if (frame != null) frame.close();
//        }
//    }
//

}
