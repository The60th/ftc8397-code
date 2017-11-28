package org.firstinspires.ftc.teamcode.mechbot;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.cv_programs.Blob;
import org.firstinspires.ftc.teamcode.cv_programs.ImgProc;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;

/**
 * Created by FTC Team 8397 on 11/10/2017.
 */

public abstract class MechBotAutonomous extends LoggingLinearOpMode {

    private boolean goForRankingPoints = false; // This boolean will control if our software tries to go for ranking points over earning us points.
    //This will mostly control the knocking off the jewel, if this is true the program will knock off the enemy jewel, and give them a 30 point lead.
    public void enableThrowing(){this.goForRankingPoints = true;}
    public void disableThrowing(){this.goForRankingPoints = false;}


    public MechBotSensor bot = new MechBotSensor();
    public enum LineFollowSide {LEFT,RIGHT}
    private final float INNER_TAPE_ANGLE = 33.70f;
    private final float INNER_TAPE_ANGLE_RADS = INNER_TAPE_ANGLE * ((float)Math.PI/180.0f);
    private final float LINE_FOLLOW_SPEED = 10.0f; //10 centimeters per second.
    private final float LINE_FOLLOW_ANGLE_FACTOR = 30.0f * ((float)Math.PI/180.0f); //30.0 Degrees converted to radians.
    private final float HEADING_CORECTION_FACTOR = 2.0f;
    public enum JewelSide {BLUE_LEFT,RED_LEFT,UNKNOWN}
    public enum TeamColor {BLUE,RED}
    public enum Side{LEFT,RIGHT,UNKNOWN}
    protected float[] robotZXPhi = null;
    public double colorSatCutOff = 0.5f; //Min value to be considered on a color.

    protected final String DRIVE_DIRECTION_GYRO_TAG = "DRIVE_DIRECTION_GYRO";
    protected final boolean DRIVE_DIRECTION_GYRO_LOG = true;

    protected final String TURN_ANGLE_TAG = "TURN_ANGLE_TAG";
    protected final boolean TURN_ANGLE_LOG = true;

    protected final String TURN_TO_HEADING_TAG = "TURN_TO_HEADING";
    protected final boolean TURN_TO_HEADING_LOG = true;

    protected final String DRIVE_GYRO_TIME_TAG = "DRIVE_GYRO_TIME";
    protected final boolean DRIVE_GYRO_TIME_LOG = true;

    protected final String FOLLOW_LINE_PROP_TAG = "FOLLOW_LINE_PROP";
    protected final boolean FOLLOW_LINE_PROP_LOG = true;

    protected void setFlashOn(){
        CameraDevice.getInstance().setFlashTorchMode(true);
    }
    protected void setFlashOff(){
        CameraDevice.getInstance().setFlashTorchMode(false);
    }

    public Side targetSide;
    public RelicRecoveryVuMark cryptoKey;

    private void followLineProportionateOLD(LineFollowSide side, ColorSensor colorSensor){
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

    public void followLineProportionate(LineFollowSide side, ColorSensor colorSensor, Predicate finish){
        float[] hsvValues = new float[3];
        final float coeff = 20.0f;
        while (opModeIsActive()){
            if(finish.isTrue()) break;
            float heading = bot.getHeadingRadians();
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            float err = side == LineFollowSide.LEFT? 0.5f - hsvValues[1] : hsvValues[1] - 0.5f;
            if (err < -0.5) err = -0.4f;
            else if (err > 0.5) err = 0.4f;
            if(FOLLOW_LINE_PROP_LOG)BetaLog.dd(FOLLOW_LINE_PROP_TAG,"Heading %.2f Sat %.2f error %.2f",heading,hsvValues[1],err);
            float angleDiff = side == LineFollowSide.LEFT? heading - INNER_TAPE_ANGLE_RADS : heading + INNER_TAPE_ANGLE_RADS;
            float vx = -LINE_FOLLOW_SPEED * (float)Math.cos(angleDiff)  + coeff*err*(float)Math.sin(angleDiff);
            float vy = LINE_FOLLOW_SPEED * (float)Math.sin(angleDiff) + coeff*err*(float)Math.cos(angleDiff);
            float va = -heading * HEADING_CORECTION_FACTOR;
            if(FOLLOW_LINE_PROP_LOG)BetaLog.dd(FOLLOW_LINE_PROP_TAG,"Angle Diff %.2f Vx %.2f Vy %.2f Va %.2f",angleDiff,vx,vy,va);
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDrivePower(0,0,0);
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
        float offset = (float) VuMarkNavigator.NormalizeAngle(targetHeading - heading);

        while (opModeIsActive() && Math.abs(offset) > tolerance) {
            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;
            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            if(TURN_ANGLE_LOG) BetaLog.dd(TURN_ANGLE_TAG,"Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
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
        final float vaMax = 0.2f * (float)Math.PI;
        float heading = bot.getHeadingRadians();
        float offset = (float)VuMarkNavigator.NormalizeAngle(targetHeading - heading);

        while (opModeIsActive() && Math.abs(offset) > tolerance) {
            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;
            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            if(TURN_TO_HEADING_LOG)BetaLog.dd(TURN_TO_HEADING_TAG,"Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            bot.setDriveSpeed(0, 0, va);
            heading = bot.getHeadingRadians();
            offset = targetHeading - heading;
        }
        bot.setDrivePower(0, 0, 0);

    }

    protected void driveStraightGyroTime(float vx, float vy, float duration) {
        try {
            BetaLog.initialize();
            final float C_ANGLE = 2.0f;
            final float initialHeading = bot.getHeadingRadians();
            ElapsedTime et = new ElapsedTime();
            double scale = bot.setDriveSpeed(vx, vy, 0);
            if(DRIVE_GYRO_TIME_LOG)BetaLog.dd(DRIVE_GYRO_TIME_TAG,"<Debug> DriveStrGyroTime Pre Scale duration = %.0f vx = %.0f vy = %.0f", duration, vx, vy);

            duration /= scale;
            vx *= scale;
            vy *= scale;
            if(DRIVE_GYRO_TIME_LOG)BetaLog.dd(DRIVE_GYRO_TIME_TAG,"<Debug> DriveStrGyroTime Post Scale( %.2f ) duration = %.0f vx = %.0f vy = %.0f", scale, duration, vx, vy);

            while (opModeIsActive()) {
                double etms = et.milliseconds();
                if (etms > duration) break;
                float currentHeading = bot.getHeadingRadians();
                float va = (initialHeading - currentHeading) * C_ANGLE;
                if(DRIVE_GYRO_TIME_LOG)BetaLog.dd(DRIVE_GYRO_TIME_TAG,"<Debug> Initial Angle = %.1f Current Angle = %.1f", initialHeading, currentHeading);
                bot.setDriveSpeed(vx, vy, va);
            }
            bot.setDriveSpeed(0, 0, 0);
        }
        finally {
            {
                BetaLog.close();
            }
        }
    }
    //VuMark Scan timeout in ms.
    public RelicRecoveryVuMark findKey(double timeOut){
        RelicRecoveryVuMark vuMarkKey = RelicRecoveryVuMark.UNKNOWN;
        ElapsedTime et = new ElapsedTime();
        while(opModeIsActive() && vuMarkKey == RelicRecoveryVuMark.UNKNOWN && et.milliseconds() < timeOut){
            vuMarkKey = VuMarkNavigator.getRelicRecoveryVumark();
        }
        return vuMarkKey;
    }

    //Time out, time in ms.
    public JewelSide findJewel(double timeOut){
        JewelSide returnSide;
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

                if(blueBlob.getAvgX() < redBlob.getAvgX()) return JewelSide.BLUE_LEFT;
                else return  JewelSide.RED_LEFT;

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
        return JewelSide.UNKNOWN;
    }

    public void driveDirectionGyro(float speedCMs, float directionAngleDegrees, Predicate finish){
            if (DRIVE_DIRECTION_GYRO_LOG) BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "Entering driveDirectionGyro");
            bot.updateOdometry();
            float directionAngleRadians = directionAngleDegrees * (float) Math.PI / 180.0f;
            while (opModeIsActive()) {
                float gyroHeading = bot.getHeadingRadians();
                float odomHeading = bot.getOdomHeadingFromGyroHeading(gyroHeading);

                if (DRIVE_DIRECTION_GYRO_LOG)
                    BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "gHeading = %.2f  oHeading = %.2f",
                            gyroHeading * 180.0 / Math.PI , odomHeading * 180.0 / Math.PI); //Fixed convert error

                this.robotZXPhi = bot.updateOdometry(robotZXPhi, odomHeading);

                if (DRIVE_DIRECTION_GYRO_LOG)
                    BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "z = %.2f  x = %.2f  Phi = %.2f",
                            robotZXPhi[0], robotZXPhi[1], robotZXPhi[2] * 180.0 / Math.PI); //Fixed convert error

                if (finish.isTrue()) break;

                float vx = -speedCMs * (float) Math.sin(directionAngleRadians - odomHeading);
                float vy = speedCMs * (float) Math.cos(directionAngleRadians - odomHeading);
                float va = -HEADING_CORECTION_FACTOR * gyroHeading;

                if (DRIVE_DIRECTION_GYRO_LOG)
                    BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "vx = %.2f  vy = %.2f  va = %.2f", vx, vy, va * 180.0 / Math.PI);

                bot.setDriveSpeed(vx, vy, va);

            }
            bot.setDriveSpeed(0,0,0);
    }
    //Robot heading in degrees.
    public void driveDirectionGyro(float speedCMs, float directionAngleDegrees, float gyroHeadingTargetDegrees, Predicate finish){

        if (DRIVE_DIRECTION_GYRO_LOG) BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "Entering driveDirectionGyro");
        bot.updateOdometry();
        float directionAngleRadians = directionAngleDegrees * (float) Math.PI / 180.0f;
        float gyroHeadingTargetRadians = gyroHeadingTargetDegrees * (float) Math.PI / 180.0f;

        while (opModeIsActive()) {
            float gyroHeading = bot.getHeadingRadians();
            float odomHeading = bot.getOdomHeadingFromGyroHeading(gyroHeading);

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "gHeading = %.2f  oHeading = %.2f",
                        gyroHeading * 180.0 / Math.PI , odomHeading * 180.0 / Math.PI); //Fixed convert error

            this.robotZXPhi = bot.updateOdometry(robotZXPhi, odomHeading);

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "z = %.2f  x = %.2f  Phi = %.2f",
                        robotZXPhi[0], robotZXPhi[1], robotZXPhi[2] * 180.0 / Math.PI); //Fixed convert error

            if (finish.isTrue()) break;

            float vx = -speedCMs * (float) Math.sin(directionAngleRadians - odomHeading);
            float vy = speedCMs * (float) Math.cos(directionAngleRadians - odomHeading);

            float headingError = (float)VuMarkNavigator.NormalizeAngle(gyroHeading - gyroHeadingTargetRadians);
            float va = -HEADING_CORECTION_FACTOR * headingError;

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "vx = %.2f  vy = %.2f  va = %.2f", vx, vy, va * 180.0 / Math.PI);

            bot.setDriveSpeed(vx, vy, va);

        }
        bot.setDrivePower(0,0,0);
    }
    protected interface Predicate{
        public boolean isTrue();
    }


    public void initAuto(TeamColor teamColor, float cryptoKeyTimeOut, double jewelTimeOut) throws InterruptedException{
        RelicRecoveryVuMark vuMark;
        JewelSide jewelSide;
        ElapsedTime et = new ElapsedTime();

        VuMarkNavigator.activate();
        while (opModeIsActive() && !VuMarkNavigator.isActive){
            sleep(1);
        }

        sleep(1000);

        double vuforiaActivateTime = et.milliseconds();
        telemetry.addData("Started Vuforia after " + vuforiaActivateTime + " milliseconds.","");
        telemetry.update();
        this.setFlashOn();
        waitForStart();

        vuMark = findKey(cryptoKeyTimeOut);
        double vuMarkFindTime = et.milliseconds() - vuforiaActivateTime;
        telemetry.addData("Found vuMark value of " + vuMark.toString() + " after " + vuMarkFindTime + " milliseconds.","");
        this.cryptoKey = vuMark;

        jewelSide = findJewel(jewelTimeOut);
        double jewlFindTime = et.milliseconds() - vuMarkFindTime;
        telemetry.addData("Found JewelSide value of " + jewelSide.toString() + " after " + jewlFindTime + " milliseconds.","");
        if(jewelSide == JewelSide.BLUE_LEFT && teamColor == TeamColor.BLUE){
            this.targetSide = Side.RIGHT;
        }else if(jewelSide == JewelSide.BLUE_LEFT && teamColor == TeamColor.RED){
            this.targetSide = Side.LEFT;
        }else if(jewelSide == JewelSide.RED_LEFT && teamColor == TeamColor.RED){
            this.targetSide = Side.RIGHT;
        }else if(jewelSide == JewelSide.RED_LEFT && teamColor == TeamColor.BLUE){
            this.targetSide = Side.LEFT;
        }else{
            this.targetSide = Side.UNKNOWN;
        }
        telemetry.addData("Found both the jewel and vuMark in: " + et.milliseconds() + " milliseconds. ","");
        telemetry.update();
        this.setFlashOff();
    }

    public boolean knockJewel(Side side){
        if(goForRankingPoints){
            //This logic flow, will score the jewel for the ENEMY team.
            return true;
        }else{
            if(side == Side.UNKNOWN){
                return false;
            }
            else if(side == Side.LEFT){
                turnToHeadingGyro(-1,-1,-1);
                //spin the robot left 45 degrees then return to facing forwards.
            }else if(side == Side.RIGHT){
                turnToHeadingGyro(-1,-1,-1);
                //spin the robot right 45 degrees then return to facing forwards.
            }
            return true;
        }
    }
}
