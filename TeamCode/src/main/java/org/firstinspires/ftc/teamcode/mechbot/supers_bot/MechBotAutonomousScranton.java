package org.firstinspires.ftc.teamcode.mechbot.supers_bot;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.cv_programs.Blob;
import org.firstinspires.ftc.teamcode.cv_programs.ImgProc;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;

/**
 * Created by FTC Team 8397 on 3/1/2018.
 */

public abstract class MechBotAutonomousScranton extends LoggingLinearOpMode {

    private boolean goForRankingPoints = false; // This boolean will control if our software tries to go for ranking points over earning us points.

    //This will mostly control the knocking off the jewel, if this is true the program will knock off the enemy jewel, and give them a 30 point lead.
    public void enableThrowing() {
        this.goForRankingPoints = true;
    }

    public void disableThrowing() {
        this.goForRankingPoints = false;
    }

    public MechBotScranton bot = new MechBotScranton();

    public enum LineFollowSide {LEFT, RIGHT}

    public final float OFF_STONE_SPEED = 25.0f;
    protected final float INNER_TAPE_ANGLE = 33.70f;
    protected final float INNER_TAPE_ANGLE_RADS = INNER_TAPE_ANGLE * ((float) Math.PI / 180.0f);
    public final float LINE_FOLLOW_SPEED = 15.0f; //10 centimeters per second.
    private final float LINE_FOLLOW_ANGLE_FACTOR = 30.0f * ((float) Math.PI / 180.0f); //30.0 Degrees converted to radians.
    protected final float HEADING_CORECTION_FACTOR = 2.0f;
    public final float DRIVE_TOWARDS_TRIANGLE_SPEED = 20.0f;

    public enum JewelSide {BLUE_LEFT, RED_LEFT, UNKNOWN}

    public enum TeamColor {BLUE, RED}

    public enum Side {LEFT, RIGHT, UNKNOWN}

    protected float[] robotZXPhi = null;

    protected final String DRIVE_DIRECTION_GYRO_TAG = "DRIVE_DIRECTION_GYRO";
    protected final boolean DRIVE_DIRECTION_GYRO_LOG = false;

    protected final String TURN_ANGLE_TAG = "TURN_ANGLE_TAG";
    protected final boolean TURN_ANGLE_LOG = false;

    protected final String TURN_TO_HEADING_TAG = "TURN_TO_HEADING";
    protected final boolean TURN_TO_HEADING_LOG = false;

    protected final String DRIVE_GYRO_TIME_TAG = "DRIVE_GYRO_TIME";
    protected final boolean DRIVE_GYRO_TIME_LOG = false;

    protected final String FOLLOW_LINE_PROP_TAG = "FOLLOW_LINE_PROP";
    protected final boolean FOLLOW_LINE_PROP_LOG = false;

    protected final String ADJUST_POS_TAG = "ADJUST_POSITION";
    protected final boolean ADJUST_POS_LOG = false;

    final String PREPARE_SCORE_TAG = "PREP_SCORE";
    final boolean PREPARE_SCORE_LOG = false;

    final String SCORE_GLYPH_TAG = "SCORE_GLYPH";
    final boolean SCORE_GLYPH_LOG = true;

    final String AUTO_POS_TAG = "AUTO_POS_TAG";
    final boolean AUTO_POS_DEBUG = false;

    protected final String TUCK_GLYPH_TAG = "TUCK_GLYPH";
    protected final boolean TUCK_GLYPH_LOG = true;

    protected void setFlashOn() {
        CameraDevice.getInstance().setFlashTorchMode(true);
    }

    protected void setFlashOff() {
        CameraDevice.getInstance().setFlashTorchMode(false);
    }

    public Side targetSide = Side.UNKNOWN;
    public RelicRecoveryVuMark cryptoKey = RelicRecoveryVuMark.UNKNOWN;
    public TeamColor teamColor;

    public float initRoll; //orientation.secondAngle + ROLL_ADJUSTMENT_DEGREES * (float)Math.PI/180.0f;
    public float initPitch;
    private final float ADJUST_INIT_ROLL_DEG = 1.5f;

    public final float GLOBAL_STANDERD_TOLERANCE = 2f; //Degrees
    public final float GLOBAL_STANDERD_LATENCY = 0.3f; //Seconds
    public final float HSV_SAT_CUT_OFF = .5f;
    public final float HSV_SAT_CUT_OFF_STONE = .40f;

    public final float JEWEL_SCAN_TIME = 500;
    public final float VUMARK_KEY_SCAN_TIME = 500;

    public final float CRYPTO_BOX_SIDE_SHIFT_VALUE = 17.5f; //Was 18.6, but was shifting too far to left and right

    public final float CRYPTO_BOX_CENTER_SHIFT_VALUE = -5.0f; //was 1.0f on 1/11/18 bottom left over shot some changing to test.

    public final float CRYPTO_BOX_FORWARD_SHIFT_VALUE = -8;

    public final float ADJUST_POS_TIMEOUT = 4000;

    public enum RotationDirection {COUNTER_CLOCK, CLOCK}

    public ElapsedTime runTime;



    //NEW followLineProportionate: this uses MechBot.getOdomHeadingFromGyroHeading(). It will work properly for different hardware
    //configurations, as long as we override getOdomHeadingFromGyroHeading in MechBotSensor subclasses, as necessary.
    public void followLineProportionate(LineFollowSide side, ColorSensor colorSensor, float lineFollowSpeed, Predicate finish) {
        float[] hsvValues = new float[3];
        final float coeff = 20.0f;
        while (opModeIsActive()) {
            if (finish.isTrue()) break;
            float heading = bot.getHeadingRadians();
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            float err = side == LineFollowSide.LEFT ? 0.5f - hsvValues[1] : hsvValues[1] - 0.5f;
            if (err < -0.5) err = -0.4f;
            else if (err > 0.5) err = 0.4f;
            if (FOLLOW_LINE_PROP_LOG)
                BetaLog.dd(FOLLOW_LINE_PROP_TAG, "Heading %.2f Sat %.2f error %.2f", heading, hsvValues[1], err);

//            float angleDiff = side == LineFollowSide.LEFT? heading - INNER_TAPE_ANGLE_RADS : heading + INNER_TAPE_ANGLE_RADS;

            float odomHeading = bot.getOdomHeadingFromGyroHeading(heading);
            float angleDiff = side == LineFollowSide.LEFT ? odomHeading - (float) Math.PI - INNER_TAPE_ANGLE_RADS :
                    odomHeading - (float) Math.PI + INNER_TAPE_ANGLE_RADS;

//            float vx = lineFollowSpeed * (float)Math.cos(angleDiff)  - coeff*err*(float)Math.sin(angleDiff);
//            float vy = -lineFollowSpeed * (float)Math.sin(angleDiff) - coeff*err*(float)Math.cos(angleDiff);

            float vx = lineFollowSpeed * (float) Math.sin(angleDiff) + coeff * err * (float) Math.cos(angleDiff);
            float vy = lineFollowSpeed * (float) Math.cos(angleDiff) - coeff * err * (float) Math.sin(angleDiff);


            float va = -heading * HEADING_CORECTION_FACTOR;
            if (FOLLOW_LINE_PROP_LOG)
                BetaLog.dd(FOLLOW_LINE_PROP_TAG, "Angle Diff %.2f Vx %.2f Vy %.2f Va %.2f", angleDiff, vx, vy, va);
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDrivePower(0, 0, 0);
    }

    public void followLineProportionate(LineFollowSide side, float tapeAngleRads, ColorSensor colorSensor, float lineFollowSpeed, Predicate finish) {
        float[] hsvValues = new float[3];
        final float coeff = 20.0f;
        while (opModeIsActive()) {
            if (finish.isTrue()) break;
            float heading = bot.getHeadingRadians();
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            float err = side == LineFollowSide.LEFT ? 0.5f - hsvValues[1] : hsvValues[1] - 0.5f;
            if (err < -0.5) err = -0.4f;
            else if (err > 0.5) err = 0.4f;
            if (FOLLOW_LINE_PROP_LOG)
                BetaLog.dd(FOLLOW_LINE_PROP_TAG, "Heading %.2f Sat %.2f error %.2f", heading, hsvValues[1], err);

            float odomHeading = bot.getOdomHeadingFromGyroHeading(heading);
            float angleDiff = odomHeading - (float) Math.PI - tapeAngleRads;

            float vx = lineFollowSpeed * (float) Math.sin(angleDiff) + coeff * err * (float) Math.cos(angleDiff);
            float vy = lineFollowSpeed * (float) Math.cos(angleDiff) - coeff * err * (float) Math.sin(angleDiff);


            float va = -heading * HEADING_CORECTION_FACTOR;
            if (FOLLOW_LINE_PROP_LOG)
                BetaLog.dd(FOLLOW_LINE_PROP_TAG, "Angle Diff %.2f Vx %.2f Vy %.2f Va %.2f", angleDiff, vx, vy, va);
            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDrivePower(0, 0, 0);
    }

    //Turns robot to a specific integratedZ heading using Gyro, targetHeading in degrees
    protected void turnToHeadingGyro(float targetHeading, float tolerance, float latency) {
        //Tolerance in degrees latency seconds.
        tolerance = tolerance * (float) Math.PI / 180f;
        targetHeading = targetHeading * (float) Math.PI / 180f;

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = 0.6f * (float) Math.PI;
        float heading;
        float offset;
        while (opModeIsActive()) {
            heading = bot.getHeadingRadians();
            float odomHeading = bot.getOdomHeadingFromGyroHeading(heading);
            this.robotZXPhi = bot.updateOdometry(robotZXPhi, odomHeading);
            offset = (float) VuMarkNavigator.NormalizeAngle(targetHeading - heading);
            if (Math.abs(offset) <= tolerance) break;

            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;

            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            if (TURN_TO_HEADING_LOG)
                BetaLog.dd(TURN_TO_HEADING_TAG, "Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
    }


    //Turns robot to a specific integratedZ heading using Gyro, targetHeading in degrees
    protected void turnToHeadingGyro(float targetHeading, float tolerance, float latency, RotationDirection rotationDirection) {
        //Tolerance in degrees latency seconds.
        tolerance = tolerance * (float) Math.PI / 180f;
        targetHeading = targetHeading * (float) Math.PI / 180f;

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.75f / latency;
        final float vaMax = 0.6f * (float) Math.PI; //Was .5 on 1/11/18
        float heading;
        float offset;
        while (opModeIsActive()) {
            heading = bot.getHeadingRadians();
            float odomHeading = bot.getOdomHeadingFromGyroHeading(heading);
            this.robotZXPhi = bot.updateOdometry(robotZXPhi, odomHeading);

            offset = (float) VuMarkNavigator.NormalizeAngle(targetHeading - heading);
            if (Math.abs(offset) <= tolerance) break;

            if (rotationDirection == RotationDirection.COUNTER_CLOCK) {
                if (offset < 0) offset += 2.0f * (float) Math.PI;
            } else {
                if (offset > 0) offset -= 2.0f * (float) Math.PI;
            }

            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;

            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            if (TURN_TO_HEADING_LOG)
                BetaLog.dd(TURN_TO_HEADING_TAG, "Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
    }

    //Turns robot to a specific integratedZ heading using Gyro, targetHeading in degrees
    protected void turnToHeadingGyroQuick(float targetHeading, float tolerance, float latency, RotationDirection rotationDirection) {
        //Tolerance in degrees latency seconds.
        tolerance = tolerance * (float) Math.PI / 180f;
        targetHeading = targetHeading * (float) Math.PI / 180f;

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.90f / latency;
        final float vaMax = 0.6f * (float) Math.PI; //Was .5 on 1/11/18
        float heading;
        float offset;
        while (opModeIsActive()) {
            heading = bot.getHeadingRadians();
            float odomHeading = bot.getOdomHeadingFromGyroHeading(heading);
            this.robotZXPhi = bot.updateOdometry(robotZXPhi, odomHeading);
            offset = (float) VuMarkNavigator.NormalizeAngle(targetHeading - heading);
            if (Math.abs(offset) <= tolerance) break;

            if (rotationDirection == RotationDirection.COUNTER_CLOCK) {
                if (offset < 0) offset += 2.0f * (float) Math.PI;
            } else {
                if (offset > 0) offset -= 2.0f * (float) Math.PI;
            }

            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;

            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            if (TURN_TO_HEADING_LOG)
                BetaLog.dd(TURN_TO_HEADING_TAG, "Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
    }

    //Turns robot to a specific integratedZ heading using Gyro, targetHeading in degrees
    protected void turnToHeadingGyroQuick(float targetHeading, float tolerance, float latency) {
        //Tolerance in degrees latency seconds.
        tolerance = tolerance * (float) Math.PI / 180f;
        targetHeading = targetHeading * (float) Math.PI / 180f;

        final float vaMin = 1.5f * tolerance / latency;
        final float C = 0.90f / latency;
        final float vaMax = 0.6f * (float) Math.PI;
        float heading;
        float offset;
        while (opModeIsActive()) {
            heading = bot.getHeadingRadians();
            float odomHeading = bot.getOdomHeadingFromGyroHeading(heading);
            this.robotZXPhi = bot.updateOdometry(robotZXPhi, odomHeading);
            offset = (float) VuMarkNavigator.NormalizeAngle(targetHeading - heading);
            if (Math.abs(offset) <= tolerance) break;

            float absAdjustedOffset = Math.abs(offset) - tolerance;
            float absVa = vaMin + C * absAdjustedOffset;

            absVa = Math.min(absVa, vaMax);
            float va = absVa * Math.signum(offset);
            if (TURN_TO_HEADING_LOG)
                BetaLog.dd(TURN_TO_HEADING_TAG, "Turning va = %.2f hd = %.0f, off = %.0f absAdjOff = %.0f", va, heading, offset, absAdjustedOffset);
            bot.setDriveSpeed(0, 0, va);
        }
        bot.setDrivePower(0, 0, 0);
    }

    //VuMark Scan timeout in ms.
    public RelicRecoveryVuMark findKey(double timeOut) {
        RelicRecoveryVuMark vuMarkKey = RelicRecoveryVuMark.UNKNOWN;
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive() && vuMarkKey == RelicRecoveryVuMark.UNKNOWN && et.milliseconds() < timeOut) {
            vuMarkKey = VuMarkNavigator.getRelicRecoveryVumark();
        }
        return vuMarkKey;
    }

    //Time out, time in ms.
    public JewelSide findJewel(double timeOut) {
        MechBotAutonomous.JewelSide returnSide;
        int blobSizeThreshhold = 400; //Blobs smaller than this will be discarded
        int sampleRatio = 5; //Number of rows and columns to skip between raw pixels selected for reduced image

        //Get resolution automatically from camera
        float[] size = CameraDevice.getInstance().getCameraCalibration().getSize().getData();
        int imgWidth = Math.round(size[0]);
        int imgHeight = Math.round(size[1]);
        byte[] imageBytes = new byte[2 * imgWidth * imgHeight];

        int y0 = 300;
        int croppedImgWidth = imgWidth;
        int croppedImgHeight = 420;

        //Set up reduced image dimensions
        int reducedImgWidth = croppedImgWidth / sampleRatio;
        int reducedImgHeight = croppedImgHeight / sampleRatio;
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
            if (!gotBytes) {
                continue;
            }

            //First, reduce the imgWidthximgHeight image to reducedImgWidth x reducedImgHeight by skipping rows and columns per sampleRatio
            //From the reduced RGB565 image, obtain the binary images for red and blue blob detection
            ImgProc.getReducedRangeRGB565(imageBytes, imgWidth, imgHeight, 0, y0, croppedImgWidth, croppedImgHeight, reducedImageBytes, sampleRatio);
            ImgProc.getBinaryImage(reducedImageBytes, 345, 15, 0.7f, 1.0f, 0.3f, 1.0f, binaryRed);
            ImgProc.getBinaryImage(reducedImageBytes, 195, 235, 0.7f, 1.0f, 0.2f, 1.0f, binaryBlue);

            //Get lists of red and blue blobs from the binary images
            redBlobs = Blob.findBlobs(binaryRed, reducedImgWidth, reducedImgHeight);
            blueBlobs = Blob.findBlobs(binaryBlue, reducedImgWidth, reducedImgHeight);

            //Filter out small blobs
            for (int i = redBlobs.size() - 1; i >= 0; i--)
                if (redBlobs.get(i).getNumPts() < blobSizeThreshhold) redBlobs.remove(i);
            for (int i = blueBlobs.size() - 1; i >= 0; i--)
                if (blueBlobs.get(i).getNumPts() < blobSizeThreshhold) blueBlobs.remove(i);

            //Take only the right-most red blob. This is to avoid problems with the VuMark, which may match the red range.
            while (redBlobs.size() > 1) {
                if (redBlobs.get(0).getAvgX() < redBlobs.get(1).getAvgX()) redBlobs.remove(0);
                else redBlobs.remove(1);
            }


            if (redBlobs.size() > 0 && blueBlobs.size() > 0) {
                Blob redBlob = redBlobs.get(0); //There is only one blob left in the red list; this is it.

                //Find the largest blue blob; it will be assumed to be the blue jewel.
                Blob blueBlob = blueBlobs.get(0);
                for (int i = 1; i < blueBlobs.size(); i++)
                    if (blueBlobs.get(i).getRectArea() > blueBlob.getRectArea())
                        blueBlob = blueBlobs.get(i);

                if (blueBlob.getAvgX() < redBlob.getAvgX()) return JewelSide.BLUE_LEFT;
                else return JewelSide.RED_LEFT;

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

    public void driveDirectionGyro(float speedCMs, float directionAngleDegrees, Predicate finish) {
        if (DRIVE_DIRECTION_GYRO_LOG)
            BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "Entering driveDirectionGyro");
        //bot.updateOdometry(); removed to fix odemtry resteing
        float directionAngleRadians = directionAngleDegrees * (float) Math.PI / 180.0f;
        while (opModeIsActive()) {
            float gyroHeading = bot.getHeadingRadians();
            float odomHeading = bot.getOdomHeadingFromGyroHeading(gyroHeading);

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "gHeading = %.2f  oHeading = %.2f",
                        gyroHeading * 180.0 / Math.PI, odomHeading * 180.0 / Math.PI); //Fixed convert error

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
        bot.setDriveSpeed(0, 0, 0);
    }

    //Robot heading in degrees.
    public void driveDirectionGyro(float speedCMs, float directionAngleDegrees, float gyroHeadingTargetDegrees, Predicate finish) {

        if (DRIVE_DIRECTION_GYRO_LOG)
            BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "Entering driveDirectionGyro");
       // bot.updateOdometry();
        float directionAngleRadians = directionAngleDegrees * (float) Math.PI / 180.0f;
        float gyroHeadingTargetRadians = gyroHeadingTargetDegrees * (float) Math.PI / 180.0f;

        while (opModeIsActive()) {
            float gyroHeading = bot.getHeadingRadians();
            float odomHeading = bot.getOdomHeadingFromGyroHeading(gyroHeading);

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "gHeading = %.2f  oHeading = %.2f",
                        gyroHeading * 180.0 / Math.PI, odomHeading * 180.0 / Math.PI); //Fixed convert error

            this.robotZXPhi = bot.updateOdometry(robotZXPhi, odomHeading);

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "z = %.2f  x = %.2f  Phi = %.2f",
                        robotZXPhi[0], robotZXPhi[1], robotZXPhi[2] * 180.0 / Math.PI); //Fixed convert error

            if (finish.isTrue()) break;

            float vx = -speedCMs * (float) Math.sin(directionAngleRadians - odomHeading);
            float vy = speedCMs * (float) Math.cos(directionAngleRadians - odomHeading);
            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "gHeading = %.3f gH Target = %.3f", gyroHeading, gyroHeadingTargetRadians);


            float headingError = (float) VuMarkNavigator.NormalizeAngle(gyroHeading - gyroHeadingTargetRadians);
            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "HeadingError = %.3f ", headingError);
            float va = -HEADING_CORECTION_FACTOR * headingError;

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "vx = %.2f  vy = %.2f  va = %.2f", vx, vy, va * 180.0 / Math.PI);

            bot.setDriveSpeed(vx, vy, va);

        }
        bot.setDrivePower(0, 0, 0);
    }

    protected interface Predicate {
        public boolean isTrue();
    }


    public void initAuto(TeamColor teamColor, float cryptoKeyTimeOut, double jewelTimeOut) throws InterruptedException {
        Orientation orientation;
        this.teamColor = teamColor;
        RelicRecoveryVuMark vuMark;
        JewelSide jewelSide;
        ElapsedTime et = new ElapsedTime();
        telemetry.addData("Starting Vuforia", "");
        telemetry.addData("Wait for flashlight to be on before starting.", "");
        telemetry.update();
        VuMarkNavigator.activate();
        while (opModeIsActive() && !VuMarkNavigator.isActive) {
            sleep(1);
        }
        double vuforiaActivateTime = et.milliseconds();
        et.reset();
        telemetry.addData("Started Vuforia after " + vuforiaActivateTime + " milliseconds.", "");
        telemetry.update();
        this.setFlashOn();

        orientation = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS); //WAS ZYX

        this.initPitch = orientation.thirdAngle;
        this.initRoll = orientation.secondAngle + ADJUST_INIT_ROLL_DEG * (float) Math.PI / 180.0f;

        waitForStart();
        bot.setRelicArmStop();
        runTime = new ElapsedTime();

        et.reset();
        vuMark = findKey(cryptoKeyTimeOut);
        double vuMarkFindTime = et.milliseconds();
        et.reset();
        telemetry.addData("Found vuMark value of " + vuMark.toString() + " after " + vuMarkFindTime + " milliseconds.", "");
        this.cryptoKey = vuMark;
        et.reset();
        jewelSide = findJewel(jewelTimeOut);
        double jewlFindTime = et.milliseconds();
        telemetry.addData("Found JewelSide value of " + jewelSide.toString() + " after " + jewlFindTime + " milliseconds.", "");
        if (jewelSide == JewelSide.BLUE_LEFT && teamColor == TeamColor.BLUE) {
            this.targetSide = Side.RIGHT;
        } else if (jewelSide == JewelSide.BLUE_LEFT && teamColor == TeamColor.RED) {
            this.targetSide = Side.LEFT;
        } else if (jewelSide == JewelSide.RED_LEFT && teamColor == TeamColor.RED) {
            this.targetSide = Side.RIGHT;
        } else if (jewelSide == JewelSide.RED_LEFT && teamColor == TeamColor.BLUE) {
            this.targetSide = Side.LEFT;
        } else {
            this.targetSide = Side.UNKNOWN;
        }
        telemetry.addData("Found both the jewel and vuMark in: " + vuMarkFindTime + jewlFindTime + " milliseconds. ", "");
        this.setFlashOff();

        knockJewel(this.targetSide); //Score the blocks and knock the jewel.
    }

    //NEW adjustPosOnTriangle: uses getOdomHeadingFromGyroHeading. It will work as long as we override that method
    //appropriately in whatever subclass of MechBotSensor we are using. We may also want to modify this to accept arguments
    //for interSensorDist and sensorOffset, so it can be used with different sensor configurations.

    public void adjustPosOnTriangle(double timeOut) {
        if (ADJUST_POS_LOG) BetaLog.dd(ADJUST_POS_TAG, "Entering Adjust Pos on Triangle.");
        final double interSensorDist = 34;
        final double sensorOffSet = 10;
        final double sensorR = Math.sqrt(interSensorDist * interSensorDist / 4.0 + sensorOffSet * sensorOffSet);
        final double sensorAngle = Math.atan(2.0 * sensorOffSet / interSensorDist);
        final float specialCoeff = (float) (sensorR * Math.sin(sensorAngle + INNER_TAPE_ANGLE_RADS) / Math.cos(INNER_TAPE_ANGLE_RADS));
        final float CAngle = 2.0f;
        final float CSum = 10.0f;
        final float CDiff = 10.0f;
        final float tolAngle = 2.0f * (float) Math.PI / 180f;
        final float tolSum = .2f;
        final float tolDiff = .2f;

        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive() && time.milliseconds() < timeOut) {
            float gyroHeading = bot.getHeadingRadians();
            float[] hsvRight = new float[3];
            float[] hsvLeft = new float[3];
            Color.RGBToHSV(bot.colorRight.red(), bot.colorRight.green(), bot.colorRight.blue(), hsvRight);
            Color.RGBToHSV(bot.colorLeft.red(), bot.colorLeft.green(), bot.colorLeft.blue(), hsvLeft);
            float sumSat = hsvRight[1] + hsvLeft[1];
            float diffSat = hsvRight[1] - hsvLeft[1];

            if (Math.abs(gyroHeading) < tolAngle && Math.abs(sumSat - 1.0f) < tolSum && Math.abs(diffSat) < tolDiff) {
                if (ADJUST_POS_LOG) BetaLog.dd(ADJUST_POS_TAG, "Adjust Pos Succeeded!!!");
                break;
            }

            float va = -CAngle * gyroHeading;


//            float vx = -CSum*(sumSat-1.0f)*(float)Math.cos(gyroHeading)
//                    - (CDiff*diffSat + specialCoeff*va)*(float)Math.sin(gyroHeading);
//
//            float vy = +CSum*(sumSat-1.0f)*(float)Math.sin(gyroHeading)
//                    - (CDiff*diffSat + specialCoeff*va)*(float)Math.cos(gyroHeading);

            //Desired robot speeds in field coordinates

            float vxField = CDiff * diffSat + specialCoeff * va;
            float vzField = CSum * (sumSat - 1.0f);

            //Desired robot speeds in robot coordinates

            float odomHeading = bot.getOdomHeadingFromGyroHeading(gyroHeading);

            float vx = -vxField * (float) Math.cos(odomHeading) + vzField * (float) Math.sin(odomHeading);
            float vy = vxField * (float) Math.sin(odomHeading) + vzField * (float) Math.cos(odomHeading);

            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0, 0);

    }


    public void adjustPosInsideTriangle(double timeOut) {
        if (ADJUST_POS_LOG) BetaLog.dd(ADJUST_POS_TAG, "Entering Adjust Pos on Triangle.");
        final double interSensorDist = 34;
        final double sensorOffSet = 10;
        final double sensorR = Math.sqrt(interSensorDist * interSensorDist / 4.0 + sensorOffSet * sensorOffSet);
        final double sensorAngle = Math.atan(2.0 * sensorOffSet / interSensorDist);
        final float specialCoeff = (float) (sensorR * Math.sin(sensorAngle + INNER_TAPE_ANGLE_RADS) / Math.cos(INNER_TAPE_ANGLE_RADS));
        final float CAngle = 2.0f;
        final float CSum = -10.0f;
        final float CDiff = -10.0f;
        final float tolAngle = 2.0f * (float) Math.PI / 180f;
        final float tolSum = .2f;
        final float tolDiff = .2f;

        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive() && time.milliseconds() < timeOut) {
            float gyroHeading = bot.getHeadingRadians();
            float[] hsvRight = new float[3];
            float[] hsvLeft = new float[3];
            Color.RGBToHSV(bot.colorRight.red(), bot.colorRight.green(), bot.colorRight.blue(), hsvRight);
            Color.RGBToHSV(bot.colorLeft.red(), bot.colorLeft.green(), bot.colorLeft.blue(), hsvLeft);
            float sumSat = hsvRight[1] + hsvLeft[1];
            float diffSat = hsvRight[1] - hsvLeft[1];

            if (Math.abs(gyroHeading) < tolAngle && Math.abs(sumSat - 1.0f) < tolSum && Math.abs(diffSat) < tolDiff) {
                if (ADJUST_POS_LOG) BetaLog.dd(ADJUST_POS_TAG, "Adjust Pos Succeeded!!!");
                break;
            }

            float va = -CAngle * gyroHeading;


//            float vx = -CSum*(sumSat-1.0f)*(float)Math.cos(gyroHeading)
//                    - (CDiff*diffSat + specialCoeff*va)*(float)Math.sin(gyroHeading);
//
//            float vy = +CSum*(sumSat-1.0f)*(float)Math.sin(gyroHeading)
//                    - (CDiff*diffSat + specialCoeff*va)*(float)Math.cos(gyroHeading);

            //Desired robot speeds in field coordinates

            float vxField = CDiff * diffSat + specialCoeff * va;
            float vzField = CSum * (sumSat - 1.0f);

            //Desired robot speeds in robot coordinates

            float odomHeading = bot.getOdomHeadingFromGyroHeading(gyroHeading);

            float vx = -vxField * (float) Math.cos(odomHeading) + vzField * (float) Math.sin(odomHeading);
            float vy = vxField * (float) Math.sin(odomHeading) + vzField * (float) Math.cos(odomHeading);

            bot.setDriveSpeed(vx, vy, va);
        }
        bot.setDriveSpeed(0, 0, 0);

    }


    public void scoreGlyph() {
        setOdometry(0,0);
//        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
//        bot.updateOdometry();
        switch (this.cryptoKey) {
            case LEFT:
                if (PREPARE_SCORE_LOG) BetaLog.dd(PREPARE_SCORE_TAG, "driveDirectionGyro left");
                driveDirectionGyro(20, -90, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return robotZXPhi[1] < -CRYPTO_BOX_SIDE_SHIFT_VALUE + CRYPTO_BOX_CENTER_SHIFT_VALUE ;
                    }
                });
                break;
            case RIGHT:
                if (PREPARE_SCORE_LOG) BetaLog.dd(PREPARE_SCORE_TAG, "driveDirectionGyro right");
                driveDirectionGyro(20, 90, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return robotZXPhi[1] > CRYPTO_BOX_SIDE_SHIFT_VALUE + CRYPTO_BOX_CENTER_SHIFT_VALUE;
                    }
                });
                break;
            case CENTER:
            case UNKNOWN:
                if (PREPARE_SCORE_LOG) BetaLog.dd(PREPARE_SCORE_TAG, "driveDirectionGyro right");
                if (CRYPTO_BOX_CENTER_SHIFT_VALUE > 0) {
                    driveDirectionGyro(20, 90, new Predicate() {
                        @Override
                        public boolean isTrue() {
                            return robotZXPhi[1] > CRYPTO_BOX_CENTER_SHIFT_VALUE;
                        }
                    });
                } else {
                    driveDirectionGyro(20, -90, new Predicate() {
                        @Override
                        public boolean isTrue() {
                            return robotZXPhi[1] < CRYPTO_BOX_CENTER_SHIFT_VALUE;
                        }
                    });
                }
                break;
        }

        bot.setFlipPlateUpwards();
        sleep(2000);

        if (PREPARE_SCORE_LOG) BetaLog.dd(PREPARE_SCORE_TAG, "driveDirectionGyro 3");

        setOdometry(0, 0);
//        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
//        bot.updateOdometry();

        driveDirectionGyro(20, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < CRYPTO_BOX_FORWARD_SHIFT_VALUE;
            }
        });

        telemetry.addData("Auto data: ", "Vumark target: " + cryptoKey + " target jewel side: " + targetSide);
        telemetry.update();

        bot.rightIntake.setPower(-1);
        bot.leftIntake.setPower(-1);
        sleep(1000);
        setOdometry(0, 0);

//        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
//        bot.updateOdometry();

        driveDirectionGyro(30, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 20;
            }
        });
        setOdometry(0, 0);

//        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};

        driveDirectionGyro(30, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < -16;
            }
        });


        bot.rightIntake.setPower(0);
        bot.leftIntake.setPower(0);
        setOdometry(0, 0);

//        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};

        driveDirectionGyro(30, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 12;
            }
        });

    }

    public void prepareToScoreGlyphQuick() {
        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        switch (this.cryptoKey) {
            case LEFT:
                if (PREPARE_SCORE_LOG) BetaLog.dd(PREPARE_SCORE_TAG, "driveDirectionGyro left");
                driveDirectionGyro(25, -90, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return robotZXPhi[1] < -CRYPTO_BOX_SIDE_SHIFT_VALUE + CRYPTO_BOX_CENTER_SHIFT_VALUE;
                    }
                });
                break;
            case RIGHT:
                if (PREPARE_SCORE_LOG) BetaLog.dd(PREPARE_SCORE_TAG, "driveDirectionGyro right");
                driveDirectionGyro(25, 90, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return robotZXPhi[1] > CRYPTO_BOX_SIDE_SHIFT_VALUE + CRYPTO_BOX_CENTER_SHIFT_VALUE;
                    }
                });
                break;
            case CENTER:
            case UNKNOWN:
                if (PREPARE_SCORE_LOG) BetaLog.dd(PREPARE_SCORE_TAG, "driveDirectionGyro right");
                if (CRYPTO_BOX_CENTER_SHIFT_VALUE > 0) {
                    driveDirectionGyro(25, 90, new Predicate() {
                        @Override
                        public boolean isTrue() {
                            return robotZXPhi[1] > CRYPTO_BOX_CENTER_SHIFT_VALUE;
                        }
                    });
                } else {
                    driveDirectionGyro(25, -90, new Predicate() {
                        @Override
                        public boolean isTrue() {
                            return robotZXPhi[1] < CRYPTO_BOX_CENTER_SHIFT_VALUE;
                        }
                    });
                }
                break;
        }

        bot.setFlipPlateUpwards();
        sleep(500);

        if (PREPARE_SCORE_LOG) BetaLog.dd(PREPARE_SCORE_TAG, "driveDirectionGyro 3");
        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();

        driveDirectionGyro(25, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < CRYPTO_BOX_FORWARD_SHIFT_VALUE;
            }
        });

        telemetry.addData("Auto data: ", "Vumark target: " + cryptoKey + " target jewel side: " + targetSide);
        telemetry.update();

        bot.rightIntake.setPower(-1);
        bot.leftIntake.setPower(-1);
        sleep(500);

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();

        driveDirectionGyro(25, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 6;
            }
        });

        bot.rightIntake.setPower(0);
        bot.leftIntake.setPower(0);
    }

    public boolean knockJewel(Side side) {
        bot.setPivotStart();
        bot.setArmCube();
        sleep(250);

        bot.setPivotEnd();
        sleep(500);
        bot.setArmJewel();
        sleep(500);

        if (goForRankingPoints) {
            //This logic flow, will score the jewel for the ENEMY team.
            return true;
        } else {
            if (side == Side.UNKNOWN) {
                return false;
            } else if (side == Side.LEFT) {
                bot.knockPivotLeft();
                sleep(400);
            } else if (side == Side.RIGHT) {
                bot.knockPivotRight();
                sleep(400);
            }

            bot.setArmCube();
            sleep(250);
            bot.setPivotStart();
            return true;
        }
    }

    boolean checkingForStall = false;
    boolean unStalling = false;

    public void multiGlyph() {
        final boolean[] inTakeOn = new boolean[]{false};
        final boolean[] inGlyphPit = new boolean[]{false};
        final boolean[] glyphsInRobot = new boolean[]{false, false};
        bot.setGlyphPincherMidPos();
        bot.setFlipPlateDownwards();
        quickTelemetry("Starting reverse");
        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        driveDirectionGyro(25, 0, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 15;
            }
        });
        quickTelemetry("Finished reverse, starting turn");

        //Reverse a tad
        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        if (this.cryptoKey == RelicRecoveryVuMark.LEFT) {
            turnToHeadingGyroQuick(180, GLOBAL_STANDERD_TOLERANCE, GLOBAL_STANDERD_LATENCY, RotationDirection.COUNTER_CLOCK); //Started in front of the left goal.
        } else {
            turnToHeadingGyroQuick(180, GLOBAL_STANDERD_TOLERANCE, GLOBAL_STANDERD_LATENCY, RotationDirection.CLOCK); //Started in front of the right or center goal..
        }

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();

        quickTelemetry("Driving away from box towards glyphs, gyro setting: " + bot.getHeadingRadians());
        driveDirectionGyro(40, 0, -180, new Predicate() {
            @Override
            public boolean isTrue() {
                if (!inTakeOn[0] && robotZXPhi[0] > 10) {
                    inTakeOn[0] = true;
                    inGlyphPit[0] = true;
                    bot.setIntakeOn();
                    checkingForStall = true;
                    checkForStall();
                }
                return robotZXPhi[0] > 30;
            }
        });

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        turnToHeadingGyroQuick(155, GLOBAL_STANDERD_TOLERANCE, GLOBAL_STANDERD_LATENCY, RotationDirection.CLOCK);

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();

        driveDirectionGyro(40, 0, -205, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 10;
            }
        });

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();

        turnToHeadingGyroQuick(205, GLOBAL_STANDERD_TOLERANCE, GLOBAL_STANDERD_LATENCY, RotationDirection.COUNTER_CLOCK);
        driveDirectionGyro(40, 0, -155, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 10;
            }
        });

        turnToHeadingGyroQuick(180, GLOBAL_STANDERD_TOLERANCE, GLOBAL_STANDERD_LATENCY, RotationDirection.CLOCK);

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        quickTelemetry("Driving back towards box.");
        driveDirectionGyro(40, 180, -180, new Predicate() {
            @Override
            public boolean isTrue() {
                if (inTakeOn[0] && robotZXPhi[0] < -40) {
                    inTakeOn[0] = false;
                    inGlyphPit[0] = false;
                    bot.setIntakeOff();
                    checkingForStall = false;
                }
                return robotZXPhi[0] < -75;
            }
        });

        bot.setFlipPlateUpwards();

        sleep(1000);

        bot.setGlyphPincherStartPos();

        sleep(500);

        bot.setGlyphPincherClosed();

        bot.setFlipPlateDownwards();

        sleep(500);

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
        driveDirectionGyro(40, 180, -180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < -10;
            }
        });

        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();

        driveDirectionGyro(40, 0, -180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 20;
            }
        });

    }

    public void quickTelemetry(String string) {
        telemetry.addData("Debug: ", string);
        telemetry.update();
    }

    public void checkForStall() {
        new Thread(new Runnable() {
            ElapsedTime et = new ElapsedTime();

            @Override
            public void run() {
                while (!isStopRequested()) {
                    if (!checkingForStall) {
                        return;
                    }
                    if (bot.isIntakeStalled() && !unStalling) {
                        if (et.milliseconds() > 150) {
                            multiThreadedHandleStall();
                            et.reset();
                            unStalling = true;
                        }
                    }

                    if (!bot.isIntakeStalled()) et.reset();
                }
            }
        }).start();
    }

    public void multiThreadedHandleStall() {
        new Thread(new Runnable() {
            @Override
            public void run() {
                ElapsedTime et = new ElapsedTime();
                double reverseIntakeTimeMS = 100;
                double reIntakeTimeMS = 250;
                while (!isStopRequested()) {
                    if (et.milliseconds() < reverseIntakeTimeMS) {
                        bot.setIntakeReverse();
                    } else if (et.milliseconds() > reverseIntakeTimeMS) {
                        reverseIntakeTimeMS = 250;
                        et.reset();
                        bot.setIntakeOn();
                    } else if (et.milliseconds() > reIntakeTimeMS) {
                        unStalling = false;
                        return;
                    }


                    //End loop under this.
                }
            }
        }).start();
    }


    public enum TriangleApproachSide {LEFT, RIGHT}

    public enum TriangleMode {INSIDE, OUTSIDE}

    final float TAPE_WIDTH = 5;

    //Handles navigation of the triangle, including line following, and adjusting position
    //Assumes that the at the start of execution, the trailing color sensor has just gotten to the triangle
    //If mode is OUTSIDE, assumes that the leading sensor is either on or has passed over the far side of the triangle
    //If mode is INSIDE, assumes that the leading sensor is either on or has not yet reached the far side of the triangle

    public void handleTriangle(TriangleApproachSide approachSide, TriangleMode mode) {

        final ColorSensor trailingSensor, leadingSensor;
        final ColorSensor lineFollowingSensor, controlSensor;
        LineFollowSide lineFollowSide;
        float tapeAngleRads;
        float approachTriangleDirection;

        if (approachSide == TriangleApproachSide.LEFT) {
            trailingSensor = bot.colorLeft;
            leadingSensor = bot.colorRight;
            lineFollowSide = LineFollowSide.LEFT;
            tapeAngleRads = mode == TriangleMode.OUTSIDE ? INNER_TAPE_ANGLE_RADS : -INNER_TAPE_ANGLE_RADS;
            approachTriangleDirection = 90;
        } else {
            trailingSensor = bot.colorRight;
            leadingSensor = bot.colorLeft;
            lineFollowSide = LineFollowSide.RIGHT;
            tapeAngleRads = mode == TriangleMode.OUTSIDE ? -INNER_TAPE_ANGLE_RADS : INNER_TAPE_ANGLE_RADS;
            approachTriangleDirection = -90;
        }

        //If mode == INSIDE, need to drive approachTriangleDirection until the leading sensor is over the far tape, then
        //the LEADING sensor will become the LINE FOLLOWING sensor and the TRAILING sensor will become the CONTROL sensor.
        //
        //If mode == OUTSIDE, no additional drive is needed; the LEADING sensor will become the CONTROL sensor and the
        //TRAILING sensor will become the LINE FOLLOWING sensor.

        if (mode == TriangleMode.INSIDE) {
            //robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
            driveDirectionGyro(DRIVE_TOWARDS_TRIANGLE_SPEED, approachTriangleDirection, new Predicate() {
                float[] hsv = new float[3];

                @Override
                public boolean isTrue() {
                    Color.RGBToHSV(leadingSensor.red() * 8, leadingSensor.green() * 8, leadingSensor.blue() * 8, hsv);
                    return hsv[1] > HSV_SAT_CUT_OFF;
                }
            });
            lineFollowingSensor = leadingSensor;
            controlSensor = trailingSensor;
        } else {
            lineFollowingSensor = trailingSensor;
            controlSensor = leadingSensor;
        }

        //Determine whether we need to follow line forward (lineFollowSpeed > 0) or reverse (lineFollowSpeed < 0)
        //Then FOLLOW LINE using previously-determined lineFollowSide, tapeAngleRads, lineFollowingSensor,
        //lineFollowSpeed; the Predicate will use the controlSensor

        float lineFollowSpeed;
        float[] hsv = new float[3];
        Color.RGBToHSV(controlSensor.red(), controlSensor.green(), controlSensor.blue(), hsv);
        if (hsv[1] < HSV_SAT_CUT_OFF) {
            lineFollowSpeed = mode == TriangleMode.OUTSIDE ? LINE_FOLLOW_SPEED : -LINE_FOLLOW_SPEED;
            followLineProportionate(lineFollowSide, tapeAngleRads, lineFollowingSensor, lineFollowSpeed,
                    new Predicate() {
                        float[] hsv = new float[3];

                        @Override
                        public boolean isTrue() {
                            Color.RGBToHSV(controlSensor.red(), controlSensor.green(), controlSensor.blue(), hsv);
                            return hsv[1] > HSV_SAT_CUT_OFF;
                        }
                    });
        } else {
            lineFollowSpeed = mode == TriangleMode.OUTSIDE ? -LINE_FOLLOW_SPEED : LINE_FOLLOW_SPEED;
            followLineProportionate(lineFollowSide, tapeAngleRads, lineFollowingSensor, lineFollowSpeed,
                    new Predicate() {
                        float[] hsv = new float[3];

                        @Override
                        public boolean isTrue() {
                            Color.RGBToHSV(controlSensor.red(), controlSensor.green(), controlSensor.blue(), hsv);
                            return hsv[1] < HSV_SAT_CUT_OFF;
                        }
                    });
        }

        //Now, ADJUST POSITION ON TRIANGLE using the appropriate method
        //NOTE: for mode = INSIDE, the robot will end up closer to the cryptobox than for mode = OUTSIDE.
        //Add a driveDirectionGyro statement here to do a short backup ( tape width / sin(INNER_TAPE_ANGLE_RADS) )

        if (mode == TriangleMode.OUTSIDE) {
            adjustPosOnTriangle(ADJUST_POS_TIMEOUT);
        } else {
            adjustPosInsideTriangle(ADJUST_POS_TIMEOUT);
            final float backupDistance = TAPE_WIDTH / (float) Math.sin(INNER_TAPE_ANGLE_RADS);
            setOdometry(0, 0);
            //robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
            driveDirectionGyro(25, 0, new Predicate() {
                @Override
                public boolean isTrue() {
                    return robotZXPhi[0] >= backupDistance;
                }
            });
        }

    }

    protected void setOdometry(float z, float x, float odomHeading){
        robotZXPhi = new float[] {z, x, odomHeading};
        bot.updateOdometry();
    }

    protected void setOdometry(float z, float x){
        robotZXPhi = new float[] {z, x, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
    }


}
