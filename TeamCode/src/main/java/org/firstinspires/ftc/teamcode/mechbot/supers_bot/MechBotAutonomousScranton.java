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
import org.firstinspires.ftc.teamcode.cv_programs.CryptoNavReverse;
import org.firstinspires.ftc.teamcode.cv_programs.ImgProc;
import org.firstinspires.ftc.teamcode.mechbot.MechBotAutonomous;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

import java.util.ArrayList;
import java.util.concurrent.BlockingQueue;

import static org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotSensorScranton.*;

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

    public final float OFF_STONE_SPEED = 60.0f;
    protected final float INNER_TAPE_ANGLE = 33.70f;
    protected final float INNER_TAPE_ANGLE_RADS = INNER_TAPE_ANGLE * ((float) Math.PI / 180.0f);
    public final float LINE_FOLLOW_SPEED = 20.0f; //10 centimeters per second.
    private final float LINE_FOLLOW_ANGLE_FACTOR = 30.0f * ((float) Math.PI / 180.0f); //30.0 Degrees converted to radians.
    protected final float HEADING_CORECTION_FACTOR = 2.0f;
    public final float DRIVE_TOWARDS_TRIANGLE_SPEED = 20.0f;

    public enum JewelSide {BLUE_LEFT, RED_LEFT, UNKNOWN}

    public enum TeamColor {BLUE, RED}

    public enum Side {LEFT, RIGHT, UNKNOWN}

    protected float[] robotZXPhi = null;

    protected final String DRIVE_DIRECTION_GYRO_TAG = "DRIVE_DIRECTION_GYRO";
    protected final boolean DRIVE_DIRECTION_GYRO_LOG = true;

    protected final String TURN_ANGLE_TAG = "TURN_ANGLE_TAG";
    protected final boolean TURN_ANGLE_LOG = false;

    protected final String TURN_TO_HEADING_TAG = "TURN_TO_HEADING";
    protected final boolean TURN_TO_HEADING_LOG = false;

    protected final String DRIVE_GYRO_TIME_TAG = "DRIVE_GYRO_TIME";
    protected final boolean DRIVE_GYRO_TIME_LOG = false;

    protected final String FOLLOW_LINE_PROP_TAG = "FOLLOW_LINE_PROP";
    protected final boolean FOLLOW_LINE_PROP_LOG = true;

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

    protected final String HANDLE_TRIANGLE_TAG = "HANDLE_TRIANGLE";
    protected final boolean HANDLE_TRIANGLE_LOG = true;

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

    public final float CRYPTO_BOX_CENTER_SHIFT_VALUE = 0f; //was 1.0f on 1/11/18 bottom left over shot some changing to test.

    public final float CRYPTO_BOX_FORWARD_SHIFT_VALUE = -22;

    public final float TOUCH_SENSOR_OFFSET_DISTANCE = -4.0f;

    public final float TOUCH_SENSOR_SPEED = 20.0f;

    public final float ADJUST_POS_TIMEOUT = 4000;

    public final int FLIP_PLATE_UPTICKS_AUTO = 350;
    public final int FLIP_PLATE_DOWNTICKS_AUTO = -215;

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
        if (FOLLOW_LINE_PROP_LOG) BetaLog.dd(FOLLOW_LINE_PROP_TAG, "Entering Follow Line");
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
                BetaLog.dd(FOLLOW_LINE_PROP_TAG, "Heading %.2f Sat %.2f error %.2f", heading*180.0/Math.PI, hsvValues[1], err);

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


    //New followLineProportionate overload. This one accepts a gyroHeadingTargetDegrees. Using 180 degrees will
    //allow line following with the glyph intake pointed away from the cryptobox.
    public void followLineProportionate(LineFollowSide side, float tapeAngleRads, ColorSensor colorSensor,
                                        float lineFollowSpeed, float gyroHeadingTargetDegrees, Predicate finish) {
        if (FOLLOW_LINE_PROP_LOG) BetaLog.dd(FOLLOW_LINE_PROP_TAG, "Entering Follow Line");
        float[] hsvValues = new float[3];
        final float coeff = 20.0f;
        float gyroHeadingTargetRadians = gyroHeadingTargetDegrees * (float) Math.PI / 180.0f;
        while (opModeIsActive()) {
            if (finish.isTrue()) break;
            float heading = bot.getHeadingRadians();
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            float err = side == LineFollowSide.LEFT ? 0.5f - hsvValues[1] : hsvValues[1] - 0.5f;
            if (err < -0.5) err = -0.4f;
            else if (err > 0.5) err = 0.4f;
            if (FOLLOW_LINE_PROP_LOG)
                BetaLog.dd(FOLLOW_LINE_PROP_TAG, "Heading %.2f Sat %.2f error %.2f", heading*180.0/Math.PI, hsvValues[1], err);

            float odomHeading = bot.getOdomHeadingFromGyroHeading(heading);
            float angleDiff = odomHeading - (float) Math.PI - tapeAngleRads;

            float vx = lineFollowSpeed * (float) Math.sin(angleDiff) + coeff * err * (float) Math.cos(angleDiff);
            float vy = lineFollowSpeed * (float) Math.cos(angleDiff) - coeff * err * (float) Math.sin(angleDiff);


            float va = -(float) VuMarkNavigator.NormalizeAngle(heading - gyroHeadingTargetRadians) * HEADING_CORECTION_FACTOR;

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
        final float vaMax = 1.0f * (float) Math.PI; //was 0.6f
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

        int y0 = 360; //Was 300
        int croppedImgWidth = imgWidth;
        int croppedImgHeight = 360; //Was 420

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
                if (redBlobs.get(0).getAvgX() < redBlobs.get(1).getAvgX())
                    redBlobs.remove(0);  // > inverted.
                else redBlobs.remove(1);
            }


            if (redBlobs.size() > 0 && blueBlobs.size() > 0) {
                Blob redBlob = redBlobs.get(0); //There is only one blob left in the red list; this is it.

                //Find the largest blue blob; it will be assumed to be the blue jewel.
                Blob blueBlob = blueBlobs.get(0);
                for (int i = 1; i < blueBlobs.size(); i++)
                    if (blueBlobs.get(i).getRectArea() > blueBlob.getRectArea())
                        blueBlob = blueBlobs.get(i);

                if (blueBlob.getAvgX() < redBlob.getAvgX())
                    return JewelSide.BLUE_LEFT; // > inverted.
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
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "gHeading = %.4f gH Target = %.4f", gyroHeading, gyroHeadingTargetRadians);


            float headingError = (float) VuMarkNavigator.NormalizeAngle(gyroHeading - gyroHeadingTargetRadians);
            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "HeadingError = %.4f ", headingError);
            float va = -HEADING_CORECTION_FACTOR * headingError;

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "vx = %.2f  vy = %.2f  va = %.2f", vx, vy, va * 180.0 / Math.PI);

            bot.setDriveSpeed(vx, vy, va);

        }
        bot.setDrivePower(0, 0, 0);
    }

    //Robot heading in degrees.
    public void driveDirectionGyroCryptoNav(float speedCMs, float directionAngleDegrees, float gyroHeadingTargetDegrees,
                                            BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue, int rawImgWidth,
                                            int rawImgHeight, byte[] imageBytes, Predicate finish) {
        if (DRIVE_DIRECTION_GYRO_LOG)
            BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "Entering driveDirectionGyro");
        // bot.updateOdometry();
        float directionAngleRadians = directionAngleDegrees * (float) Math.PI / 180.0f;
        float gyroHeadingTargetRadians = gyroHeadingTargetDegrees * (float) Math.PI / 180.0f;

        while (opModeIsActive()) {
            float gyroHeading = bot.getHeadingRadians();
            float odomHeading = bot.getOdomHeadingFromGyroHeading(gyroHeading);
            float cameraHeading = bot.getCameraHeadingFromGyroHeading(gyroHeading);

            if (DRIVE_DIRECTION_GYRO_LOG)
                BetaLog.dd(DRIVE_DIRECTION_GYRO_TAG, "gHeading = %.2f  oHeading = %.2f Camera heading = %.2f",
                        gyroHeading * 180.0 / Math.PI, odomHeading * 180.0 / Math.PI, cameraHeading * 180.0f / Math.PI); //Fixed convert error
            float[] cameraZX = null;
            if (VuMarkNavigator.getRGB565Array(frameQueue, rawImgWidth, rawImgHeight, imageBytes)) {
                //Have an image.
                cameraZX = CryptoNavReverse.updateLocationZX(imageBytes, cameraHeading);
            }

            if (cameraZX == null) {
                bot.updateOdometry(robotZXPhi, odomHeading);
            } else {
                float[] robotZX = bot.getRobotZXfromCameraZX(cameraZX, gyroHeading);
                robotZXPhi[0] = robotZX[0];
                robotZXPhi[1] = robotZX[1];
                robotZXPhi[2] = odomHeading;
                bot.updateOdometry();
            }


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
        VuMarkNavigator.activate(false);
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

        bot.setGlyphPincherClosed();

        vuMark = RelicRecoveryVuMark.UNKNOWN;
        while (!isStarted() && !isStopRequested()){
            vuMark = VuMarkNavigator.getRelicRecoveryVumark();
            telemetry.addData("Key: ", vuMark);
            telemetry.update();
        }

        //waitForStart();

        runTime = new ElapsedTime();

        bot.setRelicArmStop();

        //et.reset();

        //vuMark = findKey(cryptoKeyTimeOut);
        //double vuMarkFindTime = et.milliseconds();
        et.reset();
        //telemetry.addData("Found vuMark value of " + vuMark.toString() + " after " + vuMarkFindTime + " milliseconds.", "");

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
        //
        //telemetry.addData("Found both the jewel and vuMark in: " + vuMarkFindTime + jewlFindTime + " milliseconds. ", "");
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


    //Overload of adjustPosOnTriangle. This one allows any desired speed, and any desired target gyroHeading.
    //Pass in the color sensors that will be to the right and left of the line.
    public void adjustPosOnTriangle(float speed, float gyroHeadingTargetDegrees, ColorSensor colorRight,
                                    ColorSensor colorLeft, double timeOut) {
        if (ADJUST_POS_LOG) BetaLog.dd(ADJUST_POS_TAG, "Entering Adjust Pos on Triangle.");
        final double interSensorDist = 34;
        final double sensorOffSet = 10;
        final double sensorR = Math.sqrt(interSensorDist * interSensorDist / 4.0 + sensorOffSet * sensorOffSet);
        final double sensorAngle = Math.atan(2.0 * sensorOffSet / interSensorDist);
        final float specialCoeff = (float) (sensorR * Math.sin(sensorAngle + INNER_TAPE_ANGLE_RADS) / Math.cos(INNER_TAPE_ANGLE_RADS));
        final float CAngle = 2.0f;
        final float CSum = speed;
        final float CDiff = speed;
        final float tolAngle = 2.0f * (float) Math.PI / 180f;
        final float tolSum = .2f;
        final float tolDiff = .2f;
        float gyroHeadingTargetRadians = gyroHeadingTargetDegrees * (float) Math.PI / 180.0f;

        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive() && time.milliseconds() < timeOut) {
            float gyroHeading = bot.getHeadingRadians();
            float[] hsvRight = new float[3];
            float[] hsvLeft = new float[3];
            Color.RGBToHSV(colorRight.red(), colorRight.green(), colorRight.blue(), hsvRight);
            Color.RGBToHSV(colorLeft.red(), colorLeft.green(), colorLeft.blue(), hsvLeft);
            float sumSat = hsvRight[1] + hsvLeft[1];
            float diffSat = hsvRight[1] - hsvLeft[1];

            if (Math.abs(gyroHeading) < tolAngle && Math.abs(sumSat - 1.0f) < tolSum && Math.abs(diffSat) < tolDiff) {
                if (ADJUST_POS_LOG) BetaLog.dd(ADJUST_POS_TAG, "Adjust Pos Succeeded!!!");
                break;
            }

            float va = -CAngle * (float) VuMarkNavigator.NormalizeAngle(gyroHeading - gyroHeadingTargetRadians);

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


    /**
     * Overload of adjustPosInsideTriangle allows any adjustment speed, target gyro heading, and any sensor
     * may be used as right and left.
     */
    public void adjustPosInsideTriangle(float speed, float gyroHeadingTargetDegrees, ColorSensor colorRight,
                                        ColorSensor colorLeft, double timeOut) {
        if (ADJUST_POS_LOG) BetaLog.dd(ADJUST_POS_TAG, "Entering Adjust Pos on Triangle.");
        final double interSensorDist = 34;
        final double sensorOffSet = 10;
        final double sensorR = Math.sqrt(interSensorDist * interSensorDist / 4.0 + sensorOffSet * sensorOffSet);
        final double sensorAngle = Math.atan(2.0 * sensorOffSet / interSensorDist);
        final float specialCoeff = (float) (sensorR * Math.sin(sensorAngle + INNER_TAPE_ANGLE_RADS) / Math.cos(INNER_TAPE_ANGLE_RADS));
        final float CAngle = 2.0f;
        final float CSum = -speed;
        final float CDiff = -speed;
        final float tolAngle = 2.0f * (float) Math.PI / 180f;
        final float tolSum = .2f;
        final float tolDiff = .2f;
        float gyroHeadingTargetRadians = gyroHeadingTargetDegrees * (float) Math.PI / 180.0f;

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

            float va = -CAngle * (float) VuMarkNavigator.NormalizeAngle(gyroHeading - gyroHeadingTargetRadians);

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

    public void freeFlipPlate() {
        bot.setRelicLiftDown();
        sleep(250);
        bot.setRelicLiftStop();
    }

    public void scoreGlyph(RelicRecoveryVuMark target) {
        setOdometry(0, 0);
//        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
//        bot.updateOdometry();
        switch (target) {
            case LEFT:
                if (PREPARE_SCORE_LOG) BetaLog.dd(PREPARE_SCORE_TAG, "driveDirectionGyro left");
                driveDirectionGyro(30, -90, 180, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return robotZXPhi[1] < -CRYPTO_BOX_SIDE_SHIFT_VALUE + CRYPTO_BOX_CENTER_SHIFT_VALUE;
                    }
                });
                break;
            case RIGHT:
                if (PREPARE_SCORE_LOG) BetaLog.dd(PREPARE_SCORE_TAG, "driveDirectionGyro right");
                driveDirectionGyro(30, 90, 180, new Predicate() {
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
                    driveDirectionGyro(30, 90, 180, new Predicate() {
                        @Override
                        public boolean isTrue() {
                            return robotZXPhi[1] > CRYPTO_BOX_CENTER_SHIFT_VALUE;
                        }
                    });
                } else {
                    driveDirectionGyro(30, -90, 180, new Predicate() {
                        @Override
                        public boolean isTrue() {
                            return robotZXPhi[1] < CRYPTO_BOX_CENTER_SHIFT_VALUE;
                        }
                    });
                }
                break;
        }

        if (PREPARE_SCORE_LOG) BetaLog.dd(PREPARE_SCORE_TAG, "driveDirectionGyro 3");

        bot.setTouchServoOut();
        setOdometry(0, 0);
        driveDirectionGyro(30, 180, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < CRYPTO_BOX_FORWARD_SHIFT_VALUE;
            }
        });

        setOdometry(0,0);
        driveDirectionGyro(TOUCH_SENSOR_SPEED, 90, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                if(robotZXPhi[1] > 16) return true;
                return !bot.touchSensor.getState();
            }
        });

        setOdometry(0, 0);

        driveDirectionGyro(TOUCH_SENSOR_SPEED, -90, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[1] < TOUCH_SENSOR_OFFSET_DISTANCE;
            }
        });
        bot.setRetractKicker();
        sleep(100);
        bot.setFlipPosition(FLIP_PLATE_UPTICKS_AUTO);
        bot.backStop.setPosition(0);

        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive() && bot.flipMotor.isBusy() && et.milliseconds()< 800);
        //sleep(600);
        bot.setGlyphPincherMidPos();

        sleep(400);
        setOdometry(0, 0);

        driveDirectionGyro(50, 0, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 5;
            }
        });

        sleep(300);

        bot.setFlipPosition(FLIP_PLATE_DOWNTICKS_AUTO);
        bot.backStop.setPosition(.28);
        bot.setTouchServoStore();

        setOdometry(0,0);

        driveDirectionGyro(50, 180, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < -4;
            }
        });

        setOdometry(0,0);

        driveDirectionGyro(50, 0, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 4;
            }
        });
    }


    public void multiGlyph(RelicRecoveryVuMark target) {
        switch (this.cryptoKey) {
            case LEFT:
                driveDirectionGyro(40, 90, 180, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return robotZXPhi[1] > CRYPTO_BOX_SIDE_SHIFT_VALUE + CRYPTO_BOX_CENTER_SHIFT_VALUE;
                    }
                });
                break;
            case RIGHT:
                driveDirectionGyro(40, -90, 180, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        return robotZXPhi[1] < -CRYPTO_BOX_SIDE_SHIFT_VALUE + CRYPTO_BOX_CENTER_SHIFT_VALUE;
                    }
                });
                break;
            case CENTER:
            case UNKNOWN:
                break;
        }

        //Is intake down?
        bot.setFlipPosition(FLIP_PLATE_UPTICKS_AUTO);
        bot.backStop.setPosition(0);

        setOdometry(0, 0);

        driveDirectionGyro(5000, 0, 180, new Predicate() {
            boolean flipDown = false;
            @Override
            public boolean isTrue() {
                if(robotZXPhi[0] >10 && !flipDown){
                    bot.setFlipPosition(FLIP_PLATE_DOWNTICKS_AUTO);
                    bot.backStop.setPosition(.28);
                    bot.setGlyphPincherMidPos();
                    flipDown = true;
                }
                return robotZXPhi[0] > 60;
            }
        });

        driveDirectionGyro(30, 180, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < 55;
            }
        });

        bot.setIntakeOn();

        driveDirectionGyro(20, 0, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 80;
            }
        });

        driveDirectionGyro(50, 180, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < 60;
            }
        });

        bot.setIntakeReverse();

        sleep(200);

        bot.setIntakeOn();
        sleep(300);


        multiGlyphScore(target);

        return;
    }

    public boolean multiGlyphScore(RelicRecoveryVuMark target) {
            bot.setKickGlyph();
            sleep(200);
            bot.setGlyphPincherClosed();

            bot.setIntakeReverse();
            handleTriangleFromFront(DRIVE_TOWARDS_TRIANGLE_SPEED, LINE_FOLLOW_SPEED, 20, 180, bot.backColorLeft, bot.backColorRight, 0); //Was 25
            bot.setIntakeOff();
            scoreGlyph(target);
            return true;
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

        // bot.setFlipPosition(bot.flipPlateUpticks);
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
        //bot.setPivotStart();
        //bot.setArmCube();
        //sleep(250);
        if(side == Side.UNKNOWN) return false;
        bot.setPivotEnd();
        sleep(700);
        bot.setArmJewel();
        sleep(700);

        if (goForRankingPoints) {
            //This logic flow, will score the jewel for the ENEMY team.
            return true;
        } else {
            if (side == Side.UNKNOWN) {
                return false;
            } else if (side == Side.LEFT) {
                bot.knockPivotLeft();
                sleep(500);
            } else if (side == Side.RIGHT) {
                bot.knockPivotRight();
                sleep(500);
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
        //bot.setFlipPosition(bot.flipPlateDownticks);
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

//        bot.setFlipPosition(bot.flipPlateUpticks);

        sleep(1000);

        bot.setGlyphPincherStartPos();

        sleep(500);

        bot.setGlyphPincherClosed();

        // bot.setFlipPosition(bot.flipPlateDownticks);

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

    public void handleTriangle(TriangleApproachSide approachSide, float followSpeed, float adjustSpeed, float gyroHeadingTargetDegrees,
                               final ColorSensor colorLeft, final ColorSensor colorRight, double timeOut) {

        if (HANDLE_TRIANGLE_LOG) BetaLog.dd(HANDLE_TRIANGLE_TAG, "Entering Handle Triangle");
        final float[] hsvLeft = new float[3];
        final float[] hsvRight = new float[3];

        if (approachSide == TriangleApproachSide.LEFT) {
            Color.RGBToHSV(colorRight.red(), colorRight.green(), colorRight.blue(), hsvRight);
            if (hsvRight[1] > HSV_SAT_CUT_OFF) { //Follow reverse
                if (HANDLE_TRIANGLE_LOG)
                    BetaLog.dd(HANDLE_TRIANGLE_TAG, "Follow Left Line in Reverse");
                followLineProportionate(LineFollowSide.LEFT, INNER_TAPE_ANGLE_RADS, colorLeft, -followSpeed, gyroHeadingTargetDegrees, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        Color.RGBToHSV(colorRight.red(), colorRight.green(), colorRight.blue(), hsvRight);
                        return hsvRight[1] < HSV_SAT_CUT_OFF;
                    }
                });
            } else { //Follow forward
                if (HANDLE_TRIANGLE_LOG)
                    BetaLog.dd(HANDLE_TRIANGLE_TAG, "Follow Left Line Forward");
                followLineProportionate(LineFollowSide.LEFT, INNER_TAPE_ANGLE_RADS, colorLeft, followSpeed, gyroHeadingTargetDegrees, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        Color.RGBToHSV(colorRight.red(), colorRight.green(), colorRight.blue(), hsvRight);
                        return hsvRight[1] > HSV_SAT_CUT_OFF;
                    }
                });
            }
        } else {
            Color.RGBToHSV(colorLeft.red(), colorLeft.green(), colorLeft.blue(), hsvLeft);
            if (hsvLeft[1] > HSV_SAT_CUT_OFF) { //Follow reverse
                if (HANDLE_TRIANGLE_LOG)
                    BetaLog.dd(HANDLE_TRIANGLE_TAG, "Follow Right Line in Reverse");
                followLineProportionate(LineFollowSide.RIGHT, -INNER_TAPE_ANGLE_RADS, colorRight, -followSpeed, gyroHeadingTargetDegrees, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        Color.RGBToHSV(colorLeft.red(), colorLeft.green(), colorLeft.blue(), hsvLeft);
                        return hsvLeft[1] < HSV_SAT_CUT_OFF;
                    }
                });
            } else { //Follow forward
                if (HANDLE_TRIANGLE_LOG)
                    BetaLog.dd(HANDLE_TRIANGLE_TAG, "Follow Right Line in Forward");
                followLineProportionate(LineFollowSide.RIGHT, -INNER_TAPE_ANGLE_RADS, colorRight, followSpeed, gyroHeadingTargetDegrees, new Predicate() {
                    @Override
                    public boolean isTrue() {
                        Color.RGBToHSV(colorLeft.red(), colorLeft.green(), colorLeft.blue(), hsvLeft);
                        return hsvLeft[1] > HSV_SAT_CUT_OFF;
                    }
                });
            }
        }
        if (HANDLE_TRIANGLE_LOG) BetaLog.dd(HANDLE_TRIANGLE_TAG, "About to Adjust Pos");
        adjustPosOnTriangle(adjustSpeed, gyroHeadingTargetDegrees, colorRight, colorLeft, timeOut);
    }


    /**
     * This method is intended to handle the triangle when approaching from the pile
     * of glyphs. Bot must be positioned such that one specified sensor will be to left and one to the right of the
     * triangle apex. Specify speeds for triangle approach, line following, and adjustment. This will handle from any orientation.
     * Use gyroHeadingTargetDegrees = 0 to travel intake-first. Use gyroHeadingTargetDegrees = 180 to go flip plate-first.
     */
    public void handleTriangleFromFront(float approachSpeed, float followSpeed, float adjustSpeed, float gyroHeadingTargetDegrees,
                                        final ColorSensor colorLeft, final ColorSensor colorRight, double timeOut) {

        final float[] hsvLeft = new float[3];
        final float[] hsvRight = new float[3];

        //Drive toward cryptobox until one of the sensors hits the tape
        driveDirectionGyro(approachSpeed, 180, gyroHeadingTargetDegrees, new Predicate() {
            @Override
            public boolean isTrue() {
                Color.RGBToHSV(colorLeft.red(), colorLeft.green(), colorLeft.blue(), hsvLeft);
                Color.RGBToHSV(colorRight.red(), colorRight.green(), colorRight.blue(), hsvRight);
                return hsvLeft[1] > HSV_SAT_CUT_OFF || hsvRight[1] > HSV_SAT_CUT_OFF;
            }
        });

        //Figure out which side of triangle to line follow, then line follow
        if (hsvLeft[1] > HSV_SAT_CUT_OFF) {      //Following left line
            if (hsvRight[1] > HSV_SAT_CUT_OFF) {    //in reverse
                followLineProportionate(LineFollowSide.LEFT, INNER_TAPE_ANGLE_RADS, colorLeft, -followSpeed, gyroHeadingTargetDegrees,
                        new Predicate() {
                            @Override
                            public boolean isTrue() {
                                Color.RGBToHSV(colorRight.red(), colorRight.green(), colorRight.blue(), hsvRight);
                                return hsvRight[1] <= HSV_SAT_CUT_OFF;
                            }
                        });
            } else {                                //in forward
                followLineProportionate(LineFollowSide.LEFT, INNER_TAPE_ANGLE_RADS, colorLeft, followSpeed, gyroHeadingTargetDegrees,
                        new Predicate() {
                            @Override
                            public boolean isTrue() {
                                Color.RGBToHSV(colorRight.red(), colorRight.green(), colorRight.blue(), hsvRight);
                                return hsvRight[1] > HSV_SAT_CUT_OFF;
                            }
                        });
            }
        } else {                                  //Following right line, forward
            followLineProportionate(LineFollowSide.RIGHT, -INNER_TAPE_ANGLE_RADS, colorRight, followSpeed, gyroHeadingTargetDegrees,
                    new Predicate() {
                        @Override
                        public boolean isTrue() {
                            Color.RGBToHSV(colorLeft.red(), colorLeft.green(), colorLeft.blue(), hsvLeft);
                            return hsvLeft[1] > HSV_SAT_CUT_OFF;
                        }
                    });
        }

        //Adjust Position on triangle

        adjustPosOnTriangle(adjustSpeed, gyroHeadingTargetDegrees, colorRight, colorLeft, timeOut);
    }

    protected void setOdometry(float z, float x, float odomHeading) {
        robotZXPhi = new float[]{z, x, odomHeading};
        bot.updateOdometry();
    }

    protected void setOdometry(float z, float x) {
        robotZXPhi = new float[]{z, x, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        bot.updateOdometry();
    }
    public void topMultiGlyph(TeamColor teamColor, RelicRecoveryVuMark target) {

        final float PILE_DRIVE_X_SHIFT = 14f * 2.54f;
        final float PILE_DRIVE_Z_SHIFT = 31f * 2.54f;
        float pileDriveDir = teamColor == TeamColor.BLUE ?
                (float)Math.atan2(PILE_DRIVE_X_SHIFT, PILE_DRIVE_Z_SHIFT) :
                (float)Math.atan2(-PILE_DRIVE_X_SHIFT, PILE_DRIVE_Z_SHIFT);
        float reversePileDriveDir = (float)VuMarkNavigator.NormalizeAngle(pileDriveDir + Math.PI);
        float pileDriveDirDegrees = pileDriveDir * 180f/(float)Math.PI;
        float reversePileDriveDirDegrees = reversePileDriveDir * 180f/(float)Math.PI;
        float pileDriveGyroHeading = bot.getGyroHeadingFromOdomHeading(pileDriveDir);
        float pileDriveGyroHeadingDegrees = pileDriveGyroHeading * 180f/(float)Math.PI;

        final float INITIAL_SIDE_SHIFT;
        final float X_TARGET;

        //Need to move robot from right in front of the target column to either the right (for BLUE)
        //or left (for RED) column. Once this position is reached, set odometry to (0,0);
        //X_TARGET will be the target X value when returning from the pile. It has an offset of -4
        //to make sure the TOUCH arm doesn't hit the rail.

        switch (target) {
            case LEFT:
                INITIAL_SIDE_SHIFT = teamColor == TeamColor.BLUE?
                        2.0f * CRYPTO_BOX_SIDE_SHIFT_VALUE : 0;
                X_TARGET = -6;
                break;
            case RIGHT:
                INITIAL_SIDE_SHIFT = teamColor == TeamColor.BLUE?
                        0 : -2.0f * CRYPTO_BOX_SIDE_SHIFT_VALUE;
                X_TARGET = -6;
                break;
            case CENTER:
            default:
                INITIAL_SIDE_SHIFT = teamColor == TeamColor.BLUE?
                        CRYPTO_BOX_SIDE_SHIFT_VALUE : - CRYPTO_BOX_SIDE_SHIFT_VALUE;
                X_TARGET = -INITIAL_SIDE_SHIFT - 6;
                break;
        }

        float initialSideShiftDirectionDegrees = INITIAL_SIDE_SHIFT >=0? 90.0f : -90.0f;

        setOdometry(0,0);
        driveDirectionGyro(50, initialSideShiftDirectionDegrees, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return INITIAL_SIDE_SHIFT >= 0? robotZXPhi[1] > INITIAL_SIDE_SHIFT : robotZXPhi[1] < INITIAL_SIDE_SHIFT;
            }
        });

        setOdometry(0,0);

        //Throughout the remainder of the method, odometry will be tracked, but NOT RESET!
        // (0,0) will continue to refer to the location the bot is at right now.

        //Turn toward pile

        turnToHeadingGyroQuick(pileDriveGyroHeadingDegrees, GLOBAL_STANDERD_TOLERANCE * 2.0f, GLOBAL_STANDERD_LATENCY * 0.5f);

        //Drive fast into pile

        driveDirectionGyro(500, pileDriveDirDegrees, pileDriveGyroHeadingDegrees, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > PILE_DRIVE_Z_SHIFT;
            }
        });

        //Back up 5 cm along line of approach

        driveDirectionGyro(500, reversePileDriveDirDegrees, pileDriveGyroHeadingDegrees, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < PILE_DRIVE_Z_SHIFT - 5;
            }
        });

        //Drive slowly into pile with Intake On, again along line of approach

        bot.setIntakeOn();

        driveDirectionGyro(20, pileDriveDirDegrees, pileDriveGyroHeadingDegrees, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > PILE_DRIVE_Z_SHIFT + 15;
            }
        });

        //Drive back slightly, again along line of approach

        driveDirectionGyro(50, reversePileDriveDirDegrees, pileDriveGyroHeadingDegrees, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < PILE_DRIVE_Z_SHIFT + 10;
            }
        });

        //Set Intake to reverse very briefly, then back to Forward, to free up stuck glyph; Then set Kicker,
        //and close Pincher. Run Intake in reverse during the run back to CryptoBox

        bot.setIntakeReverse();

        sleep(200);

        bot.setIntakeOn();
        sleep(300);

        bot.setKickGlyph();
        sleep(200);

        bot.setGlyphPincherClosed();

        bot.setIntakeReverse();

        //Drive fast in reverse, along original approach line, until the site of the initial
        //run toward the pile is ALMOST reached (until z < 10)

        driveDirectionGyro(500, reversePileDriveDirDegrees, pileDriveGyroHeadingDegrees, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < 10;
            }
        });

        //Turn back toward CryptoBox

        turnToHeadingGyroQuick(180, GLOBAL_STANDERD_TOLERANCE * 2.0f, GLOBAL_STANDERD_LATENCY * 0.5f);

        //Now shift in the X or -X direction until X is about at X_TARGET

        final float FINAL_X_OFFSET = X_TARGET - robotZXPhi[1];

        float finalSideShiftDirectionDegrees = FINAL_X_OFFSET >= 0? 90 : -90;

        driveDirectionGyro(30, finalSideShiftDirectionDegrees, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return FINAL_X_OFFSET >=0? robotZXPhi[1] > X_TARGET : robotZXPhi[1] < X_TARGET;
            }
        });

        //Set the Touch Arm out and drive in to CryptoBox, until Z < 0; May need to change
        // this to Z < some small negative number

        bot.setTouchServoOut();

        driveDirectionGyro(30, 180, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < -9;
            }
        });

        //The rest of this method should parallel the end part of the ScoreGlyph method
        //Modify this so if is the same as the end part of ScoreGlyph

        setOdometry(0,0);
        driveDirectionGyro(TOUCH_SENSOR_SPEED, 90, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                if(robotZXPhi[1] > 16) return true;
                return !bot.touchSensor.getState();
            }
        });

        setOdometry(0, 0);

        driveDirectionGyro(TOUCH_SENSOR_SPEED, -90, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[1] < TOUCH_SENSOR_OFFSET_DISTANCE;
            }
        });
        bot.setRetractKicker();
        sleep(100);
        bot.setFlipPosition(FLIP_PLATE_UPTICKS_AUTO);
        bot.backStop.setPosition(0);

        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive() && bot.flipMotor.isBusy() && et.milliseconds()< 800);
        //sleep(600);
        bot.setGlyphPincherMidPos();

        sleep(400);
        setOdometry(0, 0);

        driveDirectionGyro(50, 0, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 5;
            }
        });

        sleep(500);

        bot.setFlipPosition(FLIP_PLATE_DOWNTICKS_AUTO);
        bot.backStop.setPosition(.28);
        bot.setTouchServoStore();

        setOdometry(0,0);

        driveDirectionGyro(50, 180, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < -4;
            }
        });

        setOdometry(0,0);

        driveDirectionGyro(50, 0, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 4;
            }
        });
    }



}
