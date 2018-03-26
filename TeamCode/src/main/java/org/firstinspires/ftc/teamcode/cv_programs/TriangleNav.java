package org.firstinspires.ftc.teamcode.cv_programs;

import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

import java.util.ArrayList;

/**
 * Created by JimLori on 3/23/2018.
 * <p>
 * Static class containing data and methods for using the Triangle and Blob detection for navigation.
 */

public class TriangleNav {

    //Control of debug logging.

    private static final String TAG = "TriangleNav";
    private static final boolean DEBUG_INIT = true;
    private static final boolean DEBUG_GETBLOBS = true;
    private static final boolean DEBUG_UPDATELOC = true;

    //Width of base of triangle

    private static final float TRIANGLE_WIDTH_CM = 32.0f * 2.54f;
    private static final float RAIL_DEPTH_CM = 4.0f * 2.54f;

    //Minimum number of points in a component blob in order for it to be incorporated into the final target blob

    private static final float MIN_POINTS_PER_COMPONENT_BLOB = 1000;

    //HSV Ranges for Red and Blue

    private static final HSV_Range RED_RANGE = new HSV_Range(345, 15, 0.5f, 1.0f, 0.3f, 1.0f);
    private static final HSV_Range BLUE_RANGE = new HSV_Range(210, 270, 0.5f, 1.0f, 0.1f, 1.0f);

    enum TeamColor {RED, BLUE}

    //Team Color

    private static TeamColor teamColor;

    //Width and Height of raw camera images

    private static int rawImageWidth;
    private static int rawImageHeight;

    //rawX0, rawY0, rawWidth, rawHeight of subrange of raw image

    private static int rawRangeX0;
    private static int rawRangeY0;
    private static int rawRangeWidth;
    private static int rawRangeHeight;

    //Interval between selected pixels for formation of reduced image

    private static int sampleRatio;

    //Width and Height of reduced camera images used to create the blobs

    private static int reducedImageWidth;
    private static int reducedImageHeight;

    //Focal length of image, in reduced image pixels

    private static float focalLength;

    //principal X value of reduced image

    private static float principalX;

    //byte array to hold the reduced image

    private static byte[] reducedImageBytes = null;

    //int array to hold the binary image

    private static int[] binaryImage = null;

    public static void initParams(TeamColor tc, int rawImgWidth, int rawImgHeight, int rangeX0, int rangeY0, int rangeWidth,
                            int rangeHeight, float rawFocalLength, float rawPrincipalX, int smplRatio) {
        if (DEBUG_INIT) RobotLog.dd(TAG,"Begin init");
        teamColor = tc;
        rawImageWidth = rawImgWidth;
        rawImageHeight = rawImgHeight;
        rawRangeX0 = rangeX0;
        rawRangeY0 = rangeY0;
        rawRangeWidth = rangeWidth;
        rawRangeHeight = rangeHeight;
        sampleRatio = smplRatio;
        reducedImageWidth = rawRangeWidth / smplRatio;
        reducedImageHeight = rawRangeHeight / smplRatio;
        principalX = (rawPrincipalX - (float)rangeX0) / (float)smplRatio;
        focalLength = rawFocalLength / smplRatio;
        reducedImageBytes = new byte[reducedImageWidth * reducedImageHeight * 2];
        binaryImage = new int[reducedImageWidth * reducedImageHeight];
        if (DEBUG_INIT) {
            RobotLog.dd(TAG, "rawW = %d rawH = %d rawX0 = %d rawY0 = %d", rawImageWidth, rawImageHeight,
                    rawRangeX0, rawRangeY0);
            RobotLog.dd(TAG, "rngW = %d, rngH = %d sRatio = %d", rawRangeWidth, rawRangeHeight, sampleRatio);
            RobotLog.dd(TAG, "redW = %d redH = %d prnX = %.1f fLen = %.1f", reducedImageWidth,
                    reducedImageHeight, principalX, focalLength);
            RobotLog.dd(TAG, "End init");
        }
    }

    public static void initParams(TeamColor tc, int rangeX0, int rangeY0, int rangeWidth, int rangeHeight, int smplRatio){
        if (DEBUG_INIT) RobotLog.dd(TAG,"Begin init");
        teamColor = tc;
        float[] size = CameraDevice.getInstance().getCameraCalibration().getSize().getData();
        rawImageWidth = Math.round(size[0]);
        rawImageHeight = Math.round(size[1]);
        rawRangeX0 = rangeX0;
        rawRangeY0 = rangeY0;
        rawRangeWidth = rangeWidth;
        rawRangeHeight = rangeHeight;
        sampleRatio = smplRatio;
        reducedImageWidth = rawRangeWidth / smplRatio;
        reducedImageHeight = rawRangeHeight / smplRatio;
        float rawPrincipalX = CameraDevice.getInstance().getCameraCalibration().getPrincipalPoint().getData()[0];
        principalX = (rawPrincipalX - (float)rangeX0) / (float)smplRatio;
        focalLength = CameraDevice.getInstance().getCameraCalibration().getFocalLength().getData()[0] / (float)smplRatio;
        reducedImageBytes = new byte[reducedImageWidth * reducedImageHeight * 2];
        binaryImage = new int[reducedImageWidth * reducedImageHeight];
        if (DEBUG_INIT) {
            RobotLog.dd(TAG, "rawW = %d rawH = %d rawX0 = %d rawY0 = %d", rawImageWidth, rawImageHeight,
                    rawRangeX0, rawRangeY0);
            RobotLog.dd(TAG, "rngW = %d, rngH = %d sRatio = %d", rawRangeWidth, rawRangeHeight, sampleRatio);
            RobotLog.dd(TAG, "redW = %d redH = %d prnX = %.1f fLen = %.1f", reducedImageWidth,
                    reducedImageHeight, principalX, focalLength);
            RobotLog.dd(TAG, "End init");
        }
    }

    public static Blob getTriangleBlobFromRawImage(byte[] rawImageBytes){
        if (DEBUG_GETBLOBS) RobotLog.dd(TAG,"Begin getTriangleBlobFromRawImage");
        HSV_Range range = teamColor == TeamColor.RED? RED_RANGE : BLUE_RANGE;
        ImgProc.getReducedRangeRGB565(rawImageBytes, rawImageWidth, rawImageHeight, rawRangeX0, rawRangeY0,
                rawRangeWidth, rawRangeHeight, reducedImageBytes, sampleRatio);
        ImgProc.getBinaryImage(reducedImageBytes, range, binaryImage);
        ArrayList<Blob> blobs = Blob.findBlobs(binaryImage, reducedImageWidth, reducedImageHeight);
        if (blobs == null || blobs.size() == 0) return null;
        while (blobs.size() > 0 &&  blobs.get(0).getNumPts() < MIN_POINTS_PER_COMPONENT_BLOB) blobs.remove(0);
        if (blobs.size() == 0) return null;
        while (blobs.size() > 1){
            if (blobs.get(1).getNumPts() >= MIN_POINTS_PER_COMPONENT_BLOB) blobs.get(0).merge(blobs.get(1));
            blobs.remove(1);
        }
        return blobs.get(0);
    }

   /*
    Attempts to obtain blob. If successful, uses the location of the back triangle corners,
    and phiPrime (passed in as argument) to determine the location of the camera (zc, xc) relative
    to the CryptoBox. Note that zc=0 cm, xc=0 cm is front,center of cryptobox. Positive xc means
    camera has moved to right of center. Positive zc means that camera has moved away
    from CryptoBox.

    The coordinate system used here is like the coordinate system for a Vuforia target. Note that with this
    method, we need to pass in the phiPrime value (obtained from gyro).
    */

    public static float[] getCameraLocationZX(Blob triangleBlob, float cameraPhiPrime){

        if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"updateLocationZX begin");

        if (triangleBlob == null) return null;

        float xImg1, xImg2, x1, x2, z1, z2;

        //Estimate position of triangle corners on IMAGE using the bounding rectangle
        //Most accurate, as long as no blobs outside of the triangle and cryptobox
        //are included in the final triangle blob.
        xImg1 = (float)triangleBlob.getRectCenter().x - triangleBlob.getRectWidth()/2f;
        xImg2 = (float)triangleBlob.getRectCenter().x + triangleBlob.getRectWidth()/2f;

        //Estimate position of triangle corners on IMAGE using blob statistics
        //float triangleImgWidth = (float)Math.sqrt(12.0 * triangleBlob.getVarXX());
        //xImg1 = triangleBlob.getAvgX() - triangleImgWidth / 2.0f;
        //xImg2 = triangleBlob.getAvgX() + triangleImgWidth / 2.0f;

        //Actual coordinates on triangle corners on FIELD
        x1 = - TRIANGLE_WIDTH_CM / 2f;
        x2 = TRIANGLE_WIDTH_CM / 2f;
        z1 = - RAIL_DEPTH_CM;
        z2 = - RAIL_DEPTH_CM;

        float u1 = (xImg1 - principalX)/focalLength;
        float u2 = (xImg2 - principalX)/focalLength;
        float sin = (float)Math.sin(cameraPhiPrime);
        float cos = (float)Math.cos(cameraPhiPrime);
        float a1 = u1 * sin + cos;
        float b1 = u1 * cos - sin;
        float a2 = u2 * sin + cos;
        float b2 = u2 * cos - sin;
        float denom = a1 * b2 - a2 * b1;
        if (Math.abs(denom) < 0.0000001) return null;

        float zc = ( a1*a2*(x2-x1) + a1*b2*z2 - a2*b1*z1 ) / denom;
        float xc = -( a2*b1*x2 - a1*b2*x1 + b1*b2*(z2-z1) ) / denom;

        if (DEBUG_UPDATELOC) RobotLog.dd(TAG, "Loc: zc = %.1f xc = %.1f", zc, xc);

        return new float[] { zc, xc };

    }


}
