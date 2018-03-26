package org.firstinspires.ftc.teamcode.cv_programs;

import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

import java.util.ArrayList;

/**
 * Created by JimLori on 9/14/2017.
 * <p>
 * Static class containing data and methods for using the Cryptobox and Blob detection for navigation.
 */

public class CryptoNav {

    //Control of debug logging.

    private static final String TAG = "CryptoNav";
    private static final boolean DEBUG_INIT = true;
    private static final boolean DEBUG_GETBLOBS = true;
    private static final boolean DEBUG_INITRAILS = true;
    private static final boolean DEBUG_FILTER = true;
    private static final boolean DEBUG_UPDATE = true;
    private static final boolean DEBUG_UPDATELOC = true;

    //Dimensions of the CryptoBox in cm

    private static final float RAIL_THICKNESS_CM = 0.75f * 2.54f;
    private static final float RAIL_DEPTH_CM = 4.0f * 2.54f;
    private static final float RAIL_SPACING_CM = 7.63f * 2.54f;

    //HSV Ranges for Red and Blue Rails

    private static final HSV_Range RED_RANGE = new HSV_Range(345, 15, 0.5f, 1.0f, 0.3f, 1.0f);
    private static final HSV_Range BLUE_RANGE = new HSV_Range(210, 270, 0.5f, 1.0f, 0.1f, 1.0f);

    //Maximum allowed fraction of inter-rail distance that robot is allowed to move between cycles.

    private static final float MAX_RAIL_MOVEMENT_FRACTION = 0.25f;

    //TeamColor enum -- self explanatory

    public enum TeamColor {RED, BLUE}

    //Side enum used for rail initialization. LEFT means that the left-most rail (Rail 0) must be
    //visible at the time that initializeRails is called. RIGHT means that the right-most rail (Rail 3)
    //must be visible at the time that initializeRails is called.

    public enum Side {LEFT, RIGHT}

    //Team Color

    private static TeamColor teamColor;

    //Array of blob to hold blobs representing the four cryptobox rails (from left to right)
    //If a given element is null, that means the corresponding rail is not currently visible

    private static Blob[] rails = new Blob[4];

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

    //lastVisibleLeftRailIndex, the index of the last visible left rail in rails.
    private static int lastVisibleLeftRailIndex = -1;

    //lastVisibleLeftRail, a reference to the last visible left rail.
    private static Blob lastVisibleLeftRail = null;

    //lastInterRailDistance, the known distance, in pixels, between two adjacent rails
    private static float lastInterRailDistance = (float)reducedImageWidth / 16f;

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

    public static ArrayList<Blob> getBlobsFromRawImage(byte[] rawImageBytes){
        if (DEBUG_GETBLOBS) RobotLog.dd(TAG,"Begin getBlobsFromRawImage");
        HSV_Range range = teamColor == TeamColor.RED? RED_RANGE : BLUE_RANGE;
        ImgProc.getReducedRangeRGB565(rawImageBytes, rawImageWidth, rawImageHeight, rawRangeX0, rawRangeY0,
                rawRangeWidth, rawRangeHeight, reducedImageBytes, sampleRatio);
        ImgProc.getBinaryImage(reducedImageBytes, range, binaryImage);
        ArrayList<Blob> blobs = Blob.findBlobs(binaryImage, reducedImageWidth, reducedImageHeight);
        if (DEBUG_GETBLOBS) {
            RobotLog.dd(TAG,"Num Blobs = %d", blobs.size());
            RobotLog.dd(TAG,"End getBlobsFromRawImage");
        }
        return blobs;
    }

    //Initialize blob array rails; if return value is false, initialization has failed, and crypto-navigation will not be possible.
    //lastInterRailDistance also initialized, if there are at least two visible rails.

    public static boolean initializeRails(byte[] rawImageBytes, Side startSide) {
        if (DEBUG_INITRAILS) RobotLog.dd(TAG,"Begin initializeRails");
        ArrayList<Blob> railList = getBlobsFromRawImage(rawImageBytes);
        if (!filterRailBlobs(railList)) {
            if (DEBUG_INITRAILS) {
                RobotLog.dd(TAG,"filterRailBlobs called from initializeRails and failed");
                RobotLog.dd(TAG,"End initializeRails");
            }
            return false;
        }
        if (DEBUG_INITRAILS) RobotLog.dd(TAG,"railList.size() = %d", railList.size());
        if (railList.size() == 0 || railList.size() > 4) {
            if (DEBUG_INITRAILS) RobotLog.dd(TAG,"End initializeRails");
            return false;
        }
        if (startSide == Side.LEFT) {
            for (int i = 0; i < railList.size(); i++) rails[i] = railList.get(i);
            for (int i = railList.size(); i < 4; i++) rails[i] = null;
            lastVisibleLeftRailIndex = 0;
            if (rails[1] != null) lastInterRailDistance = rails[1].getAvgX() - rails[0].getAvgX();
        } else {
            for (int i = 0; i < railList.size(); i++)
                rails[3 - i] = railList.get(railList.size() - 1 - i);
            for (int i = railList.size(); i < 4; i++) rails[3 - i] = null;
            if (rails[2] != null) lastInterRailDistance = rails[3].getAvgX() - rails[2].getAvgX();
            lastVisibleLeftRailIndex = 3 - railList.size() + 1;
        }
        lastVisibleLeftRail = railList.get(0);
        if (DEBUG_INITRAILS){
            for (int i = 0; i < 4; i++) {
                if (rails[i] == null) RobotLog.dd(TAG, "rails[%d] = null", i);
                else RobotLog.dd(TAG, "rails[%d]: x = %.1f w = %.1f", i, rails[i].getAvgX(), rails[i].getWidth());
            }
            RobotLog.dd(TAG,"lastVisibleLeftRailIndex = %d", lastVisibleLeftRailIndex);
            RobotLog.dd(TAG,"End initializeRails");
        }
        return true;
    }


    /*
    Filters an array of blobs, leaving only a list of, at most, 4 blobs that represent the cryptobox rails.
    Returns true if successful, false if unsuccessful.
    */
    public static boolean filterRailBlobs(ArrayList<Blob> blobs) {
        if (DEBUG_FILTER) {
            RobotLog.dd(TAG, "filterRailBlobs begin");
            for (int i = 0; i < blobs.size(); i++)
                RobotLog.dd(TAG, "blobs[%d] x = %.1f", i, blobs.get(i).getAvgX());
        }
        if (blobs.size() == 0) {
            if (DEBUG_FILTER) {
                RobotLog.dd(TAG,"Initial blobs.size() = 0; returning false.");
                RobotLog.dd(TAG,"End filterRailBlobs");
            }
            return false;
        }
        //Select blobs with correct size and shape
        for (int i = blobs.size() - 1; i >= 0; i--) {
            Blob b = blobs.get(i);
            if (b.getNumPts() < 100 || (b.getLength() > 2.0f * b.getWidth() && Math.abs(b.getAngle()) < Math.PI/3.0))
                blobs.remove(i);
        }
        if (blobs.size() == 0) {
            if (DEBUG_FILTER) {
                RobotLog.dd(TAG,"blobs.size() = 0 after size,orientation,shape filter");
                RobotLog.dd(TAG,"End filterRailBlobs");
            }
            return false;
        }

        //Sort by increasing avgX
        for (int i = 1; i < blobs.size(); i++)
            for (int j = i; j > 0; j--)
                if (blobs.get(j).getAvgX() < blobs.get(j - 1).getAvgX()) {
                    Blob temp = blobs.get(j - 1);
                    blobs.remove(j - 1);
                    blobs.add(j, temp);
                }

        //Coalesce blobs with  X overlap
        int index = 0;
        while (index < blobs.size()-1){
            if (Math.abs(blobs.get(index).getAvgX() - blobs.get(index+1).getAvgX())
                    < 0.4 * (blobs.get(index).getWidth() + blobs.get(index+1).getWidth())){
                blobs.get(index).merge(blobs.get(index+1));
                blobs.remove(index+1);
            }
            else index++;
        }

        if (DEBUG_FILTER){
            RobotLog.dd(TAG,"%d blobs remain", blobs.size());
            for (int i = 0; i < blobs.size(); i++)
                RobotLog.dd(TAG,"blobs[%d]: x = %.1f w = %.1f", i, blobs.get(i).getAvgX(),
                        blobs.get(i).getWidth());
        }

        if (blobs.size() > 4) return false;

        return true;
    }

    /*
    Assign the sorted, filtered blobs properly to the rails array.
    Argument rawImageBytes is the new raw RGB565 image.
    Tasks accomplished in this method:
      Determine the appropriate index of the new leftMost visible rail (blobs.get(0)).
      Update lastVisibleLeftRail, lastVisibleLeftRailIndex, and lastInterRailDistance.
      Fill in the rails array with new rails from the blobs list, padding with nulls.
    If there is a failure at any point, return false, leaving lastVisibleLeftRail, lastVisibleLeftRailIndex,
    and lastInterRailDistance unchanged.
    */
    public static boolean updateRails(byte[] rawImageBytes) {

        if (DEBUG_UPDATE) RobotLog.dd(TAG, "updateRails begin");

        ArrayList<Blob> blobs = getBlobsFromRawImage(rawImageBytes);

        if (!filterRailBlobs(blobs)) {
            if (DEBUG_UPDATE) {
                RobotLog.dd(TAG, "filterRailBlobs failed");
                RobotLog.dd(TAG, "End updateRails");
            }
            return false;
        }

        //If lastVisibleLeftRail is null, then initialization either failed or was never attempted, and navigation isn't possible.

        if (lastVisibleLeftRail == null) {
            if (DEBUG_UPDATE){
                RobotLog.dd(TAG,"lastVisibleLeftRail is null");
                RobotLog.dd(TAG,"End updateRails");
            }
            return false;
        }

        //If blobs is empty, set all items in rails array to null and return false.
        //But, leave lastVisibleLeftRail and lastInterRailDistance alone.
        //Navigation won't be possible until a rail reappears.

        if (DEBUG_UPDATE) RobotLog.dd(TAG, "blobs.size() = %d", blobs.size());

        if (blobs.size() == 0){
            for (int i = 0; i < 4; i++) rails[i] = null;
            if (DEBUG_UPDATE) RobotLog.dd(TAG, "End updateRails");
            return false;
        }

        //Determine current interRailDistance

        float interRailDistance =  blobs.size() > 1? blobs.get(1).getAvgX() - blobs.get(0).getAvgX() : lastInterRailDistance;

        float threshholdDistance = MAX_RAIL_MOVEMENT_FRACTION * interRailDistance;

        //Determine the current visible left rail index

        int visibleLeftRailIndex = lastVisibleLeftRailIndex;

        //If the leftMost visible rail appears to have moved too far to the left, then we assume in reality that it has moved
        //to the right, and an additional rail has appeared on the left. If that would result in a visibleLeftRailIndex < 0,
        //then we've failed.

        if (blobs.get(0).getAvgX() < lastVisibleLeftRail.getAvgX() - threshholdDistance) {
            if (visibleLeftRailIndex == 0) {
                if (DEBUG_UPDATE){
                    RobotLog.dd(TAG,"Update of visibleLeftRailIndex failed at 0");
                    RobotLog.dd(TAG,"End updateRails");
                }
                return false;
            }
            else visibleLeftRailIndex--;
        }

        //else, if the leftMost visible rail appears to have moved too far to the right, then we assume in reality that it has disappeared
        //to the left, and the rail to the right of it is the new leftMost visible rail. If that would result in a visibleLeftRailIndex > 3,
        //then we've failed.

        else if (blobs.get(0).getAvgX() > lastVisibleLeftRail.getAvgX() + threshholdDistance) {
            if (visibleLeftRailIndex == 3) {
                if (DEBUG_UPDATE){
                    RobotLog.dd(TAG,"Update of visibleLeftRailIndex failed at 3");
                    RobotLog.dd(TAG,"End updateRails");
                }
                return false;
            }
            else visibleLeftRailIndex++;
        }

        //If the combination of the new visibleLeftRailIndex and the number of blobs in the list would result in a total of more
        //than four rails, then we've failed.

        if (visibleLeftRailIndex + blobs.size() > 4) {
            if (DEBUG_UPDATE) {
                RobotLog.dd(TAG, "New visibleLeftRailIndex and blobs.size() incompatible");
                RobotLog.dd(TAG, "End updateRails");
            }
            return false;
        }

        //We've succeeded in finding the leftMost visible rail and assigning it an index.

        //Update the lastVisibleLeftRail, lastVisibleLeftRailIndex, and lastInterRailDistance

        lastVisibleLeftRail = blobs.get(0);
        lastVisibleLeftRailIndex = visibleLeftRailIndex;
        lastInterRailDistance = interRailDistance;

        if (DEBUG_UPDATE){
            RobotLog.dd(TAG,"visibleLeftRailIndex = %d",visibleLeftRailIndex);
            RobotLog.dd(TAG,"interRailDistance = %.1f", interRailDistance);
        }

        //Fill in the rails array with values from the blobs list, padding with nulls.

        for (int i = 0; i < visibleLeftRailIndex; i++) rails[i] = null;
        for (int i = visibleLeftRailIndex; i < visibleLeftRailIndex + blobs.size(); i++) rails[i] = blobs.get(i - visibleLeftRailIndex);
        for (int i = visibleLeftRailIndex + blobs.size(); i < 4; i++) rails[i] = null;

        if (DEBUG_UPDATE) {
            for (int i = 0; i < 4; i++) {
                if (rails[i] == null) RobotLog.dd(TAG, "rails[%d]: null", i);
                else RobotLog.dd(TAG, "rails[%d]: x = %.1f", i, rails[i].getAvgX());
            }
            RobotLog.dd(TAG,"End updateRails");
        }

        return true;

    }

   /*
    Attempts to update rails. If successful, and if at least two rails are visible, uses the location
    of two furthest-apart rails, and phiPrime (passed in as argument) to determine the location of the
    camera (zc, xc) relative to the CryptoBox. Note that zc=0 cm, xc=0 cm is front,center of cryptobox.
    Positive xc means camera has moved to right of center. Positive zc means that camera has moved away
    from CryptoBox.

    The coordinate system used here is like the coordinate system for a Vuforia target. Note that with this
    method, we need to pass in the phiPrime value (obtained from gyro).
    */

    public static float[] updateLocationZX(byte[] rawImageBytes, float phiPrime){

        if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"updateLocationZX begin");

        if (!updateRails(rawImageBytes)) {
            if (DEBUG_UPDATELOC){
                RobotLog.dd(TAG,"updateRails failed");
                RobotLog.dd(TAG,"End updateLocationZX");
            }
            return null;
        }

        int index1 = -1;
        for (int i = 0; i < 4; i++)
            if (rails[i] != null) {
                index1 = i;
                break;
            }


        if (DEBUG_UPDATELOC) RobotLog.dd(TAG, "index1 = %d", index1);
        if (index1 < 0 || index1 == 3)
        {
            if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"End updateLocationZX");
            return null;
        }

        int index2 = -1;
        for (int i = index1 + 1; i < 4; i++) if (rails[i] != null) index2 = i;

        if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"index2 = %d",index2);
        if (index2 < 0) {
            if (DEBUG_UPDATELOC) RobotLog.dd(TAG, "End updateLocationZX");
            return null;
        }

        float u1Center = (rails[index1].getAvgX() - principalX)/focalLength;
        float u2Center = (rails[index2].getAvgX() - principalX)/focalLength;

        if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"uCenter: 1 = %.3f  2 = %.3f", u1Center, u2Center);

        float tan = (float)Math.tan(phiPrime);

        float xImg1, xImg2, x1, x2, z1, z2;

        if (u1Center < tan){
            //Camera center is to the right of the rail1; use right rail edge
            xImg1 = rails[index1].getAvgX() + rails[index1].getWidth()/2.0f;
            x1 = ((float)index1 - 1.5f) * RAIL_SPACING_CM + 0.5f * RAIL_THICKNESS_CM;
            if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"Right edge: xImg1 = %.1f x1 = %.1f", xImg1, x1);
        }
        else{
            //Camera center is to the left of the rail1; use left rail edge
            xImg1 = rails[index1].getAvgX() - rails[index1].getWidth()/2.0f;
            x1 = ((float)index1 - 1.5f) * RAIL_SPACING_CM - 0.5f * RAIL_THICKNESS_CM;
            if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"Left edge: xImg1 = %.1f x1 = %.1f", xImg1, x1);
        }

        if (u2Center < tan){
            //Camera center is to the left of the rail2; use right rail edge
            xImg2 = rails[index2].getAvgX() + rails[index2].getWidth()/2.0f;
            x2 = ((float)index2 - 1.5f) * RAIL_SPACING_CM + 0.5f * RAIL_THICKNESS_CM;
            if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"Right edge: xImg2 = %.1f x2 = %.1f", xImg2, x2);
        }
        else{
            //Camera center is to the right of the rail2; use left rail edge
            xImg2 = rails[index2].getAvgX() - rails[index2].getWidth()/2.0f;
            x2 = ((float)index2 - 1.5f) * RAIL_SPACING_CM - 0.5f * RAIL_THICKNESS_CM;
            if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"Left edge: xImg2 = %.1f x2 = %.1f", xImg1, x1);
        }

        z1 = -RAIL_DEPTH_CM;
        z2 = -RAIL_DEPTH_CM;

        if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"z1 = %.1f z2 = %.1f", z1, z2);

        float u1 = (xImg1 - principalX)/focalLength;
        float u2 = (xImg2 - principalX)/focalLength;
        float sin = (float)Math.sin(phiPrime);
        float cos = (float)Math.cos(phiPrime);
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
