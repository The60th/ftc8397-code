package org.firstinspires.ftc.teamcode.cv_programs;

import android.graphics.Point;

import java.util.ArrayList;

/**
 * Created by JimLori on 7/4/2017.
 *
 * A Blob is a group of pixels that all satisfy a particular "condition", as specified in one of the findBlobs methods.
 *
 * One key property of a blob is its BOUNDING RECTANGLE. This is defined by the x value of its left-most pixel (left),
 * the y value of its top-most pixel (top), the x value of its right-most pixel (right), and the y value of its
 * bottom-most pixel (bottom).
 *
 * The two basic rules for constructing blobs in the findBlobs methods are:
 * 1. If a pixel satisfying "the condition" is located within or immediately adjacent to (i.e. touching)
 *    the BOUNDARY RECTANGLE of an existing blob, the pixel must be added to that blob.
 *
 * 2. If two existing blobs have BOUNDARY RECTANGLES that overlap or touch eachother, then those blobs
 *    should be combined into a single blob. The existing code performs a single pass to merge overlapping
 *    blobs; it isn't perfect, but is pretty good. More passes would ultimately achieve complete merging, but
 *    could take much more time.
 *
 * Note: these rules don't guarantee that all of the pixels in a blob will be contiguous.
 *
 * The other important properties of a blob are its STATISTICS. These consist of six SUMS that are
 * created by the findBlobs methods
 *
 *
 */

public class Blob {

    private int left;
    private int top;
    private int right;
    private int bottom;
    private long sumX;  //Sum of the x values of all pixels in the blob
    private long sumY;  //Sum of the y values of all pixels in the blob
    private long sumX2;  //Sum of the x-squared values of all pixels in the blob
    private long sumY2;  //Sum of the y-squared values of all pixels in the blob
    private long sumXY;  //Sum of the x*y values of all pixels in the blob

    //numPts is the total number of pixels included in the blob. This includes only the pixels that
    //satisfy the "condition", NOT necessarily all pixels within the bounding rectangle.
    private int numPts;

    /**
     * Using the statistics, it is possible to obtain certain information regarding the shape and orientation
     * of a blob. This information, when requested, will be calculated, stored privately in the orientation array,
     * and returned as individual properties using the getAngle, getLength, and getWidth methods.
     *
     * When used, the orientation array will contain the following:
     * [0] = angle of blob long axis relative to image x axis
     * [1] = Maximum variance; at this point used only for calculating length
     * [2] = Minimum variance; at this point used only for calculating width
     * [3] = approximate Length of the blob along its long axis (length of a rectangle having the same statistics)
     * [4] = approximate Width of the blob along its short axis (width of a rectangle having the same statistics)
     */
    private float[] orientation = null;

    /*
    PRIVATE constructor! Creates a new blob consisting of a single point.
    This is only to be called for blob construction in the findBlobs methods.
     */
    private Blob(int x, int y){
        left = x;
        top = y;
        right = x;
        bottom = y;
        sumX = x;
        sumX2 = x * x;
        sumY = y;
        sumY2 = y * y;
        sumXY = x * y;
        numPts = 1;
    }

    //Return center of the bounding rectangle of this Blob
    public Point getRectCenter(){
        return new Point((left+right)/2, (top+bottom)/2);
    }

    //Return area of the bounding rectangle of this Blob
    public int getRectArea(){
        return (bottom-top+1)*(right-left+1);
    }

    //Return width (x dimension) of the bounding rectangle of this Blob
    public int getRectWidth(){ return right-left+1; }

    //Return height (y dimension) of the bounding rectangle of this Blob
    public int getRectHeight(){ return bottom-top+1; }

    //Return x value of the "centroid" of this Blob
    public float getAvgX(){
        return (float)sumX / (float)numPts;
    }

    //Return y value of the "centroid" of this Blob
    public float getAvgY(){
        return (float)sumY / (float)numPts;
    }

    //Return variance of the x values in the Blob
    private float getVarXX(){
        return ( (float)sumX2 - (float)(sumX * sumX)/(float)numPts) / (float)numPts;
    }

    //Return variance of the y values in this Blob
    private float getVarYY(){
        return ( (float)sumY2 - (float)(sumY * sumY)/(float)numPts) / (float)numPts;
    }

    //Return number of points in this Blob
    public int getNumPts(){ return numPts; }

    //Return mean-radius-squared for the points in this blob (radius is distance of point from the centroid)
    public float getMeanR2(){
        return getVarXX() + getVarYY();
    }

    //Return the orientation angle of the blob (0 is parallel to the X axis, pi/2 parallel to the Y axis)
    public float getAngle(){
        if (orientation == null) orientation = getOrientation();
        return orientation[0];
    }

    //Return the rectangle-equivalent length of the blob
    public float getLength(){
        if (orientation == null) orientation = getOrientation();
        return orientation[3];
    }

    //Return the rectangle-equivalent width of the blob
    public float getWidth(){
        if (orientation == null) orientation = getOrientation();
        return orientation[4];
    }

    //Returns a float array containing:
    //theta: orientation angle in radians of the long axis of the blob relative to the x axis of the image
    //maxVar: mean-square of distances of blob points from blob center, in long-axis direction
    //minVar: mean-square of distances of blob points from blob center, in short-axis direction
    //length: equivalent length of a rectangle that has the same maxVar as this blob
    //width: equivalent width of a rectangle that has the same minVar as this blob
    private float[] getOrientation(){
        float sXX = (float)sumX2 - (float)(sumX * sumX)/(float)numPts;
        float sYY = (float)sumY2 - (float)(sumY * sumY)/(float)numPts;
        float sXY = (float)sumXY - (float)(sumX * sumY)/(float)numPts;
        float theta = 0.5f * (float)Math.atan2( 2.0 * sXY, sXX - sYY);
        float qq = 0.5f * (float)Math.sqrt( 4.0*sXY*sXY + (sXX-sYY)*(sXX-sYY));
        float ss = 0.5f * (sXX + sYY);
        float maxVar = (ss + qq)/(float)numPts;
        float minVar = (ss - qq)/(float)numPts;
        float length = (float)Math.sqrt(12.0 * maxVar);
        float width = (float)Math.sqrt(12.0 * minVar);
        return new float[] {theta, maxVar, minVar, length, width};
    }




    //This method is only used during construction of blobs by the findBlobs methods.
    //If point x,y is adjacent to, or contained within, this blob, modify left/right/top/bottom to
    //contain the point (if necessary), and return true.
    private boolean addPointIfWithinOrAdjacent(int x, int y){
        if (x >= (left-1) && x <= (right+1) && y >= (top-1) && y <= (bottom+1)){
            left = Math.min(left, x);
            right = Math.max(right, x);
            top = Math.min(top, y);
            bottom = Math.max(bottom, y);
            sumX += x;
            sumY += y;
            sumX2 += x * x;
            sumY2 += y * y;
            sumXY += x * y;
            numPts++;
            return true;
        }
        else return false;
    }


    //This method is only used during construction of blobs by the findBlobs methods.
    //If there is any overlap or adjacency between this blob and other blob, modify left,top,right,and bottom
    //of this blob so it includes both original blobs (if necessary), and return true.
    private boolean joinIfOverlappedOrAdjacent(Blob other){
        if (top <= (other.bottom+1) && bottom >= (other.top-1) && left <= (other.right+1) && right >= (other.left-1)){
            left = Math.min(left, other.left);
            right = Math.max(right, other.right);
            top = Math.min(top, other.top);
            bottom = Math.max(bottom, other.bottom);
            sumX += other.sumX;
            sumY += other.sumY;
            sumX2 += other.sumX2;
            sumY2 += other.sumY2;
            sumXY += other.sumXY;
            numPts += other.numPts;
            return true;
        }
        else return false;
    }


    //This merges the points in the other blob with this blob. This could be helpful to the end user.
    //For example, it might be useful to merge blobs that are close together, or share similar avgX
    //values, etc.
    public void merge(Blob other){
        left = Math.min(left, other.left);
        right = Math.max(right, other.right);
        top = Math.min(top, other.top);
        bottom = Math.max(bottom, other.bottom);
        sumX += other.sumX;
        sumY += other.sumY;
        sumX2 += other.sumX2;
        sumY2 += other.sumY2;
        sumXY += other.sumXY;
        numPts += other.numPts;
        orientation = null;
    }



    /*
    findBlobs
    Takes a binary source image (src), with width and height passed in
    Returns an ArrayList of Blobs detected in the image. The source image is an int[] containing only
    values of 0 or 1.
     */
    public static ArrayList<Blob> findBlobs(int[] src, int width, int height){
        ArrayList<Blob> blobs = new ArrayList<Blob>(); //empty arraylist; this will hold the blobs as they are created.

        //Initial search for points that match target criteria (i.e. have value of 1). For each such point, either combine with
        //existing blob (if contained or adjacent), or create a new blob.
        for (int y = 0; y < height; y++){
            int rowIndex = y * width;
            for (int x = 0; x < width; x++){
                int index = rowIndex + x;

                //If the current pixel is 0, then it won't be added to any blob. Continue to next pixel.
                if (src[index] == 0) continue;

                boolean containedByBlob = false;

                //iterate through all existing blobs. If the current pixel is within or adjacent to an existing blob,
                //add it to that blob.
                for (int i = 0; i < blobs.size(); i++){
                    Blob b = blobs.get(i);
                    if (b.addPointIfWithinOrAdjacent(x,y)){
                        containedByBlob = true;
                        break;
                    }
                }

                //If the current pixel was not added to an existing blob, then create a new blob containing only that pixel.
                if (!containedByBlob){
                    blobs.add(new Blob(x, y));
                }
            }
        }

        //Iterate through all blobs, find blobs that overlap, and condense so that the
        //final list has no overlapping blobs. NOTE: one run through this block isn't always
        //enough to merge all blobs with overlapping boundary rectangles, but its pretty good.
        //If needed, this block could be repeated until no more overlaps are found, at the expense of computation time.
        for (int i = blobs.size()-1; i > 0; i--){
            Blob b1 = blobs.get(i);
            boolean merged = false;
            for (int j = i-1; j >= 0; j--){
                Blob b2 = blobs.get(j);
                if (b2.joinIfOverlappedOrAdjacent(b1)){
                    blobs.remove(i);
                    merged = true;
                    break;
                }
            }
            if (merged) continue;
        }

        return blobs;

    }


    /*
    This is an alternate findBlobs method that could be helpful in noise reduction, when used with image processing.
     */

//    public static ArrayList<Blob> findBlobs(float[] src, int width, int height, float threshhold){
//        ArrayList<Blob> blobs = new ArrayList<Blob>();
//
//        //Initial search for points that meet threshhold. For each such point, either combine with
//        //existing blob (if contained or adjacent), or create a new blob.
//        for (int y = 0; y < height; y++){
//            int rowIndex = y * width;
//            for (int x = 0; x < width; x++){
//                int index = rowIndex + x;
//                if (src[index] < threshhold) continue;
//                boolean containedByBlob = false;
//                for (int i = 0; i < blobs.size(); i++){
//                    Blob b = blobs.get(i);
//                    if (b.addPointIfWithinOrAdjacent(x,y)){
//                        containedByBlob = true;
//                        break;
//                    }
//                }
//                if (!containedByBlob){
//                    blobs.add(new Blob(x, y));
//                }
//            }
//        }
//
//        //Iterate through all blobs, find blobs that overlap, and condense so that the
//        //final list has no overlapping blobs.
//        for (int i = blobs.size()-1; i > 0; i--){
//            Blob b1 = blobs.get(i);
//            for (int j = i-1; j >= 0; j--){
//                Blob b2 = blobs.get(j);
//                if (b2.joinIfOverlappedOrAdjacent(b1)){
//                    blobs.remove(i);
//                    break;
//                }
//            }
//        }
//
//        return blobs;
//
//    }


}
