package org.firstinspires.ftc.teamcode.cv_programs;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by JimLori on 4/8/2018.
 */

public class CryptoNavReverse extends CryptoNavNew {

    //Need to swith Right for Left in initializeRails for reversed camera orientation
    public static boolean initializeRails(byte[] rawImageBytes, Side side){
        return side == Side.LEFT? CryptoNavNew.initializeRails(rawImageBytes, Side.RIGHT) :
                CryptoNavNew.initializeRails(rawImageBytes, Side.LEFT);

    }


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

        //Code from the method in CryptoNav is commented out:
        //float u1Center = (rails[index1].getAvgX() - principalX)/focalLength;
        //float u2Center = (rails[index2].getAvgX() - principalX)/focalLength;

        //and replaced with:
        float u1Center = -(rails[index1].getAvgX() - principalX)/focalLength;
        float u2Center = -(rails[index2].getAvgX() - principalX)/focalLength;

        if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"uCenter: 1 = %.3f  2 = %.3f", u1Center, u2Center);

        float tan = (float)Math.tan(phiPrime);

        float xImg1, xImg2, x1, x2, z1, z2;

//      Code from the CryptoNav method is commented out:
//        if (u1Center < tan) {
//            //Camera center is to the right of the rail1; use right rail edge
//            xImg1 = rails[index1].getAvgX() + rails[index1].getWidth() / 2.0f;
//            x1 = ((float) index1 - 1.5f) * RAIL_SPACING_CM + 0.5f * RAIL_THICKNESS_CM;
//            if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"Right edge: xImg1 = %.1f x1 = %.1f", xImg1, x1);
//        }
//        else{
//            //Camera center is to the left of the rail1; use left rail edge
//            xImg1 = rails[index1].getAvgX() - rails[index1].getWidth()/2.0f;
//            x1 = ((float)index1 - 1.5f) * RAIL_SPACING_CM - 0.5f * RAIL_THICKNESS_CM;
//            if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"Left edge: xImg1 = %.1f x1 = %.1f", xImg1, x1);
//        }
//
//        if (u2Center < tan){
//            //Camera center is to the left of the rail2; use right rail edge
//            xImg2 = rails[index2].getAvgX() + rails[index2].getWidth()/2.0f;
//            x2 = ((float)index2 - 1.5f) * RAIL_SPACING_CM + 0.5f * RAIL_THICKNESS_CM;
//            if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"Right edge: xImg2 = %.1f x2 = %.1f", xImg2, x2);
//        }
//        else{
//            //Camera center is to the right of the rail2; use left rail edge
//            xImg2 = rails[index2].getAvgX() - rails[index2].getWidth()/2.0f;
//            x2 = ((float)index2 - 1.5f) * RAIL_SPACING_CM - 0.5f * RAIL_THICKNESS_CM;
//            if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"Left edge: xImg2 = %.1f x2 = %.1f", xImg1, x1);
//        }


        //and replaced with:

        if (u1Center < tan) {
            //Camera center is to the right of the rail1; use right rail edge
            xImg1 = rails[index1].getAvgX() - rails[index1].getWidth() / 2.0f;
            x1 = -((float) index1 - 1.5f) * RAIL_SPACING_CM + 0.5f * RAIL_THICKNESS_CM;
            if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"Right edge: xImg1 = %.1f x1 = %.1f", xImg1, x1);
        }
        else{
            //Camera center is to the left of the rail1; use left rail edge
            xImg1 = rails[index1].getAvgX() + rails[index1].getWidth()/2.0f;
            x1 = -((float)index1 - 1.5f) * RAIL_SPACING_CM - 0.5f * RAIL_THICKNESS_CM;
            if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"Left edge: xImg1 = %.1f x1 = %.1f", xImg1, x1);
        }

        if (u2Center < tan){
            //Camera center is to the left of the rail2; use right rail edge
            xImg2 = rails[index2].getAvgX() - rails[index2].getWidth()/2.0f;
            x2 = -((float)index2 - 1.5f) * RAIL_SPACING_CM + 0.5f * RAIL_THICKNESS_CM;
            if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"Right edge: xImg2 = %.1f x2 = %.1f", xImg2, x2);
        }
        else{
            //Camera center is to the right of the rail2; use left rail edge
            xImg2 = rails[index2].getAvgX() + rails[index2].getWidth()/2.0f;
            x2 = -((float)index2 - 1.5f) * RAIL_SPACING_CM - 0.5f * RAIL_THICKNESS_CM;
            if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"Left edge: xImg2 = %.1f x2 = %.1f", xImg1, x1);
        }


        z1 = -RAIL_DEPTH_CM;
        z2 = -RAIL_DEPTH_CM;

        if (DEBUG_UPDATELOC) RobotLog.dd(TAG,"z1 = %.1f z2 = %.1f", z1, z2);

        float u1 = -(xImg1 - principalX)/focalLength;
        float u2 = -(xImg2 - principalX)/focalLength;
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
