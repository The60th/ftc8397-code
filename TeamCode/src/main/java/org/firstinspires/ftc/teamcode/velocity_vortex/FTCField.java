package org.firstinspires.ftc.teamcode;

/**
 * Created by JimLori on 11/6/2016.
 */

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FTCField {

    //Field Width in mm
    public static final float WIDTH = 3580;

    //Height of target image center above floor, in mm
    public static final float TARGET_CENTER_HEIGHT = (1.5f + 8.5f/2.0f) * 25.4f;

    //Array containing the positions of the navigation targets on the field, distances in mm

    public static final OpenGLMatrix[] TARGET_LOCATIONS = new OpenGLMatrix[]{

            //Wheels:
            OpenGLMatrix.translation(302, WIDTH/2, TARGET_CENTER_HEIGHT)
            .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,
                    AxesOrder.XZX,AngleUnit.DEGREES,90,0,0)),

            //Tools
            OpenGLMatrix.translation(-WIDTH/2, 905, TARGET_CENTER_HEIGHT)
                    .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,
                    AxesOrder.XZX,AngleUnit.DEGREES,90,90,0)),

            //Legos
            OpenGLMatrix.translation(-905, WIDTH/2, TARGET_CENTER_HEIGHT)
                    .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,
                    AxesOrder.XZX,AngleUnit.DEGREES,90,0,0)),

            //Gears
            OpenGLMatrix.translation(-WIDTH/2, -302, TARGET_CENTER_HEIGHT)
                    .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC,
                    AxesOrder.XZX,AngleUnit.DEGREES,90,90,0))
    };

    public static final String[] TARGET_NAMES = new String[]
            {"Wheels", "Tools", "Legos", "Gears"};


}
