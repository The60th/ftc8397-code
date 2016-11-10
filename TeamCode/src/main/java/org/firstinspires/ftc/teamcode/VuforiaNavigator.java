package org.firstinspires.ftc.teamcode;

/**
 * Created by JimLori on 11/6/2016.
 */


import com.qualcomm.ftcrobotcontroller.R;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class VuforiaNavigator {

    private VuforiaLocalizer vuforia = null;
    public VuforiaTrackables targets = null;
    public String targetAssetName = null;
    public OpenGLMatrix[] targetLocations = null;
    public String[] targetNames = null;
    public OpenGLMatrix phoneLocationOnRobot = null;
    public VuforiaLocalizer.CameraDirection cameraDirection = null;

    public VuforiaNavigator(String assetName, OpenGLMatrix[] targetLocs, String[] targNames,
                            OpenGLMatrix phoneLocOnRobot, VuforiaLocalizer.CameraDirection camDirection){

        this.targetAssetName = assetName;
        this.targetLocations = targetLocs;
        this.targetNames = targNames;
        this.phoneLocationOnRobot = phoneLocOnRobot;
        this.cameraDirection = camDirection;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = cameraDirection;
        parameters.vuforiaLicenseKey = "Ae1jMu//////AAAAGeOTvbI/jknrpPvd1Jvqdk4+f9Csy0CC9PVlGo1o3pXA9ZN/v5SYrMDAzxObLuZ5pRWhZ9F6XZH5tkZcgPBMnb5pe9r1MgIPvxEC7IQ9mGWsnzGdGl5ABrOCR/wrog7l5YKJeMuRrlVRib9So+hurF1WchB8nlnuwQe3E7fiC+0M/wtz2qYRBxcH6dM7XKhPHNKpiyH5s82YXwIVMEwFtSQHQI68ghVvj3vnVVf5B0o4aUS8gn2/ygAa4uhEwUf3kmAiZO/kwGpdQGy3moOOdzBDyJ8KmGZIckcsRFGQdtQtgDVCoKxItGMCeH0vWyRmSp4dqLsUVnDQLWZfEP8axZ7UIWO+A8ppclFj7A1qehC6";
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        targets = vuforia.loadTrackablesFromAsset(targetAssetName);
        int numberOfTargets = targets.size();
        for (int i = 0; i< numberOfTargets; i++)
        {
            targets.get(i).setLocation(targetLocations[i]);
            targets.get(i).setName(targetNames[i]);
            ((VuforiaTrackableDefaultListener)targets.get(i).getListener()).setPhoneInformation(phoneLocationOnRobot, cameraDirection);
        }
    }

    public void activate(){
        targets.activate();
    }

    public OpenGLMatrix getTargetPoseRelativeToRobot(int targetIndex){
        OpenGLMatrix targetPoseRelativeToCamera =
                ((VuforiaTrackableDefaultListener)targets.get(targetIndex).getListener()).getPose();
        if (targetPoseRelativeToCamera == null) return null;
        return phoneLocationOnRobot.multiplied(targetPoseRelativeToCamera);
    }

    public OpenGLMatrix getRobotPoseRelativeToTarget(int targetIndex){
        OpenGLMatrix targetPoseRelativeToCamera =
                ((VuforiaTrackableDefaultListener)targets.get(targetIndex).getListener()).getPose();
        if (targetPoseRelativeToCamera == null) return null;
        return (phoneLocationOnRobot.multiplied(targetPoseRelativeToCamera)).inverted();
    }

    public OpenGLMatrix getRobotLocationOnField(int targetIndex){
        return ((VuforiaTrackableDefaultListener)targets.get(targetIndex).getListener()).getRobotLocation();
    }

    //return: X,Y position and Theta, the angle between the X axis of the coordinate system
    //being transformed to, and the projection of the robot Y-axis into the XY plane of the
    //coordinate system being transformed to.
    public static double[] getRobot_X_Y_Theta_FromLocationTransform(OpenGLMatrix locationTransform)
    {
        float[] locationData = locationTransform.getData();
        double[] returnValue = new double[3];
        returnValue[0] = locationData[12];
        returnValue[1] = locationData[13];
        returnValue[2] = Math.atan2( locationData[5], locationData[4]);
        return returnValue;
    }

    //return: Z,X position and Phi, the angle between the Z axis of the coordinate system
    //being transformed to and the projection of the robot Y-axis into the ZX plane of the
    //coordinate system being transformed to.
    public static double[] getRobot_Z_X_Phi_FromLocationTransform(OpenGLMatrix locationTransform)
    {
        float[] locationData = locationTransform.getData();
        double[] returnValue = new double[3];
        returnValue[0] = locationData[14];
        returnValue[1] = locationData[12];
        returnValue[2] = Math.atan2( locationData[4], locationData[6]);
        return returnValue;
    }

    public static double NormalizeAngle(double angle){
        double temp = (angle + Math.PI)/(2.0 * Math.PI);
        return (temp - Math.floor(temp) - 0.5) * 2.0 * Math.PI;
    }

}
