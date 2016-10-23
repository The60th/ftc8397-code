package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

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

public class VuforiaNav  {
    VuforiaLocalizer vuforia = null;
    VuforiaTrackables trackables= null;
    OpenGLMatrix phoneLocationOnRobot = null;

    public VuforiaNav(){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "INSERT YOUR LICENSE KEY HERE!";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        trackables = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
        OpenGLMatrix[] targetLocationsOnField = new OpenGLMatrix[4];


        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));


         targetLocationsOnField[0] = OpenGLMatrix.translation(0, 1000, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        trackables.get(0).setLocation(targetLocationsOnField[0]);


        targetLocationsOnField[1] = OpenGLMatrix.translation(0, 1000, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        trackables.get(1).setLocation(targetLocationsOnField[1]);


        targetLocationsOnField[2] = OpenGLMatrix.translation(0, 1000, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        trackables.get(2).setLocation(targetLocationsOnField[2]);


        targetLocationsOnField[3] = OpenGLMatrix.translation(0, 1000, 0)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        trackables.get(3).setLocation(targetLocationsOnField[3]);

        for(int i =0; i <4; i++){
            ((VuforiaTrackableDefaultListener)trackables.get(i).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

    }
    public void activate(){
        trackables.activate();
    }


    public OpenGLMatrix[] getRobotGlMatrixArray(){
        OpenGLMatrix[] robotGlMatrixReturn = new OpenGLMatrix[4];
        for(int i =0; i<4;i++){
            robotGlMatrixReturn[i] = ((VuforiaTrackableDefaultListener) trackables.get(i).getListener()).getUpdatedRobotLocation();

        }
        return robotGlMatrixReturn;
    }
    public Pose getPoseFromTransform(OpenGLMatrix robotLocationTransform){
        if(robotLocationTransform != null) {
            float[] locationArray = robotLocationTransform.getData();
            float x = locationArray[12];
            float y = locationArray[13];
            float theta = (float) Math.atan2(locationArray[5], locationArray[4]) * (180.0f) / (float) Math.PI;
            return new Pose(x,y,theta);
        }
        else{
            return null;
        }
    }

            //telemetry.update();
           // idle();
        }

   // }





