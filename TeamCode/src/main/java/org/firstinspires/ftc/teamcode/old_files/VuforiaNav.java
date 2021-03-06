//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//
//public class VuforiaNav  {
//    VuforiaLocalizer vuforia = null;
//    VuforiaTrackables trackables= null;
//    OpenGLMatrix phoneLocationOnRobot = null;
//
//    public VuforiaNav(){
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
//        parameters.vuforiaLicenseKey = "Ae1jMu//////AAAAGeOTvbI/jknrpPvd1Jvqdk4+f9Csy0CC9PVlGo1o3pXA9ZN/v5SYrMDAzxObLuZ5pRWhZ9F6XZH5tkZcgPBMnb5pe9r1MgIPvxEC7IQ9mGWsnzGdGl5ABrOCR/wrog7l5YKJeMuRrlVRib9So+hurF1WchB8nlnuwQe3E7fiC+0M/wtz2qYRBxcH6dM7XKhPHNKpiyH5s82YXwIVMEwFtSQHQI68ghVvj3vnVVf5B0o4aUS8gn2/ygAa4uhEwUf3kmAiZO/kwGpdQGy3moOOdzBDyJ8KmGZIckcsRFGQdtQtgDVCoKxItGMCeH0vWyRmSp4dqLsUVnDQLWZfEP8axZ7UIWO+A8ppclFj7A1qehC6";
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//        trackables = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
//        OpenGLMatrix[] targetLocationsOnField = new OpenGLMatrix[4];
//
//
//        phoneLocationOnRobot = OpenGLMatrix
//                .translation(41,0,0) //Changed from 50 to -56 to test new files. -56 was over by about 1cm, changing to -41bit
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 0, 0));
//
//
//         targetLocationsOnField[0] = OpenGLMatrix.translation(-1790, -300, 90)
//                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 90, 0));
//        trackables.get(0).setLocation(targetLocationsOnField[0]);
//        trackables.get(0).setName("Wheels");
//
//
//        targetLocationsOnField[1] = OpenGLMatrix.translation(-900, 1790, 90)
//                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 0, 0));
//        trackables.get(1).setLocation(targetLocationsOnField[1]);
//        trackables.get(1).setName("Tools");
//
//
//        targetLocationsOnField[2] = OpenGLMatrix.translation(-1790, 900, 90)
//                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 90, 0));
//        trackables.get(2).setLocation(targetLocationsOnField[2]);
//        trackables.get(2).setName("Legos");
//
//
//        targetLocationsOnField[3] = OpenGLMatrix.translation(300, 1790, 90)
//                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 0, 0));
//        trackables.get(3).setLocation(targetLocationsOnField[3]);
//        trackables.get(3).setName("Gears");
//
//        for(int i =0; i <4; i++){
//            ((VuforiaTrackableDefaultListener)trackables.get(i).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//        }
//
//    }
//    public void activate(){
//        trackables.activate();
//    }
//
//
//    public OpenGLMatrix[] getRobotGlMatrixArray(){
//        OpenGLMatrix[] robotGlMatrixReturn = new OpenGLMatrix[4];
//        for(int i =0; i<4;i++){
//            robotGlMatrixReturn[i] = ((VuforiaTrackableDefaultListener) trackables.get(i).getListener()).getUpdatedRobotLocation();
//
//        }
//        return robotGlMatrixReturn;
//    }
//    public Pose getPoseFromTransform(OpenGLMatrix robotLocationTransform){
//        if(robotLocationTransform != null) {
//            float[] locationArray = robotLocationTransform.getData();
//            float x = locationArray[12];
//            float y = locationArray[13];
//            float theta = (float) Math.atan2(locationArray[5], locationArray[4]) * (180.0f) / (float) Math.PI;
//            return new Pose(x,y,theta);
//        }
//        else{
//            return null;
//        }
//    }
//
//    public OpenGLMatrix getTargetLocation(int targetNumber){
//        OpenGLMatrix relativeToCamera = ((VuforiaTrackableDefaultListener)trackables.get(targetNumber).getListener()).getPose();
//        if(relativeToCamera == null) return null;
//        return  phoneLocationOnRobot.multiplied(relativeToCamera);
//    }
//
//    public OpenGLMatrix getRobotLocationRelativeToTarget(int targetNumber){
//        OpenGLMatrix relativeToCamera = ((VuforiaTrackableDefaultListener)trackables.get(targetNumber).getListener()).getPose();
//        if(relativeToCamera == null) return null;
//        OpenGLMatrix relativeToRobot = phoneLocationOnRobot.multiplied(relativeToCamera);
//        return relativeToRobot.inverted();
//    }
//    public boolean isTargetVisible(int targetNumber){
//       return ((VuforiaTrackableDefaultListener)trackables.get(targetNumber).getListener()).isVisible();
//    }
//
//    public static float[] GetZXPH(OpenGLMatrix relativeToTarget){
//        float[] zxPhi = relativeToTarget.getData();
//        float z = zxPhi[14]/10;
//        float x = zxPhi[12]/10;
//        float robotX = zxPhi[4];
//        float robotZ = zxPhi[6];
//        float phi = (float)Math.atan2(robotX,robotZ);
//        return new float[]{z,x,phi};
//    }
//    public static float remapAngle(float angle){
//        float temp = (float)((angle+Math.PI)/(2* Math.PI));
//        return (float)(temp -Math.floor(temp)-.5)*((float)(2*Math.PI));
//    }
//}
//
//
//
//
//
//
//
