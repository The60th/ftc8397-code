//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
///**
// * Created by CanAdirondack on 12/5/2016.
// */
//@Autonomous(name="VuforiaDemo", group="Pushbot")
//@Disabled
//public class VuforiaDemo extends LinearOpMode {
//    OmniBot        robot   = new OmniBot();
//    VuforiaNav vuforianav = null;
//    float[] zxPhi;
//    float v = 20;
//    public final float C_PHI = .1f;
//    public final float C_X = .1f;
//
//    public void runOpMode() throws InterruptedException{
//
//        OpenGLMatrix robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);
//        //Once code is here it is now in front of the beacon and has tried to press the button.
//        //Now to adjust in the -x to the left to get to the second beacon for a distance of 45.5 inches.
//        while(robotPosition == null){
//            idle();
//            robot.setDrivePower(0,0,0,"");
//            robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);
//            telemetry.addData("NullPos loop","");
//            telemetry.update();
//            // Do a recovery here.
//        }
//        zxPhi = VuforiaNav.GetZXPH(robotPosition);
//        telemetry.addData("Starting Nav","");
//        telemetry.update();
//        while (opModeIsActive() && zxPhi[0] >= 20) { //Was 15 before changing to 21 for testing.
//            float[] newSpeeds = getCorrectedSpeeds(zxPhi[1], zxPhi[2], v);
//            robot.setDriveSpeed(newSpeeds[0], newSpeeds[1], newSpeeds[2]);
//            idle();
//            robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);
//            if(robotPosition != null) {
//                zxPhi = VuforiaNav.GetZXPH(robotPosition);
//            }
//        }
//        robot.setDrivePower(0, 0, 0,"");
//
//    }
//    public float[] getCorrectedSpeeds(float x,float phi,float v) {
//        float phiPrime = VuforiaNav.remapAngle(phi-(float)Math.PI);
//        float va = -phiPrime*v*C_PHI;
//        float vx = -C_X*v*x*(float)Math.cos(phiPrime);
//        float vy = v+v*C_X*x*(float)Math.sin(phiPrime);
//        return new float[]{vx,vy,va};
//    }
//}