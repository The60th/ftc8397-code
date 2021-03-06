//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//
///**
// * Created by CanAdirondack on 11/9/2016.
// */
//@Autonomous(name=" TestVuforiaNavClass: Test ", group="Test")
//@Disabled
//
//public class TestVuforiaNavClass extends LinearOpMode {
//    VuforiaNav vuforianav = null;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        vuforianav = new VuforiaNav();
//        vuforianav.activate();
//        waitForStart();
//        ElapsedTime et = new ElapsedTime();
//        while (opModeIsActive()){
//            if (et.milliseconds() > 500){
//                OpenGLMatrix robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);
//                if ( robotPosition != null){
//                    float[] zxPhi = VuforiaNav.GetZXPH(robotPosition);
//                    float phiPrime = VuforiaNav.remapAngle((float)(zxPhi[2]-Math.PI));
//                    telemetry.addData("position ", "x = %.0f y = %.0f phi = %.0f",zxPhi[1],zxPhi[0],(phiPrime*180.0f)/(Math.PI));
//                    telemetry.update();
//                }
//                et.reset();
//            }
//
//            idle();
//        }
//    }
//}
