//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.vuforia.Vuforia;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//
///**
// * Created by JimLori on 11/6/2016.
// */
//
//@Autonomous(name = "TestVuforiaNavigatorClass", group = "Test Opmodes")
//@Disabled
//public class TestVuforiaNavigatorClass extends LinearOpMode {
//
//    final OpenGLMatrix CAMERA_LOCATION_ON_ROBOT =
//            OpenGLMatrix.translation(0,0,0).multiplied(
//                    Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
//                            AngleUnit.DEGREES,90,0,0));
//    final VuforiaLocalizer.CameraDirection CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.BACK;
//
//    VuforiaNavigator vuforiaNavigator = null;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        vuforiaNavigator = new VuforiaNavigator("FTC_2016-17", FTCField.TARGET_LOCATIONS, FTCField.TARGET_NAMES,
//                CAMERA_LOCATION_ON_ROBOT, CAMERA_DIRECTION);
//        vuforiaNavigator.activate();
//
//        waitForStart();
//
//        ElapsedTime et = new ElapsedTime();
//
//        while(opModeIsActive()) {
//            if (et.milliseconds() > 500) {
//                OpenGLMatrix robotPoseRelToWheels = vuforiaNavigator.getRobotPoseRelativeToTarget(0);
//                OpenGLMatrix robotPoseOnField = vuforiaNavigator.getRobotLocationOnField(0);
//                if (robotPoseRelToWheels != null && robotPoseOnField != null) {
//                    double[] xytOnField1 = VuforiaNavigator
//                            .getRobot_X_Y_Theta_FromLocationTransform(robotPoseOnField);
//                    double[] zxpRelToWheels = VuforiaNavigator
//                            .getRobot_Z_X_Phi_FromLocationTransform(robotPoseRelToWheels);
//                    OpenGLMatrix robotPoseOnFieldFromRobotPoseRelToWheels =
//                            vuforiaNavigator.targets.get(0).getLocation().multiplied(robotPoseRelToWheels);
//                    double[] xytOnField2 = VuforiaNavigator
//                            .getRobot_X_Y_Theta_FromLocationTransform(robotPoseOnFieldFromRobotPoseRelToWheels);
//                    telemetry.addData("FieldPos1: ", "x = %.1f y = %.1f th = %.0f", xytOnField1[0] / 10,
//                            xytOnField1[1] / 10, xytOnField1[2]*180/(float)Math.PI);
//                    telemetry.addData("FieldPos2: ", "x = %.1f y = %.1f th = %.0f", xytOnField2[0] / 10,
//                            xytOnField2[1] / 10, xytOnField2[2]*180/(float)Math.PI);
//                    telemetry.addData("TargPos:   ", "z = %.1f x = %.1f ph = %.0f", zxpRelToWheels[0]/10, zxpRelToWheels[1]/10,
//                            VuforiaNavigator.NormalizeAngle(zxpRelToWheels[2] + Math.PI) * 180/(float)Math.PI);
//                    telemetry.update();
//                }
//                et.reset();
//            }
//        }
//
//    }
//
//}
