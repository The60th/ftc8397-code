///*
//Copyright (c) 2016 Robert Atkinson
//
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without modification,
//are permitted (subject to the limitations in the disclaimer below) provided that
//the following conditions are met:
//
//Redistributions of source code must retain the above copyright notice, this list
//of conditions and the following disclaimer.
//
//Redistributions in binary form must reproduce the above copyright notice, this
//list of conditions and the following disclaimer in the documentation and/or
//other materials provided with the distribution.
//
//Neither the name of Robert Atkinson nor the names of his contributors may be used to
//endorse or promote products derived from this software without specific prior
//written permission.
//
//NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
//TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
//THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//*/
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.ftcrobotcontroller.R;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
///**
// * This OpMode illustrates the basics of using the Vuforia localizer to determine
// * positioning and orientation of robot on the FTC field.
// * The code is structured as a LinearOpMode
// *
// * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
// *
// * When images are located, Vuforia is able to determine the position and orientation of the
// * image relative to the camera.  This sample code then combines that information with a
// * knowledge of where the target images are on the field, to determine the location of the camera.
// * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
// * is explained below.
// */
//
//@Autonomous(name="Vuforia Navigation Test", group ="Concept")
////@Disabled
////Enabling for vuforia demos.
//public class VuforiaNavigationTest extends LinearOpMode {
//
//    public static final String TAG = "Vuforia Sample";
//
//    OpenGLMatrix lastLocation = null;
//
//    float x = 0;
//    float y = 0;
//    float z = 0;
//    float theta = 0;
//
//    /**
//     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
//     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
//     * Vuforia will not load without a valid license being provided. Vuforia 'Development' license
//     * keys, which is what is needed here, can be obtained free of charge from the Vuforia developer
//     * web site at https://developer.vuforia.com/license-manager.
//     */
//    VuforiaLocalizer vuforia;
//
//    @Override public void runOpMode() throws InterruptedException {
//
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
//        parameters.vuforiaLicenseKey = "INSERT YOUR LICENSE KEY HERE!";
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//
//        VuforiaTrackables stonesAndChips = this.vuforia.loadTrackablesFromAsset("StonesAndChips");
//        VuforiaTrackable redTarget = stonesAndChips.get(0);
//        redTarget.setName("RedTarget");  // Stones
//
//        VuforiaTrackable blueTarget  = stonesAndChips.get(1);
//        blueTarget.setName("BlueTarget");  // Chips
//
//        //Red target on Wall with Y = 1000
//        OpenGLMatrix redTargetLocationOnField = OpenGLMatrix.translation(0, 1000, 0)
//                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 0, 0));
//        redTarget.setLocation(redTargetLocationOnField);
//
//        //Blue target on Wall with Y = -1000
//        OpenGLMatrix blueTargetLocationOnField = OpenGLMatrix.translation(0, -1000, 0)
//                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 180, 0));
//        blueTarget.setLocation(blueTargetLocationOnField);
//
//        //Phone at center of robot, front facing back (for back camera use)
//        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
//                .translation(0,0,0)
//                .multiplied(Orientation.getRotationMatrix(
//                        AxesReference.EXTRINSIC, AxesOrder.XZX,
//                        AngleUnit.DEGREES, 90, 0, 0));
//
//        /**
//         * Let the trackable listeners we care about know where the phone is. We know that each
//         * listener is a VuforiaTrackableDefaultListener and can so safely cast because
//         * we have not ourselves installed a listener of a different type.
//         */
//
//        ((VuforiaTrackableDefaultListener)redTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//        ((VuforiaTrackableDefaultListener)blueTarget.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
//
//        /** Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start tracking");
//        telemetry.update();
//        waitForStart();
//
//        /** Start tracking the data sets we care about. */
//        stonesAndChips.activate();
//
//        ElapsedTime et = new ElapsedTime(0);
//
//
//        while (opModeIsActive()) {
//
//            if (et.milliseconds() > 500) {
//
//
//                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) redTarget.getListener()).getUpdatedRobotLocation();
//
//                if (robotLocationTransform != null) {
//                    lastLocation = robotLocationTransform;
//                    telemetry.addData("Localization ", "Succeeded");
//                    float[] locationArray = robotLocationTransform.getData();
//                    x = locationArray[12];
//                    y = locationArray[13];
//                    z = locationArray[14];
//                    theta = (float) Math.atan2(locationArray[5], locationArray[4]) * (180.0f) / (float) Math.PI;
//                    telemetry.addData("Loc: ", "x = %.0f y = %.0f z = %.0f theta = %.0f", x, y, z, theta);
//                } else {
//                    telemetry.addData("Localization ", "Failed");
//                    telemetry.addData("Loc ", "Unknown");
//                }
//
//                et.reset();
//            }
//            telemetry.update();
//            idle();
//        }
//    }
//
//}
