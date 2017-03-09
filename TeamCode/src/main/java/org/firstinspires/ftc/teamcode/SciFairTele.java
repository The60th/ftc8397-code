package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Justi on 2/24/2017.
 */
@TeleOp(name="Custom Teleop", group="Custom")
public class SciFairTele extends LinearOpMode {

    /**
     * Declare local versions of inuse classes.
     */
    private sciFairBot robot = new sciFairBot();

    /**
     * Global variables px,py,pTheta for position tracking.
     */
    private float px;
    private float py;
    private float pTheta;


    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parms = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parms.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parms.vuforiaLicenseKey = "AedCWCX/////AAAAGfY4rdOhE0zUoTS0YHGpDSwVSabNSFbSJ+niLI1oFc5sbMeopnrf7cNqwWI0Ty/K/8rhjAjgN2bmLESVhGcOilvJH7JjE+n0lKsXdxSz4K8gIOEszE0BI5+HyDzobZMRo9h5F7lekqaHUsCJZ1AQoxKcdj/obbwkUrTWvpFkX50kwhY4EDwapfB0rjVu7M7+X3VRQ7hFwx7gudHJ/5XaqEm3iunpycpS0TfxCNuGib5rDzZnqUJ4lEo8vPA4JNP/plHzy4iG9QEHIKqNVCvchq1yDXeMXX2lxp1fFAH16fxdEIstNyx1gvVjUIvfi3EdSgHZ8jphnKu+NL0OKgc4VT/6PpMC4dVxcliXlWMK45AF";
        parms.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.TEAPOT;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parms);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_OBJECT_TARGETS,4);

        VuforiaTrackables trackObject = vuforia.loadTrackablesFromAsset("AppleFinal_OT");

        trackObject.get(0).setName("Calc");




        /**
         * Init and creation of hardware map using the OmniBot class.
         */
        robot.init(hardwareMap);

        /**
         * Pause at this point till start opMode command is given from the app.
         */
        trackObject.activate();

        telemetry.addData("","All system go!");
        telemetry.update();
        waitForStart();

        ElapsedTime et = new ElapsedTime();

        /**
         * Main whileLoop the program will stay in till opMode is no longer active.
         */
        while (opModeIsActive()) {
            if(gamepad1.guide){
                Boolean taco = true;
                while(taco){
                    for(VuforiaTrackable object : trackObject){
                        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) object.getListener()).getPose();
                        if(pose != null){
                            VectorF translation = pose.getTranslation();

                            telemetry.addData(object.getName() + "-Translation: ",translation);

                            double degresToTurn = Math.toDegrees(Math.atan2(translation.get(1),translation.get(2)));

                            telemetry.addData(object.getName() + "-Degrees to turn: " , degresToTurn);

                            telemetry.update();
                        }
                    }
                    if(!gamepad1.a) taco = false;
                }
            }
            /**
             *
             *  Gamepad One Joy Stick Controls
             *
             */

            /**
             * Get values from the joystick when they are greater then our minimum threshold of 0.05.
             * The left stick on game-pad one controls the main drive wheels using the values px,py.
             * While the right stick controls rational turn using pTheta
             *
             */
            if(gamepad1.x) robot.topArm.setPower(1);
            else if (gamepad1.b) robot.topArm.setPower(-1);
            else robot.topArm.setPower(0);

            if(gamepad1.y) robot.bottomArm.setPower(1);
            else if (gamepad1.a) robot.bottomArm.setPower(-1);
            else robot.bottomArm.setPower(0);

            if(gamepad1.dpad_up) robot.servo.setPosition(1);
            else if(gamepad1.dpad_down) robot.servo.setPosition(0);
            else robot.servo.setPosition(.50);
            px = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
            py = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y : 0;
            pTheta = Math.abs(gamepad1.right_stick_x) > 0.05 ? -gamepad1.right_stick_x : 0;
            robot.setDrivePower(-px/14, -py/14, pTheta/14, OmniBot.phoneFront);
        }
    }
}