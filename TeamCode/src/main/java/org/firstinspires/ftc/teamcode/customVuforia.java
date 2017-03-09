package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@TeleOp(name="Custom Vuforia", group="Custom")
public class customVuforia extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Hello","1");
        telemetry.update();
        VuforiaLocalizer.Parameters parms = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parms.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parms.vuforiaLicenseKey = "AedCWCX/////AAAAGfY4rdOhE0zUoTS0YHGpDSwVSabNSFbSJ+niLI1oFc5sbMeopnrf7cNqwWI0Ty/K/8rhjAjgN2bmLESVhGcOilvJH7JjE+n0lKsXdxSz4K8gIOEszE0BI5+HyDzobZMRo9h5F7lekqaHUsCJZ1AQoxKcdj/obbwkUrTWvpFkX50kwhY4EDwapfB0rjVu7M7+X3VRQ7hFwx7gudHJ/5XaqEm3iunpycpS0TfxCNuGib5rDzZnqUJ4lEo8vPA4JNP/plHzy4iG9QEHIKqNVCvchq1yDXeMXX2lxp1fFAH16fxdEIstNyx1gvVjUIvfi3EdSgHZ8jphnKu+NL0OKgc4VT/6PpMC4dVxcliXlWMK45AF";
        parms.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.TEAPOT;
        telemetry.addData("Hello","2");
        telemetry.update();
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parms);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_OBJECT_TARGETS,4);

        telemetry.addData("Hello","3");
        telemetry.update();

        VuforiaTrackables trackObject = vuforia.loadTrackablesFromAsset("AppleFinal_OT");

        trackObject.get(0).setName("Calc");
        //trackObject.get(1).setName("Apple2");

        waitForStart();

        trackObject.activate();
        telemetry.addData("New version: ", "1.03");
        telemetry.update();
        int i = 0;

        while (opModeIsActive()){
            for(VuforiaTrackable object : trackObject){
                telemetry.addData("",i);
                telemetry.update();
                i++;
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) object.getListener()).getPose();
                if(pose != null){
                    VectorF translation = pose.getTranslation();

                    telemetry.addData(object.getName() + "-Translation: ",translation);

                    double degresToTurn = Math.toDegrees(Math.atan2(translation.get(1),translation.get(2)));

                    telemetry.addData(object.getName() + "-Degrees to turn: " , degresToTurn);

                    telemetry.update();
                }
            }

        }


    }
}
