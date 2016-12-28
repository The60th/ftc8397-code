package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Created by JimLori on 12/22/2016.
 */
@TeleOp(name="Omnibot Test Encoders", group="Pushbot")
public class TestOmniBotEncoders extends LinearOpMode {

    OmniBot robot = new OmniBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //float x = 0;
        //float y = 0;
        //float theta = 0;

       // VectorF priorReportedTicks = new VectorF(0,0,0,0);
        telemetry.addData("encoders "," %d %d %d %d ", robot.one.getCurrentPosition(),robot.two.getCurrentPosition(),robot.three.getCurrentPosition(),robot.four.getCurrentPosition());
        telemetry.update();
        waitForStart();

        ElapsedTime et = new ElapsedTime();

        while (opModeIsActive()){
           // float pos[] = robot.updateOdometry(x, y, theta);
            //x = pos[0];
            //y = pos[1];
            //theta = pos[2];
            float seconds = (float)et.seconds();
            if (seconds > 0.2){
                et.reset();
               //telemetry.addData("Encoders", robot.last_Wheel_Ticks.toString());
                //telemetry.addData("Ticks/Sec",
                  //      robot.last_Wheel_Ticks.subtracted(priorReportedTicks).multiplied(1.0f/seconds).toString());
                //telemetry.addData("Pos", "x = %.0f y = %.0f theta = %.0f", x, y, theta * 180.0 / Math.PI);
                //telemetry.addData("IntZ Value", "%d", robot.sensorGyro.getIntegratedZValue());
                telemetry.addData("encoders "," %d %d %d %d ", robot.one.getCurrentPosition(),robot.two.getCurrentPosition(),robot.three.getCurrentPosition(),robot.four.getCurrentPosition());
                telemetry.update();
                //priorReportedTicks = robot.last_Wheel_Ticks;
            }
            float gm1lx = gamepad1.left_stick_x;
            float gm1ly = -gamepad1.left_stick_y;
            float gm1rx = -gamepad1.right_stick_x;
            float px = Math.abs(gm1lx) < 0.05? 0 : (float)Math.signum(gm1lx) * gm1lx * gm1lx;
            float py = Math.abs(gm1ly) < 0.05? 0 : (float)Math.signum(gm1ly) * gm1ly * gm1ly;
            float pa = Math.abs(gm1rx) < 0.05? 0 : (float)Math.signum(gm1rx) * gm1rx * gm1rx;
            robot.setDrivePower(px, py, pa,"");
        }

    }

}
