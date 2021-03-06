package org.firstinspires.ftc.teamcode.competition_in_work.driver_control;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotScranton;
import org.firstinspires.ftc.teamcode.mechbot.utill.MechBotDriveControls;
import org.firstinspires.ftc.teamcode.mechbot.presupers_bot.MechBotSensor;

/**
 * Created by FTC Team 8397 on 2/22/2018.
 */
@TeleOp(name = "AutoBalance", group = "Rev")
@Disabled
public class AutoBalance extends LinearOpMode {
    public MechBotScranton bot = new MechBotScranton();
    public AutoBalancer autoBalancer = null;
    boolean balancing = false;
    private MechBotDriveControls mechBotDriveControls = new MechBotDriveControls(gamepad1,gamepad2,bot);

    private boolean AUTOBALANCER_LOG = true;
    private String AUTOBALANCER_TAG ="AutoBalancer";

    @Override
    public void runOpMode() throws InterruptedException {
        bot.init(hardwareMap);
        telemetry.addData("Ready to start:","Goo!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            if(balancing){
                if(gamepad1.b){
                    balancing = true;
                    autoBalancer.update();
                    continue;
                }else{
                    balancing = false;
                    autoBalancer = null;
                    bot.setDrivePower(0,0,0);
                }
            }else{
                if(gamepad1.b){
                    balancing = true;
                    autoBalancer = new AutoBalancer(-30,-120,0,60,0);
                    autoBalancer.start();
                    continue;
                }
            }
            mechBotDriveControls.refreshGamepads(gamepad1, gamepad2);
            mechBotDriveControls.joyStickMecnumDriveCompNewBot(new float[8]); //Do an arr
        }
    }

    private class AutoBalancer{
        private float initSpeed;
        private float rollCoeff, pitchCoeff, pitchDerivCoeff, rollDerivCoeff; //Coefficients for control of pitch and roll
        private float initPitch, initRoll;  //Base values of pitch and roll, before starting balance operation
        private ElapsedTime timer;
        private boolean mounted = false;  //Has bot gotten fully onto the stone yet?
        private float prevTimeSec, prevPitchRads, prevRollRads; //Time in seconds from previous update

        public AutoBalancer(float initialSpeed, float rollCoefficient, float rollDerivCoefficient, float pitchCoefficient, float pitchDerivCoefficient){
            initSpeed = initialSpeed;
            rollCoeff = rollCoefficient;
            pitchCoeff = pitchCoefficient;
            pitchDerivCoeff = pitchDerivCoefficient;
            rollDerivCoeff = rollDerivCoefficient;
        }

        public void start(){
            Orientation angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            initRoll = angles.secondAngle;
            initPitch = angles.thirdAngle;
            timer = new ElapsedTime();
            prevTimeSec = (float)timer.seconds();
            prevPitchRads = initPitch;
            prevRollRads = initRoll;
            bot.setDriveSpeed(0, initSpeed, 0);
        }

        public void update(){
            if (!mounted){
                int red = bot.colorRight.blue();
                int green = bot.colorRight.green();
                int blue = bot.colorRight.blue();
                float[] hsv = new float[3];
                Color.RGBToHSV(red, green, blue, hsv);
                if (hsv[1] > 0.5) {
                    mounted = true;
                    pidControlIteration();
                }
            }
            else{
                pidControlIteration();
            }
        }

        private void pidControlIteration(){
            Orientation angles = bot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            float roll = angles.secondAngle;
            float pitch = angles.thirdAngle;

            float seconds = (float)timer.seconds();

            float pitchDerivative = (pitch - prevPitchRads) / (seconds - prevTimeSec);
            float vy = pitchCoeff * (pitch - initPitch) + pitchDerivCoeff * pitchDerivative;

            float rollDerivative = (roll - prevRollRads)  /(seconds - prevTimeSec);
            float vx = rollCoeff * (roll - initRoll) + rollDerivCoeff + rollDerivative;

            prevTimeSec = seconds;
            prevPitchRads = pitch;
            prevRollRads = roll;
            bot.setDriveSpeed(vx, vy, 0);
        }
    }
}
