package org.firstinspires.ftc.teamcode.competition_in_work.auto.scranton;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.beta_log.BetaLog;
import org.firstinspires.ftc.teamcode.beta_log.LoggingLinearOpMode;
import org.firstinspires.ftc.teamcode.cv_programs.CryptoNav;
import org.firstinspires.ftc.teamcode.mechbot.supers_bot.MechBotAutonomousScranton;
import org.firstinspires.ftc.teamcode.vuforia_libs.VuMarkNavigator;

import java.util.concurrent.BlockingQueue;

/**
 * Created by FTC Team 8397 on 3/26/2018.
 */


@Autonomous(name = "Red Bottom MG", group = "Auto")
@Disabled
public class RedBottomMG extends MechBotAutonomousScranton {

    private final String TAG = "RED_BOTTOM_MG";

    private int sampleRatio = 4;
    private int rawImgWidth = 1280; //use 1280 for G4 //640
    private int rawImgHeight = 720; //use 720 for G4 //480
    private int rangeX0 = 40; //Consider using 40 for G4 //0
    private int rangeY0 = 0; //Consider using 240 for G4 //0
    private int rangeWidth = 1200; //Consider using 1200 for G4 //640
    private int rangeHeight = 240; //Consider using 240 for G4 //480
    private float rawFocalLength = 1082f; //Correct for g4
    private float rawPrincipalX = 640f;  //Should be roughly 640 for G4 //320f

    private byte[] imageBytes = new byte[2 * rawImgWidth * rawImgHeight];

    final float[] hsvValues = new float[3];

    final boolean BLUE_BOTTOM_START_LOG = true;
    final String RED_BOTTOM_START_TAG = "Red bottom start Worlds:";
    @Override
    public void runLoggingOpmode() throws InterruptedException {
        bot.init(hardwareMap, -90); //The starting value of the gyro heading comapred to the wall.
        CryptoNav.initParams(CryptoNav.TeamColor.RED, rawImgWidth, rawImgHeight, rangeX0, rangeY0, rangeWidth, rangeHeight,
                rawFocalLength, rawPrincipalX, sampleRatio);

        //The starting angle is the gyro heading relative to the crypto box.
        robotZXPhi = new float[3];

        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(RED_BOTTOM_START_TAG, "INITIALIZE AUTO");

        //Contains wait for start.
        initAuto(MechBotAutonomousScranton.TeamColor.RED, VUMARK_KEY_SCAN_TIME, JEWEL_SCAN_TIME); //Knocks the jewel off the stone and finds crypto key.

        final float OFF_STONE_DISTANCE = 25.0f * 2.54f;

        setOdometry(0,0);

        driveDirectionGyro(50, -90, -90, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[1] < -OFF_STONE_DISTANCE;
            }
        });

//        //Assume the robot is facing the wall once again still on the balance stone and the wall is a heading of 0.
//        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(RED_BOTTOM_START_TAG, "driveDirectionGyro 1");
//        //added the 180 to this line of code to keep the robot from turning around.
//        driveDirectionGyro(OFF_STONE_SPEED, -90, -90, new MechBotAutonomousScranton.Predicate() {
//            @Override
//            public boolean isTrue() {
//                Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
//
//                if (BLUE_BOTTOM_START_LOG)
//                    BetaLog.dd(RED_BOTTOM_START_TAG, "Driving on stone sats: S: %.2f", hsvValues[1]);
//
//                if (hsvValues[1] < HSV_SAT_CUT_OFF_STONE) {
//                    //Color sensors are off the stone.
//                    return true;
//                }
//                return false;
//            }
//        });
//
//        setOdometry(0, 0);
//        //robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
//        //Robot is now partly off the stone. Just the front color sensors are off, time to drive the rest of the robot off the stone.
//        driveDirectionGyro(OFF_STONE_SPEED, -90, -90, new MechBotAutonomousScranton.Predicate() {
//            @Override
//            public boolean isTrue() {
//                return robotZXPhi[1] < -5; //Need a constant defined here.
//            }
//        });

        //Robot is now all the way off the balance stone and ready to turn towards the crypto box.
        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(RED_BOTTOM_START_TAG, "turnToheadingGyro");


        turnToHeadingGyro(0, GLOBAL_STANDERD_TOLERANCE, GLOBAL_STANDERD_LATENCY, MechBotAutonomousScranton.RotationDirection.COUNTER_CLOCK); //Turn to face the wall.


        robotZXPhi = new float[]{0, 0, bot.getOdomHeadingFromGyroHeading(bot.getHeadingRadians())};
        driveDirectionGyro(DRIVE_TOWARDS_TRIANGLE_SPEED, 0, new MechBotAutonomousScranton.Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] > 5.0f;
            }
        });

        //Drive towards the box till the colored tape is detected.
        if (BLUE_BOTTOM_START_LOG) BetaLog.dd(RED_BOTTOM_START_TAG, "driveDirectionGyro 2");

        driveDirectionGyro(DRIVE_TOWARDS_TRIANGLE_SPEED, -90, new MechBotAutonomousScranton.Predicate() {
            @Override
            public boolean isTrue() { // I just changed the speed of this to see if we can get back over the balancing stone it used to be 20. Also if this works change all the other auto programs.
                Color.RGBToHSV(bot.colorRight.red() * 8, bot.colorRight.green() * 8, bot.colorRight.blue() * 8, hsvValues);
                if (BLUE_BOTTOM_START_LOG)
                    BetaLog.dd(RED_BOTTOM_START_TAG, "Driving to line sats: S: %.2f", hsvValues[1]);
                if (hsvValues[1] > HSV_SAT_CUT_OFF) {
                    return true;
                }

                return false;
            }
        });

        handleTriangle(TriangleApproachSide.RIGHT,15,10,0,bot.colorLeft,bot.colorRight,2000);

        scoreGlyph(this.cryptoKey);


        setOdometry(53, CRYPTO_BOX_CENTER_SHIFT_VALUE);
        if(this.cryptoKey == RelicRecoveryVuMark.LEFT){
         robotZXPhi[1] -= CRYPTO_BOX_SIDE_SHIFT_VALUE;
        }else if(this.cryptoKey == RelicRecoveryVuMark.RIGHT){
            robotZXPhi[1] += CRYPTO_BOX_SIDE_SHIFT_VALUE;
        }

        bot.setFlipPosition(0); //Lower plate

        sleep(200);

        //bot.disableFlipPlate();

        turnToHeadingGyroQuick(180,GLOBAL_STANDERD_TOLERANCE,GLOBAL_STANDERD_LATENCY,RotationDirection.CLOCK);

        bot.setIntakeOn();

        driveDirectionGyro(50, 0, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] >= 111;
            }
        });

        //turnToHeadingGyroQuick(-90,GLOBAL_STANDERD_TOLERANCE,GLOBAL_STANDERD_LATENCY);

        /*bot.setIntakeOff();

        CameraDevice.getInstance().setFlashTorchMode(true);

        BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue = VuMarkNavigator.getFrameQueue();

        VuMarkNavigator.clearFrameQueue(frameQueue);

        BetaLog.dd(TAG, "Cleared Frame Queue");

        boolean cryptoNavInitialized = false;
        while (opModeIsActive()){
            //Try to get an image; if image not yet available, loop and try again.
            if (!VuMarkNavigator.getRGB565Array(frameQueue, rawImgWidth, rawImgHeight, imageBytes)) continue;

            BetaLog.dd(TAG, "Got CryptoNav Initialization Image");

            //Try to initialize the rails. We're assuming for this demonstration that the left-most
            //rail is visible to the camera before initializeRails is called.
            if(!CryptoNav.initializeRails(imageBytes, CryptoNav.Side.LEFT)) break;
            BetaLog.dd(TAG, "CryptoNav Initialization Succeeded");
            cryptoNavInitialized = true;
            break;
        }

        if (cryptoNavInitialized) {
            BetaLog.dd(TAG, "CryptoNav Initialization Succeeded");
            quickTelemetry("CryptoNav Initialization Succeeded.");
        }
        else {
            BetaLog.dd(TAG, "CryptoNav Initialization Failed.");
            quickTelemetry("CryptoNav Initialization Failed.");
            while (opModeIsActive()) continue;
        }

        //SEVENTH: Enter the actual navigation loop. In practice, each iteration of this loop would include adjustments
        //of motor powers, just as we did with Vuforia navigation.
        BetaLog.dd(TAG, "Entering CryptoNav navigation loop");
        while (opModeIsActive()) {

            //Get a new image; if no image is a available, keep on trying.
            if (!VuMarkNavigator.getRGB565Array(frameQueue, rawImgWidth, rawImgHeight, imageBytes)) continue;

            BetaLog.dd(TAG,"Got CrytoNav navigation image");

            //From the image, use the CryptoNav.updateLocationZX method to obtain new z,x coordinates.
            //Note that this method does not GIVE us the phiPrime heading value. Instead, we need to provide the
            //method with the phiPrime value. In practice, we would obtain this from the GYRO. For this
            //demonstration, it will be assumed that phiPrime is 0 (just keep the phone pointed directly toward the
            //wall.
            float gyroHeading = bot.getHeadingRadians();
            float odomHeading = bot.getOdomHeadingFromGyroHeading(gyroHeading);
            float cameraHeading = bot.getCameraHeadingFromGyroHeading(gyroHeading);

            float[] zx = CryptoNav.updateLocationZX(imageBytes, cameraHeading);

            //Note: if navigation fails, that's usually because only zero or one rails is currently visible.
            //The updateLocationZX method needs to "see" at least two rails. No problem; once two rails become
            //visible again, we should start getting valid locations again.
            if (zx == null){
                BetaLog.dd(TAG, "Crypto Navigation Failed");
                telemetry.addData("Crypto Navigation Failed","");
                robotZXPhi = bot.updateOdometry(robotZXPhi,odomHeading);

            }
            else{
                BetaLog.dd(TAG, "Crypto Navigation Succeeded");
                BetaLog.dd(TAG, "Camera Coords: z = %.1f  x = %.1f", zx[0], zx[1]);
                telemetry.addData("Crypto Navigation Succeeded, Camera:"," z=%.1f x=%.1f", zx[0], zx[1]);
                float[] robotZX = bot.getRobotZXfromCameraZX(zx,gyroHeading);
                setOdometry(robotZX[0],robotZX[1],odomHeading);
            }

            BetaLog.dd("Robot coords:", " z=%.1f  x=%.1f  phi=%.1f", robotZXPhi[0], robotZXPhi[1],
                    robotZXPhi[2] * 180.0/Math.PI);
            telemetry.addData("Robot coords:"," z=%.1f x=%.1f phi=%.1f", robotZXPhi[0], robotZXPhi[1],
                    robotZXPhi[2] * 180.0/Math.PI);

            if(robotZXPhi[0] < 41) break;
            final float vNom = 20.0f;
            final float coeff = 1.0f;
            float vX = -vNom *(float)Math.cos(cameraHeading) - coeff * robotZXPhi[1] * (float)Math.sin(cameraHeading);
            float vY = vNom * (float)Math.sin(cameraHeading) - coeff * robotZXPhi[1] * (float)Math.cos(cameraHeading);
            float vA = -HEADING_CORECTION_FACTOR * (float)VuMarkNavigator.NormalizeAngle(gyroHeading + Math.PI/2.0f);
            bot.setDriveSpeed(vX,vY,vA);

            telemetry.update();
        }

        bot.setDriveSpeed(0,0,0);

        telemetry.update();

        while (opModeIsActive()) continue;*/

        //Encoder based scoring.
        turnToHeadingGyroQuick(180,GLOBAL_STANDERD_TOLERANCE,GLOBAL_STANDERD_LATENCY);

        bot.setIntakeOff();

        if(robotZXPhi[1] > 0){
            driveDirectionGyro(25, -90, 180, new Predicate() {
                @Override
                public boolean isTrue() {
                    return robotZXPhi[1] <= 0;
                }
            });
        }else {
            driveDirectionGyro(25, 90, 180, new Predicate() {
                @Override
                public boolean isTrue() {
                    return robotZXPhi[1] >= 0;
                }
            });
        }

        driveDirectionGyro(50, 180, 180, new Predicate() {
            @Override
            public boolean isTrue() {
                return robotZXPhi[0] < 100;
            }
        });

        handleTriangleFromFront(25,15,10,180,bot.backColorLeft,bot.backColorRight,2000);

        while (opModeIsActive()){
            telemetry.addData("Run time: " , runTime.seconds());
            telemetry.update();
        }
       // setOdometry(0,0);

//        driveDirectionGyro(50, 90, 180, new Predicate() {
//            @Override
//            public boolean isTrue() {
//                return robotZXPhi[1] > -CRYPTO_BOX_CENTER_SHIFT_VALUE*2;
//            }
//        });


    }
}
