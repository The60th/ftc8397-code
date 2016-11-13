/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;


@Autonomous(name="AutonomousOpMode1", group="Pushbot")
//@Disabled
public class AutonomousOpMode1 extends LinearOpMode {
    public final float C_PHI = .1f;
    public final float C_X = .1f;
    float[] zxPhi;
    VuforiaNav vuforianav = null;

    /* Declare OpMode members. */
    OmniBot        robot   = new OmniBot();
    allSensors sensors = new allSensors();
    @Override
    public void runOpMode() throws InterruptedException {
        float v = 10;
        robot.init(hardwareMap);
        vuforianav = new VuforiaNav();
        vuforianav.activate();
        waitForStart();
        OpenGLMatrix robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);
        while(opModeIsActive() && robotPosition == null){
            robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);
            robot.setDrivePower(0, 0, 0);
            idle();
        }
        if(robotPosition != null){
            zxPhi = VuforiaNav.GetZXPH(robotPosition);
        }
        while (opModeIsActive() && zxPhi[0] >= 15) {
            float[] newSpeeds = getCorrectedSpeeds(zxPhi[1], zxPhi[2], v);
            robot.setDriveSpeed(newSpeeds[0], newSpeeds[1], newSpeeds[2]);
            idle();
            robotPosition = vuforianav.getRobotLocationRelativeToTarget(3);
            if(robotPosition != null) {
                zxPhi = VuforiaNav.GetZXPH(robotPosition);
            }
        }
        robot.setDrivePower(0, 0, 0);

    }

    public float[] getCorrectedSpeeds(float x,float phi,float v) {
        float phiPrime = VuforiaNav.remapAngle(phi-(float)Math.PI);
        float va = -phiPrime*v*C_PHI;
        float vx = -C_X*v*x*(float)Math.cos(phiPrime);
        float vy = v+v*C_X*x*(float)Math.sin(phiPrime);
        return new float[]{vx,vy,va};
    }
}

