
/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

        package org.firstinspires.ftc.teamcode;

        import android.app.Activity;
        import android.graphics.Color;
        import android.util.Log;
        import android.view.View;

        import com.qualcomm.ftccommon.DbgLog;
        import com.qualcomm.ftcrobotcontroller.R;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorController;
        import com.qualcomm.robotcore.util.Range;

        import java.util.Random;
public class ColorTest extends OpMode {
    public void init() {
        // ColorSensor sensorRGB;
        //DcMotor leftMotor;
        //DcMotor rightMotor;
    }

    ColorSensor sensorRGB;

    @Override
    public void loop() {
        hardwareMap.logDevices();
        sensorRGB = hardwareMap.colorSensor.get("mr");
        //sensorRGB.enableLed(true);

        String colorfound = "none";
        double blue = sensorRGB.blue();
        double red = sensorRGB.red();
        double clear = sensorRGB.alpha();
        double green = sensorRGB.green();
        float[] HSVTest = {0F, 0F, 0F};
        sensorRGB.enableLed(true);
        Color.RGBToHSV(sensorRGB.red() * 8, sensorRGB.green() * 8, sensorRGB.blue() * 8, HSVTest);
        telemetry.addData("Clear", sensorRGB.alpha());
        telemetry.addData("Red  ", sensorRGB.red());
        telemetry.addData("Green", sensorRGB.green());
        telemetry.addData("Blue ", sensorRGB.blue());
        telemetry.addData("Hue", HSVTest[0]);
        telemetry.addData("Saturation", HSVTest[1]);
        telemetry.addData("Value", HSVTest[2]);
    }
}
           // if(HSVTest[0] >= 180 && HSVTest[1] >= 0.5 && HSVTest[2] >= 0.033  || HSVTest[0] >= 0 && HSVTest[1] >= 1 && HSVTest[2] >= 0.03 && HSVTest[0] <= 50){
              //  colorfound = "blue or red";






           // if(HSVTest[0] >= 180 && HSVTest[1] >= 0.5 && HSVTest[2] >= 0.033  ||/*tests for red */ HSVTest[0] >= 0 && HSVTest[1] >= 1 && HSVTest[2] >= 0.03 && HSVTest[0] <= 50 ) // && blue >= 5 && green >=1 && clear >=2 && green <= 15 && clear <= 10 && red >= 0 && red <= 2) //Tests for blue
           // {
                //colorfound = "blue or red";

               // if (/*HSVTest[0] >= 180 && HSVTest[1] >= 0.5 && HSVTest[2] >= 0.033*/HSVTest[0] >= 0 && HSVTest[1] >= 1 && HSVTest[2] >= 0.03 && HSVTest[0] <= 50) {



                //if(HSVTest[0] >= 0 && HSVTest[1] >= 1 && HSVTest[2] >= 0.3 && red >= 8 && green >=3 && clear >=10 && green <= 7 && clear <= 21 && blue >= 0 && blue <= 2)

//