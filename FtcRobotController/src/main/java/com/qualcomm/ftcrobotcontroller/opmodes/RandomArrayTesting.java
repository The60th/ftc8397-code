package com.qualcomm.ftcrobotcontroller.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.util.Log;
import android.view.View;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.lang.reflect.Array;
import java.util.Random;


public class RandomArrayTesting extends LinearOpMode {
    ColorSensor sensorRGB;
    //double blue = sensorRGB.blue();
    //double red = sensorRGB.red();
    //double clear = sensorRGB.alpha();
    //double green = sensorRGB.green();
    //float []HSVTest ={0F,0F,0F};
    //String[] Test = {"Taco","NonTaco"};


   /* public ColorSensorClass(ColorSensor mr){

  sensorRGB = mr;
        sensorRGB.enableLed(true);

        Color.RGBToHSV(sensorRGB.red()*8, sensorRGB.green()*8, sensorRGB.blue()*8, HSVTest);
        /*telemetry.addData("Clear", sensorRGB.alpha());
        telemetry.addData("Red  ", sensorRGB.red());
        telemetry.addData("Green", sensorRGB.green());
        telemetry.addData("Blue ", sensorRGB.blue());
        telemetry.addData("Hue", HSVTest[0]);
        telemetry.addData("Saturation", HSVTest[1]);
        telemetry.addData("Value", HSVTest[2]);
    String random = (Test[new Random().nextInt(Test.length)]);


    } */


   /* public void FindRed(double red,double blue,double clear,double green, float HSVTest){

        if(HSVTest[0] >= 0 && HSVTest[1] >= 1 && HSVTest[2] >= 0.3 && red >= 8 && green >=3 && clear >=10 && green <= 7 && clear <= 21 && blue >= 0 && blue <= 2){

            telemetry.addData("Found Red","");
        }


    } */

    public void runOpMode() throws InterruptedException {
        hardwareMap.logDevices();
    /*sensorRGB = hardwareMap.colorSensor.get("mr");
    sensorRGB.enableLed(false);
    double blue = sensorRGB.blue();
    double red = sensorRGB.red();
    double clear = sensorRGB.alpha();
    double green = sensorRGB.green();
    float []HSVTest ={0F,0F,0F};*/

        String[] Test = {
                "( .-. )",
                "( .o.)",
                "( `·´ )",
                "( ° ͜ ʖ °)",
                "( ͡° ͜ʖ ͡°)",
                "( ⚆ _ ⚆ )",
                "( ︶︿︶)",
                "( ﾟヮﾟ)",
                "(\\/)(°,,,°)(\\/)",
                "(¬_¬)",
                "(¬º-°)¬",
                "(¬‿¬)",
                "(°ロ°)☝",
                "(´・ω・)っ",
                "(ó ì_í)",
                "(ʘᗩʘ')",
                "(ʘ‿ʘ)",
                "(̿▀̿ ̿Ĺ̯̿̿▀̿ ̿)̄",
                "(͡° ͜ʖ ͡°)",
                "(ಠ_ಠ)",
                "(ಠ‿ಠ)",
                "(ಠ⌣ಠ)",
                "(ಥ_ಥ)",
                "(ಥ﹏ಥ)",
                "(ง ͠° ͟ل͜ ͡°)ง",
                "(ง ͡ʘ ͜ʖ ͡ʘ)ง",
                "(ง •̀_•́)ง",
                "(ง'̀-'́)ง",
                "(ง°ل͜°)ง",
                "(ง⌐□ل͜□)ง",
                "(ღ˘⌣˘ღ)",
                "(ᵔᴥᵔ)",
                "(•ω•)",
                "(•◡•)/",
                "(⊙ω⊙)",
                "(⌐■_■)",
                "(─‿‿─)",
                "(╯°□°）╯",
                "(◕‿◕)",
                "(☞ﾟ∀ﾟ)☞",
                "(❍ᴥ❍ʋ)",
                "(っ◕‿◕)っ",
                "(づ｡◕‿‿◕｡)づ",
                "(ノಠ益ಠ)ノ",
                "(ノ・∀・)ノ",
                "(；一_一)",
                "(｀◔ ω ◔´)",
                "(｡◕‿‿◕｡)",
                "(ﾉ◕ヮ◕)ﾉ",
                "*<{:¬{D}}}",
                "=^.^=",
                "t(-.-t)",
                "| (• ◡•)|",
                "~(˘▾˘~)",
                "¬_¬",
                "¯(°_o)/¯",
                "¯_(ツ)_/¯",
                "°Д°",
                "ɳ༼ຈل͜ຈ༽ɲ",
                "ʅʕ•ᴥ•ʔʃ",
                "ʕ´•ᴥ•`ʔ",
                "ʕ•ᴥ•ʔ",
                "ʕ◉.◉ʔ",
                "ʕㅇ호ㅇʔ",
                "ʕ；•`ᴥ•´ʔ",
                "ʘ‿ʘ",
                "͡° ͜ʖ ͡°",
                "ζ༼Ɵ͆ل͜Ɵ͆༽ᶘ",
                "Ѱζ༼ᴼل͜ᴼ༽ᶘѰ",
                "ب_ب",
                "٩◔̯◔۶",
                "ಠ_ಠ",
                "ಠoಠ",
                "ಠ~ಠ",
                "ಠ‿ಠ",
                "ಠ⌣ಠ",
                "ಠ╭╮ಠ",
                "ರ_ರ",
                "ง ͠° ل͜ °)ง",
                "๏̯͡๏﴿",
                "༼ ºººººل͟ººººº ༽",
                "༼ ºل͟º ༽",
                "༼ ºل͟º༼",
                "༼ ºل͟º༽",
                "༼ ͡■ل͜ ͡■༽",
                "༼ つ ◕_◕ ༽つ",
                "༼ʘ̚ل͜ʘ̚༽",
                "ლ(´ڡ`ლ)",
                "ლ(́◉◞౪◟◉‵ლ)",
                "ლ(ಠ益ಠლ)",
                "ᄽὁȍ ̪őὀᄿ",
                "ᔑ•ﺪ͟͠•ᔐ",
                "ᕕ( ᐛ )ᕗ",
                "ᕙ(⇀‸↼‶)ᕗ",
                "ᕙ༼ຈل͜ຈ༽ᕗ",
                "ᶘ ᵒᴥᵒᶅ",
                "(ﾉಥ益ಥ）ﾉ",
                "≧☉_☉≦",
                "⊙▃⊙",
                "⊙﹏⊙",
                "┌( ಠ_ಠ)┘",
                "╚(ಠ_ಠ)=┐",
                "◉_◉",
                "◔ ⌣ ◔",
                "◔̯◔",
                "◕‿↼",
                "◕‿◕",
                "☉_☉",
                "☜(⌒▽⌒)☞",
                "☼.☼",
                "♥‿♥",
                "⚆ _ ⚆",
                "✌(-‿-)✌",
                "〆(・∀・＠)",
                "ノ( º _ ºノ)",
                "ノ( ゜-゜ノ)",
                "ヽ( ͝° ͜ʖ͡°)ﾉ",
                "ヽ(`Д´)ﾉ",
                "ヽ༼° ͟ل͜ ͡°༽ﾉ",
                "ヽ༼ʘ̚ل͜ʘ̚༽ﾉ",
                "ヽ༼ຈل͜ຈ༽ง",
                "ヽ༼ຈل͜ຈ༽ﾉ",
                "ヽ༼Ὸل͜ຈ༽ﾉ",
                "ヾ(⌐■_■)ノ",
                "꒰･◡･๑꒱",
                "﴾͡๏̯͡๏﴿",
                "｡◕‿◕｡",
                "ʕノ◔ϖ◔ʔノ","( .-. )",
                "( .o.)",
                "( `·´ )",
                "( ° ͜ ʖ °)",
                "( ͡° ͜ʖ ͡°)",
                "( ⚆ _ ⚆ )",
                "( ︶︿︶)",
                "( ﾟヮﾟ)",
                "(\\/)(°,,,°)(\\/)",
                "(¬_¬)",
                "(¬º-°)¬",
                "(¬‿¬)",
                "(°ロ°)☝",
                "(´・ω・)っ",
                "(ó ì_í)",
                "(ʘᗩʘ')",
                "(ʘ‿ʘ)",
                "(̿▀̿ ̿Ĺ̯̿̿▀̿ ̿)̄",
                "(͡° ͜ʖ ͡°)",
                "(ಠ_ಠ)",
                "(ಠ‿ಠ)",
                "(ಠ⌣ಠ)",
                "(ಥ_ಥ)",
                "(ಥ﹏ಥ)",
                "(ง ͠° ͟ل͜ ͡°)ง",
                "(ง ͡ʘ ͜ʖ ͡ʘ)ง",
                "(ง •̀_•́)ง",
                "(ง'̀-'́)ง",
                "(ง°ل͜°)ง",
                "(ง⌐□ل͜□)ง",
                "(ღ˘⌣˘ღ)",
                "(ᵔᴥᵔ)",
                "(•ω•)",
                "(•◡•)/",
                "(⊙ω⊙)",
                "(⌐■_■)",
                "(─‿‿─)",
                "(╯°□°）╯",
                "(◕‿◕)",
                "(☞ﾟ∀ﾟ)☞",
                "(❍ᴥ❍ʋ)",
                "(っ◕‿◕)っ",
                "(づ｡◕‿‿◕｡)づ",
                "(ノಠ益ಠ)ノ",
                "(ノ・∀・)ノ",
                "(；一_一)",
                "(｀◔ ω ◔´)",
                "(｡◕‿‿◕｡)",
                "(ﾉ◕ヮ◕)ﾉ",
                "*<{:¬{D}}}",
                "=^.^=",
                "t(-.-t)",
                "| (• ◡•)|",
                "~(˘▾˘~)",
                "¬_¬",
                "¯(°_o)/¯",
                "¯_(ツ)_/¯",
                "°Д°",
                "ɳ༼ຈل͜ຈ༽ɲ",
                "ʅʕ•ᴥ•ʔʃ",
                "ʕ´•ᴥ•`ʔ",
                "ʕ•ᴥ•ʔ",
                "ʕ◉.◉ʔ",
                "ʕㅇ호ㅇʔ",
                "ʕ；•`ᴥ•´ʔ",
                "ʘ‿ʘ",
                "͡° ͜ʖ ͡°",
                "ζ༼Ɵ͆ل͜Ɵ͆༽ᶘ",
                "Ѱζ༼ᴼل͜ᴼ༽ᶘѰ",
                "ب_ب",
                "٩◔̯◔۶",
                "ಠ_ಠ",
                "ಠoಠ",
                "ಠ~ಠ",
                "ಠ‿ಠ",
                "ಠ⌣ಠ",
                "ಠ╭╮ಠ",
                "ರ_ರ",
                "ง ͠° ل͜ °)ง",
                "๏̯͡๏﴿",
                "༼ ºººººل͟ººººº ༽",
                "༼ ºل͟º ༽",
                "༼ ºل͟º༼",
                "༼ ºل͟º༽",
                "༼ ͡■ل͜ ͡■༽",
                "༼ つ ◕_◕ ༽つ",
                "༼ʘ̚ل͜ʘ̚༽",
                "ლ(´ڡ`ლ)",
                "ლ(́◉◞౪◟◉‵ლ)",
                "ლ(ಠ益ಠლ)",
                "ᄽὁȍ ̪őὀᄿ",
                "ᔑ•ﺪ͟͠•ᔐ",
                "ᕕ( ᐛ )ᕗ",
                "ᕙ(⇀‸↼‶)ᕗ",
                "ᕙ༼ຈل͜ຈ༽ᕗ",
                "ᶘ ᵒᴥᵒᶅ",
                "(ﾉಥ益ಥ）ﾉ",
                "≧☉_☉≦",
                "⊙▃⊙",
                "⊙﹏⊙",
                "┌( ಠ_ಠ)┘",
                "╚(ಠ_ಠ)=┐",
                "◉_◉",
                "◔ ⌣ ◔",
                "◔̯◔",
                "◕‿↼",
                "◕‿◕",
                "☉_☉",
                "☜(⌒▽⌒)☞",
                "☼.☼",
                "♥‿♥",
                "⚆ _ ⚆",
                "✌(-‿-)✌",
                "〆(・∀・＠)",
                "ノ( º _ ºノ)",
                "ノ( ゜-゜ノ)",
                "ヽ( ͝° ͜ʖ͡°)ﾉ",
                "ヽ(`Д´)ﾉ",
                "ヽ༼° ͟ل͜ ͡°༽ﾉ",
                "ヽ༼ʘ̚ل͜ʘ̚༽ﾉ",
                "ヽ༼ຈل͜ຈ༽ง",
                "ヽ༼ຈل͜ຈ༽ﾉ",
                "ヽ༼Ὸل͜ຈ༽ﾉ",
                "ヾ(⌐■_■)ノ",
                "꒰･◡･๑꒱",
                "﴾͡๏̯͡๏﴿",
                "｡◕‿◕｡",
                "ʕノ◔ϖ◔ʔノ"
        };


        while (opModeIsActive()) {
            String RandomFace = (Test[new Random().nextInt(Test.length)]);
            double[] RanNumberArray = {1};
            double RanNumber = (RanNumberArray[new Random().nextInt(0-100)]);
            telemetry.addData("Number Value?",RanNumber);

            if(RanNumber == 66 || RanNumber ==17 || RanNumber == 76 || RanNumber == 2 || RanNumber == 82 || RanNumber == 30 || RanNumber == 52 || RanNumber == 62 || RanNumber ==14 || RanNumber == 83 || RanNumber == 43 || RanNumber == 55 || RanNumber == 72 || RanNumber == 20 || RanNumber == 90  ) {
                telemetry.addData("Test", RandomFace);
                wait(2000);
                telemetry.clearData();

            }
            wait(5000);

        }





    }
}



