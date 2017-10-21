package org.firstinspires.ftc.teamcode;

/**
 * Created by JimLori on 10/11/2017.
 */

public class HSV_Range {

        private float minHue;
        private float maxHue;
        private float minSat;
        private float maxSat;
        private float minVal;
        private float maxVal;

        public HSV_Range(float hueMin, float hueMax, float satMin, float satMax, float valMin, float valMax){
            minHue = hueMin;
            maxHue = hueMax;
            minSat = satMin;
            maxSat = satMax;
            minVal = valMin;
            maxVal = valMax;
        }

        public float getMinHue() {
            return minHue;
        }

        public float getMaxHue() {
            return maxHue;
        }

        public float getMinSat() {
            return minSat;
        }

        public float getMaxSat() {
            return maxSat;
        }

        public float getMinVal() {
            return minVal;
        }

        public float getMaxVal() {
            return maxVal;
        }
}
