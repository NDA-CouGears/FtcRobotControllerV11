package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

public class ScanBay extends CarouselOperations {
    int bay;

    public ScanBay(int bay) {
        this.bay = bay;
    }

    @Override
    public void loop() {
        String ball;
        PredominantColorProcessor.Swatch current = robot.colorSensor.getAnalysis().closestSwatch;
        robot.telemetry.addLine(String.valueOf(current));
        int[] rgb = robot.colorSensor.getAnalysis().RGB;
        robot.telemetry.addLine(String.format("r:%d; g:%d; b:%d;", rgb[0], rgb[1], rgb[2]));
        int[] hsv = robot.colorSensor.getAnalysis().HSV;
        robot.telemetry.addLine(String.format("h:%d; s:%d; v:%d;", hsv[0], hsv[1], hsv[2]));

        /*
        if (current == PredominantColorProcessor.Swatch.ARTIFACT_GREEN){
            ball = "g";
            //finished = true;
        }
        else if (current == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE){
            ball = "p";
            //finished = true;
        }
        else {
            ball = null;
        }
        */
        if (hsv[0] < 100){
            ball = "g";
            //finished = true;
        }
        else if (hsv[1] > 100){
            ball = "p";
            //finished = true;
        }
        else {
            ball = null;
        }
        colors.set(bay-1, ball);
    }
}
