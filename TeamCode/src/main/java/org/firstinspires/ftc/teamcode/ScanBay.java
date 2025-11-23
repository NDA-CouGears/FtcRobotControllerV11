package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

public class ScanBay extends CarouselOperations{
    int bay;

    public ScanBay(int bay) {
        this.bay = bay;
    }

    @Override
    public void loop() {
        String ball;
        PredominantColorProcessor.Swatch current = robot.colorSensor.getAnalysis().closestSwatch;
        robot.telemetry.addLine(String.valueOf(current));
        if (current == PredominantColorProcessor.Swatch.ARTIFACT_GREEN){
            ball = "g";
            finished = true;
        }
        else if (current == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE){
            ball = "p";
            finished = true;
        }
        else {
            ball = null;
        }
        colors.set(bay-1, ball);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
