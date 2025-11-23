package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

public class ScanBay extends CarouselOperations{
    public ScanBay(int bay){
        String ball;
        if (robot.colorSensor.getAnalysis().closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN){
            ball = "g";
        }
        else if (robot.colorSensor.getAnalysis().closestSwatch == PredominantColorProcessor.Swatch.ARTIFACT_GREEN){
            ball = "p";
        }
        else {
            ball = null;
        }
        if (bay == 1){
            colors.set(0,ball);
        }
        else if (bay == 2){
            colors.set(1,ball);
        }
        else if (bay == 3){
            colors.set(2,ball);
        }
    }
}
