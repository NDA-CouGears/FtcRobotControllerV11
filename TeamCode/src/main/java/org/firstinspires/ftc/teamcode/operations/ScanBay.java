package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IterativeRobotParent;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

/**
 *
 */
public class ScanBay extends CarouselOperations {
    int bay;
    ElapsedTime scanTime = new ElapsedTime();
    double maxWait;  // How long we wait for a ball to be loaded and visible
    double minWait; // How long to wait before checking the visible color

    public ScanBay(int bay) {
        this.bay = bay;
        this.maxWait = 0.2;
        this.minWait = 0.25;
    }

    public ScanBay(int bay, double minWait, double maxWait) {
        this.bay = bay;
        this.minWait = minWait;
        this.maxWait = maxWait;
    }

    @Override
    public void init(IterativeRobotParent robot) {
        super.init(robot);
        scanTime.reset();
    }

    @Override
    public void loop() {
        // If we have been scanning longer than our configured timeout give up
        if (scanTime.seconds() > maxWait) {
            finished = true;
            return;
        }

        String ball;
        PredominantColorProcessor.Swatch current = robot.colorSensor.getAnalysis().closestSwatch;
        robot.telemetry.addLine(String.valueOf(current));
        int[] rgb = robot.colorSensor.getAnalysis().RGB;
        robot.telemetry.addLine(String.format("r:%d; g:%d; b:%d;", rgb[0], rgb[1], rgb[2]));
        int[] hsv = robot.colorSensor.getAnalysis().HSV;
        robot.telemetry.addLine(String.format("h:%d; s:%d; v:%d;", hsv[0], hsv[1], hsv[2]));

        if (hsv[0] < 100){
            ball = "g";
        }
        else if (hsv[1] > 100){
            ball = "p";
        }
        else {
            ball = null;
        }
        colors.set(bay-1, ball);

        // Give it a couple of frames to settle before trusting the scanned value, only say we are
        // finished if we see a ball after a minimum wait time.
        if (scanTime.seconds() > minWait && ball != null) {
            finished = true;
        }

    }
}
