package org.firstinspires.ftc.teamcode.operations;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IterativeRobotParent;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.Locale;

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

    @NonNull
    @Override
    public String toString() {
        return(String.format(Locale.US, "%s(%d,%f,%f)",getClass().getSimpleName(), bay, minWait, maxWait, colors.get(bay-1), finished));
    }

    @Override
    public void init(IterativeRobotParent robot) {
        super.init(robot);
        scanTime.reset();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        // If we have been scanning longer than our configured timeout give up
        if (scanTime.seconds() > maxWait) {
            finished = true;
            return;
        }

        String ball;
        PredominantColorProcessor.Result ballData = robot.getBallAnalysis();
        PredominantColorProcessor.Swatch current = ballData.closestSwatch;
        robot.telemetry.addLine(String.valueOf(current));
        int[] rgb = ballData.RGB;
        robot.telemetry.addLine(String.format("r:%d; g:%d; b:%d;", rgb[0], rgb[1], rgb[2]));
        int[] hsv = ballData.HSV;
        robot.telemetry.addLine(String.format("h:%d; s:%d; v:%d;", hsv[0], hsv[1], hsv[2]));

        if (current == PredominantColorProcessor.Swatch.ARTIFACT_GREEN){
            ball = "g";
        }
        else if (current == PredominantColorProcessor.Swatch.ARTIFACT_PURPLE){
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
