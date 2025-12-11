package org.firstinspires.ftc.teamcode.operations;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Locale;

public class IterativeOtisAprilTagCalibration extends RobotOperation {

    boolean detected = false;
    @Override
    public void loop() {
        if (detected){
            return;
        }

        List<AprilTagDetection> detections = robot.getDetections();

        for (AprilTagDetection detection : detections) {
            if (!detection.metadata.name.contains("Obelisk")) {
                //heading is returned with camera relative to the front
                robot.setCurrentPosition(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw() + 180);
                detected = true;
                return;
            }
        }
    }

    @NonNull
    @Override
    public String toString() {
        return(String.format(Locale.US, "%s:%b",getClass().getSimpleName(), detected));
    }

    @Override
    public boolean isFinished() {
        return detected;
    }
}
