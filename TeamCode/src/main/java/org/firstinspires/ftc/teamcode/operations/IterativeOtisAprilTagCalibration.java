package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

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

    @Override
    public boolean isFinished() {
        return detected;
    }
}
