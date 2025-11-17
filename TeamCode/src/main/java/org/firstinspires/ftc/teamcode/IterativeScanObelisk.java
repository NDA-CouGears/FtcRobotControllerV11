package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.List;

public class IterativeScanObelisk extends RobotOperation{
    private AprilTagPoseFtc ftcPose;

    public enum OBELISK_PATTERN{
        GPP,
        PGP,
        PPG,
        unknown}

    public static OBELISK_PATTERN curPattern = OBELISK_PATTERN.unknown;
    @Override
    public void loop() {
        List<AprilTagDetection> detections = robot.getDetections();

        for (AprilTagDetection tag : detections){
            switch (tag.id) {
                case 21:
                    curPattern = OBELISK_PATTERN.GPP;
                    break;
                case 22:
                    curPattern = OBELISK_PATTERN.PGP;
                    break;
                case 23:
                    curPattern = OBELISK_PATTERN.PPG;
                    break;
            }

        }
    }
    @Override
    public boolean isFinished() {
        return curPattern != OBELISK_PATTERN.unknown;
    }
}
