package org.firstinspires.ftc.teamcode.operations;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IterativeRobotParent;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.List;
import java.util.Locale;

public class IterativeScanObelisk extends RobotOperation {
    private AprilTagPoseFtc ftcPose;
    private ElapsedTime runtime = new ElapsedTime();

    public enum OBELISK_PATTERN{
        GPP,
        PGP,
        PPG,
        unknown}

    @Override
    public void init(IterativeRobotParent robot) {
        super.init(robot);
        runtime.reset();
    }

    @NonNull
    @Override
    public String toString() {
        return(String.format(Locale.US, "%s(%s)",getClass().getSimpleName(), curPattern));
    }

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
        if (runtime.seconds()>2){
            curPattern = OBELISK_PATTERN.PPG;
        }
    }
    @Override
    public boolean isFinished() {
        return curPattern != OBELISK_PATTERN.unknown;
    }
}
