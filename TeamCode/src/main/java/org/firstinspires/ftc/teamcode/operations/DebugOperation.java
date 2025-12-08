package org.firstinspires.ftc.teamcode.operations;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

public class DebugOperation extends RobotOperation {
    boolean isFinished = false;

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        if (robot.gamepad1.x || robot.gamepad1.y || robot.gamepad1.a || robot.gamepad1.b)
            isFinished = true;

        Pose2D cur_position = robot.getFieldPosition();
        double x = cur_position.getX(DistanceUnit.INCH);
        double y = cur_position.getY(DistanceUnit.INCH);
        double h = cur_position.getHeading(AngleUnit.DEGREES);
        robot.telemetry.addLine(String.format("OTOS (x,y,h):%2.1f %2.1f %2.1f", x, y, h));

        PredominantColorProcessor.Result ballData = robot.getBallAnalysis();
        PredominantColorProcessor.Swatch swatch = ballData.closestSwatch;
        int[] rgb = ballData.RGB;
        int[] hsv = ballData.HSV;
        robot.telemetry.addLine(
                String.format("COLOR HSV:%d:%d:%d; RBG:%d:%d:%d; SW:%s",
                        hsv[0], hsv[1], hsv[2],
                        rgb[0], rgb[1], rgb[2],
                        swatch));

        List<AprilTagDetection> detections = robot.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata == null)
                continue;

            // Show location data for goals and just id for obelisk
            if (!detection.metadata.name.contains("Obelisk")) {
                x = detection.robotPose.getPosition().x;
                y = detection.robotPose.getPosition().y;
                h = detection.robotPose.getOrientation().getYaw() + 180;
                robot.telemetry.addLine(
                        String.format("Tag (id,x,y,h):%s %2.1f %2.1f %2.1f",
                                detection.metadata.name, x, y, h));
            }
            else {
                robot.telemetry.addLine(String.format("Tag (id):%s", detection.metadata.name));
            }
        }
        robot.addCaroselTelemetry();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
