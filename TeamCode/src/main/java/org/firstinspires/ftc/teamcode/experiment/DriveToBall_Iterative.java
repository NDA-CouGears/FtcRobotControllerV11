package org.firstinspires.ftc.teamcode.experiment;

import android.graphics.Color;
import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.IterativeRobotParent;
import org.firstinspires.ftc.teamcode.operations.RobotOperation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;
import java.util.Locale;

public class DriveToBall_Iterative extends RobotOperation {
    ElapsedTime runtime;
    private double maxDriveSpeed;
    private double x_target_position;
    private double y_target_position;
    private double target_heading;
    private boolean finished;

    public DriveToBall_Iterative(double maxDriveSpeed) {
        this.maxDriveSpeed = maxDriveSpeed;
    }

    @NonNull
    @Override
    public String toString() {
        return(String.format(Locale.US, "%s(%2.2f, %2.2f, %2.2f):%b",getClass().getSimpleName(), x_target_position, y_target_position, target_heading, finished));
    }

    public void init(IterativeRobotParent robot) {
        super.init(robot);
        runtime = new ElapsedTime();
        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        robot.initDriveToBall();
    }

    public void loop() {
        if (finished) {
            return;
        }
        /*
        Pose2D cur_position = robot.getFieldPosition();

        //SparkFunOTOS.Pose2D cur_position = otosSensor.getPosition();
        double x = cur_position.getX(DistanceUnit.INCH);
        double y = cur_position.getY(DistanceUnit.INCH);
        double h = cur_position.getHeading(AngleUnit.DEGREES);

        //fix heading errors...three coordinate systems!!
        // Determine the heading current error
        double headingError = target_heading - h;

        // Normalize the error to be within +/- 180 degrees to avoid wasting time with overly long turns
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0

        robot.telemetry.addLine("turn speed: " + turnSpeed);
        robot.telemetry.addLine("heading: " + h);

        double fieldXError = x_target_position - x;
        double fieldYError = y_target_position - y;

        robot.telemetry.addLine("x error field: " + fieldXError);
        robot.telemetry.addLine("y error field: " + fieldYError);

        // This code rotates the error vector from field space to robot space. The heading
        // represents how far out of alignment the field and robot space is, to undo this
        // misalignment for the error vector we negate the heading.
        double trigHeading = Math.toRadians(-h);
        double robotXError = fieldXError * Math.cos(trigHeading) - fieldYError * Math.sin(trigHeading);
        double robotYError = fieldYError * Math.cos(trigHeading) + fieldXError * Math.sin(trigHeading);

        // TO DO: figure out the field x error's impact on robot errors

        double xSpeed = 0;
        double ySpeed = 0;

        if (robotXError > 0.1) {
            xSpeed = Range.clip(robotXError * IterativeRobotParent.P_DRIVE_GAIN, 0.1, maxDriveSpeed);
        } else if (robotXError < -0.1) {
            xSpeed = Range.clip(robotXError * IterativeRobotParent.P_DRIVE_GAIN, -maxDriveSpeed, -0.1);
        }

        if (robotYError > 0.1) {
            ySpeed = Range.clip(robotYError * IterativeRobotParent.P_DRIVE_GAIN, 0.1, maxDriveSpeed);
        } else if (robotYError < -0.1) {
            ySpeed = Range.clip(robotYError * IterativeRobotParent.P_DRIVE_GAIN, -maxDriveSpeed, -0.1);
        }

        robot.telemetry.addLine("x error: " + robotXError);
        robot.telemetry.addLine("y error: " + robotYError);
        robot.telemetry.addLine("heading error: " + headingError);
        robot.telemetry.addLine("current position x: " + x);
        robot.telemetry.addLine("current position y: " + y);
        robot.telemetry.addLine("x speed: " + xSpeed);
        robot.telemetry.addLine("y speed: " + ySpeed);
        robot.telemetry.addLine("h: " + h);
        //robot.telemetry.update();

        if (robot.gamepad1.y) {
            robot.moveRobot(0, 0, 0);
            return;
        }

        // Ramp up to max driving speed over one second
        if (runtime.seconds() < 1) {
            robot.moveRobot(xSpeed * runtime.seconds(), ySpeed * runtime.seconds(), -turnSpeed * runtime.seconds());
        } else {
            robot.moveRobot(xSpeed, ySpeed, -turnSpeed); // negated turnSpeed b/c field has counterclockwise as positive
        }
        // change the field errors to robot errors here so they're all calculated with the same type??
        if (Math.abs(fieldXError) < 1 && Math.abs(fieldYError) < 1 && Math.abs(headingError) < 1) {
            robot.moveRobot(0, 0, 0);
            if (!hold) {
                finished = true;
            }
        }

         */
    }

    public boolean isFinished() {
        return finished;
    }

    public void stop() {
        robot.moveRobot(0, 0, 0);
        finished = true;
    }

    /*
    private double calculateDistance(boolean green) {
        List<ColorBlobLocatorProcessor.Blob> balls = robot.getBallDetections();
        ColorBlobLocatorProcessor.Blob nearest;
        double distance;
        double size = Double.MAX_VALUE;
        for (ColorBlobLocatorProcessor.Blob ball : balls) {
            if (ball.getContourArea() < size) {
                size = ball.getContourArea();
                nearest = ball;
            }
        }
        return distance;
    }
     */
}
