package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class IterativeDriveToLocation extends RobotOperation {
    ElapsedTime runtime;
    double maxDriveSpeed;

    double x_target_position;
    double y_target_position;
    double target_heading;
    boolean finished;

    public IterativeDriveToLocation(double maxDriveSpeed,
                                    double x_target_position, double y_target_position,
                                    double target_heading) {
        this.maxDriveSpeed = maxDriveSpeed;
        this.x_target_position = x_target_position;
        this.y_target_position = y_target_position;
        this.target_heading = target_heading;

    }

    public void init(IterativeRobotParent robot) {
        super.init(robot);
        runtime = new ElapsedTime();

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxDriveSpeed = Math.abs(maxDriveSpeed);


    }

    public void loop() {
        if (finished) {
            return;
        }
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
        double turnSpeed = Range.clip(headingError * IterativeRobotParent.P_TURN_GAIN, -maxDriveSpeed, maxDriveSpeed);


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

        if (robotXError > 1) {
            xSpeed = Range.clip(robotXError * IterativeRobotParent.P_DRIVE_GAIN, 0.1, maxDriveSpeed);
        } else if (robotXError < -1) {
            xSpeed = Range.clip(robotXError * IterativeRobotParent.P_DRIVE_GAIN, -maxDriveSpeed, -0.1);
        }

        if (robotYError > 1) {
            ySpeed = Range.clip(robotYError * IterativeRobotParent.P_DRIVE_GAIN, 0.1, maxDriveSpeed);
        } else if (robotYError < -1) {
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
        if (Math.abs(fieldXError) < 1 && Math.abs(fieldYError) < 1 && Math.abs(headingError) < 1) {
            robot.moveRobot(0, 0, 0);
            finished = true;

        }


    }

    public boolean isFinished() {
        return finished;
    }

    public void stop() {
        robot.moveRobot(0, 0, 0);
        finished = true;
    }

}
