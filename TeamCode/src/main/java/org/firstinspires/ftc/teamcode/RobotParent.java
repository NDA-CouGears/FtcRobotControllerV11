/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


/*
 */
public abstract class RobotParent extends LinearOpMode {

    protected DcMotor leftFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor rightBackDrive = null;
    protected DcMotorEx leftShoot = null;
    protected DcMotorEx rightShoot = null;
    protected DcMotorEx carousel = null;
    protected Servo carouselArm = null;
    protected DcMotorEx intakeSpinny = null;
    public double targetHeading = 0;
    public double headingError = 0;
    public double turnSpeed = 0;
    public List<LynxModule> allHubs = null;
    public IMU imu = null;// Control/Expansion Hub IMU
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal aprilTagVisionPortal;
    private SparkFunOTOS otosSensor;
    private ElapsedTime shootRunTime = new ElapsedTime();
    private boolean shootButtonPressed = false;
    private static final int MAX_SHOOT_TIME = 2;
    private static final float SHOOT_GEAR_RATIO = 3.7f;
    private static final float SHOOT_MAX_RPM = 1620f;
    private static final float SHOOT_TICKS_PER_ROTATION = 28*SHOOT_GEAR_RATIO;
    private static final double CAROUSEL_ARM_OPEN = .25;
    private static final double CAROUSEL_ARM_CLOSED = .75;
    static final double P_DRIVE_GAIN = 0.03;// Larger is more responsive, but also less stable.
    static final double P_TURN_GAIN = 0.02;// Larger is more responsive, but also less stable.
    static final double HEADING_THRESHOLD = 1.0;// How close must the heading get to the target before moving to next step.
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 102 / 25.4;
    static final double COUNTS_PER_MOTOR_REV = 483.3836858;  //NEED TO FIX DIS >:3
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159);


    public void initHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf_drive"); // control hub 2
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb_drive"); // control hub 0
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf_drive"); // control hub 3
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb_drive"); // control hub 1

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftShoot = hardwareMap.get(DcMotorEx.class, "left shoot");
        rightShoot = hardwareMap.get(DcMotorEx.class, "right shoot");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        carouselArm = hardwareMap.get(Servo.class, "carousel arm");
        intakeSpinny = hardwareMap.get(DcMotorEx.class, "intake spinny");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftShoot.setDirection(DcMotor.Direction.REVERSE);
        rightShoot.setDirection(DcMotor.Direction.FORWARD);
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }




        initOtos();


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    protected void setCurrentPosition(double x, double y, double heading) {
        otosSensor.setPosition(new SparkFunOTOS.Pose2D(y, -x, heading - 90));
    }

    /**
     * Don't access the OTOS sensor except through this
     * @return field pose as computed from the OTOS sensor
     */
    protected Pose2D getFieldPosition(){
        SparkFunOTOS.Pose2D otosPos = otosSensor.getPosition();
        double heading = otosPos.h + 90;
        double x = -otosPos.y;
        double y = otosPos.x;
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
    }

    private void initOtos() {
        otosSensor = hardwareMap.tryGet(SparkFunOTOS.class, "otos");

        if (otosSensor == null) {
            return;
        }

        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        otosSensor.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        otosSensor.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-2, -2.75, 0);
        otosSensor.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        otosSensor.setLinearScalar(.9984);
        otosSensor.setAngularScalar(0.9945);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        otosSensor.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        otosSensor.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otosSensor.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otosSensor.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    protected void initAprilTag() {

        // Create the AprilTag processor.
        aprilTagProcessor = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Tag Cam"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        // builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        // builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        // builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        // builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTagProcessor);

        // Build the Vision Portal, using the above settings.
        aprilTagVisionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        // visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    public void driveToAprilTag(double maxSpeed, int tagID, double targetForwardDistance, double targetLateralDistance, double heading, double speedGain) throws InterruptedException {
        //final double SPEED_GAIN = 0.03;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double TURN_GAIN = 0.005;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
        double retryStartTime = 0;
        int retryCount = 0;
        setManualExposureAprilTag(0,75);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && !gamepad1.y) {

            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.

            AprilTagDetection target = null;// store target AprilTag

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if (detection.id == tagID) {
                        target = detection;
                        break;
                    }
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                }
            }   // end for() loop

            if (target == null && retryStartTime == 0) {
                retryStartTime = getRuntime();
                retryCount = 0;
                moveRobot(0, 0, 0);
            }
            if (target != null) {
                retryStartTime = 0;
                retryCount = 0;
            }
            if (retryStartTime != 0) {
                while ((getRuntime() - retryStartTime) <= .1 && opModeIsActive()) {
                    Thread.sleep(10);
                    telemetry.addLine("Retrying: " + retryCount);
                    telemetry.update();
                }
                if (!opModeIsActive() || retryCount >= 2) {
                    return;
                }
                retryCount += 1;
                continue;
            }

            double yawRad = Math.toRadians(target.ftcPose.yaw);
            double errorX = target.ftcPose.x + (Math.sin(yawRad) * targetForwardDistance + Math.cos(yawRad) * targetLateralDistance);
            double errorY = target.ftcPose.y + (-Math.sin(yawRad) * targetLateralDistance - Math.cos(yawRad) * targetForwardDistance);
            //double rangeTagError = target.ftcPose.range - targetForwardDistance;

            // If we are close on all axes stop, we need to experiment to find good values
            if ((Math.abs(errorX) < 1) && (Math.abs(errorY) < 1) && (Math.abs(target.ftcPose.yaw) < 1)) {
                break;
            }

            // Use the speed and turn "gains" to calculate how we want the robot to move. These are
            // more values with best guesses that need experimentation to find good values
            double forwardDriveSpeed = Math.min(errorY * speedGain, maxSpeed);
            double lateralDriveSpeed = Math.min(errorX * speedGain, maxSpeed);
            double turnSpeed = Math.min(target.ftcPose.yaw * TURN_GAIN, maxSpeed); //getSteeringCorrection(heading, TURN_GAIN);
            telemetry.addLine(String.format("turn speed = %5.2f", turnSpeed));
            telemetry.addLine(String.format("error x = %5.2f; drive speed = %5.2f", errorX, lateralDriveSpeed));
            telemetry.addLine(String.format("error y = %5.2f;", errorY));

            if (forwardDriveSpeed < 0) {
                forwardDriveSpeed = Range.clip(forwardDriveSpeed, -maxSpeed, -0.1);
            } else if (forwardDriveSpeed > 0) {
                forwardDriveSpeed = Range.clip(forwardDriveSpeed, 0.1, maxSpeed);
            }
            if (lateralDriveSpeed < 0) {
                lateralDriveSpeed = Range.clip(lateralDriveSpeed, -maxSpeed, -0.1);
            } else if (lateralDriveSpeed > 0) {
                lateralDriveSpeed = Range.clip(lateralDriveSpeed, 0.1, maxSpeed);
            }

            if (gamepad1.b) {
                moveRobot(0, 0, 0);
            } else {
                // negate drive speeds because camera is currently at the back of the robot
                moveRobot(-lateralDriveSpeed, -forwardDriveSpeed, -turnSpeed);
            }


        }

        moveRobot(0, 0, 0);
    }
    private double signPreserveSquare(double value) {

        if (value > 0) {
            return value * value;
        } else {
            return -(value * value);
        }
    }

    protected void mecanumDrive() {
        //mecanum drive
        double max;
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = signPreserveSquare(gamepad1.left_stick_y * -0.9); // Remember, this is reversed!
        double lateral = signPreserveSquare(gamepad1.left_stick_x * 0.7); // Counteract imperfect strafing
        double yaw = (signPreserveSquare(gamepad1.right_stick_x * 1)) * 0.5;

        moveRobot(lateral, axial, yaw);
    }

    public void shootArtifact(){
        if (gamepad1.left_bumper && !shootButtonPressed){
            shootButtonPressed = true;
            shootRunTime.reset();
            float motorVel = (SHOOT_MAX_RPM/60)*SHOOT_TICKS_PER_ROTATION;
            leftShoot.setVelocity(motorVel);
            rightShoot.setVelocity(motorVel);
        }
        if (shootRunTime.seconds()>=MAX_SHOOT_TIME && shootButtonPressed){
            shootButtonPressed = false;
            leftShoot.setVelocity(0);
            rightShoot.setVelocity(0);
        }
    }

    public void intakeBall(){
        if (gamepad1.a){
            intakeSpinny.setPower(50);
        }
        else{
            intakeSpinny.setPower(0);
        }
    }

    public void controlCarousel(){
        float RPM=30;
        float carouselSpeed = (gamepad1.right_trigger - gamepad1.left_trigger);
        // carouselSpeed * (rotations per second * ticks per rotation * gear ratio)
        float carouselVelocity = carouselSpeed * (RPM/60 * 28 * 26.9f);
        carousel.setVelocity(carouselVelocity);
        if (gamepad1.dpad_up){
            carouselArm.setPosition(CAROUSEL_ARM_OPEN);
        }
        else if (gamepad1.dpad_down){
            carouselArm.setPosition(CAROUSEL_ARM_CLOSED);
        }
    }


    /**
     *
     * @param x positive is right, negative is left
     * @param y
     * @param yaw positive is clockwise, negative is counterclockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // make forward = positive y, right = positive x, and clockwise = positive yaw
        x=-x;
        // y=y;
        yaw=-yaw;

        // Calculate wheel powers.
        double leftFrontPower = y - x - yaw;
        double rightFrontPower = y + x + yaw;
        double leftBackPower = y + x - yaw;
        double rightBackPower = y - x + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);


        telemetry.addLine(String.format("X, Y, Yaw: %1.2f, %1.2f, %1.2f", x, y, yaw));

    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees to avoid wasting time with overly long turns
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void turnToHeading(double maxTurnSpeed, double heading) throws InterruptedException {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_TURN_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            if( turnSpeed < 0 ) {
                turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, -0.1);
            }
            else{
                turnSpeed = Range.clip(turnSpeed,0.1, maxTurnSpeed);
            }
            // Pivot in place by applying the turning correction
            moveRobot(0, 0, turnSpeed);
        }

        // Stop all motion;
        moveRobot(0, 0, 0);
    }
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) throws InterruptedException {
        //clear cache to get most recent encoder value(s)
        clearBulkCache();

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            ElapsedTime runtime = new ElapsedTime();
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            int newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            int newLeftBackTarget = leftBackDrive.getCurrentPosition() + moveCounts;
            int newRightFrontTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
            int newRightBackTarget = rightBackDrive.getCurrentPosition() + moveCounts;
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0, 0);

            // We are using bulk cache so the isBusy flag will be cached along with all other sensor
            // values when we accessed the encoder values above. Flush the cache no to ensure we get an
            // up to date reading.
            clearBulkCache();

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
                telemetry.addLine("turn speed: " + turnSpeed);
                telemetry.addLine("heading: " + this.getHeading());

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) {
                    turnSpeed *= -1.0;
                    telemetry.addLine("turn speed 2: " + turnSpeed);
                }

                // Ramp up to max driving speed over one second
                if (runtime.seconds() < 1){
                    moveRobot(maxDriveSpeed * runtime.seconds(), 0, turnSpeed);
                }
                else {
                    // Apply the turning correction to the current driving speed.
                    double remainingDistance = (newLeftFrontTarget - leftFrontDrive.getCurrentPosition())/COUNTS_PER_INCH;
                    double driveSpeed = remainingDistance/10 * maxDriveSpeed;
                    driveSpeed = Range.clip(driveSpeed, -maxDriveSpeed, maxDriveSpeed);
                    moveRobot(driveSpeed, 0, turnSpeed);
                }
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0, 0);

        }
    }
    public void clearBulkCache() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }
    protected boolean setManualExposureAprilTag(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (aprilTagVisionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (aprilTagVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (aprilTagVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = aprilTagVisionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = aprilTagVisionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }

    /**
     * Drive to location and heading in field coordinate system.
     *
     * @param maxDriveSpeed
     * @param x_target_position
     * @param y_target_position
     * @param target_heading
     * @throws InterruptedException
     */
    public void driveToLocation(double maxDriveSpeed,
                              double x_target_position, double y_target_position,
                              double target_heading) throws InterruptedException {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            ElapsedTime runtime = new ElapsedTime();

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive()){
                // need to update this to use getFieldPosition()
                Pose2D cur_position = getFieldPosition();
                //SparkFunOTOS.Pose2D cur_position = otosSensor.getPosition();
                double x = cur_position.getX(DistanceUnit.INCH);
                double y = cur_position.getY(DistanceUnit.INCH);
                double h = cur_position.getHeading(AngleUnit.DEGREES);

                //fix heading errors...three coordinate systems!!
                // Determine the heading current error
                headingError = target_heading - h;

                // Normalize the error to be within +/- 180 degrees to avoid wasting time with overly long turns
                while (headingError > 180) headingError -= 360;
                while (headingError <= -180) headingError += 360;

                // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
                turnSpeed = Range.clip(headingError * P_TURN_GAIN, -1, 1);


                telemetry.addLine("turn speed: " + turnSpeed);
                telemetry.addLine("heading: " + h);

                double fieldXError = x_target_position - x;
                double fieldYError = y_target_position - y;

                telemetry.addLine("x error field: " + fieldXError);
                telemetry.addLine("y error field: " + fieldYError);
                double trigHeading = -Math.toRadians(h);

                double robotXError = fieldXError * Math.cos(trigHeading) + fieldYError * Math.sin(trigHeading);
                double robotYError = fieldYError * Math.cos(trigHeading) + fieldXError * Math.sin(trigHeading);

                double xSpeed = 0;
                double ySpeed = 0;

                if (robotXError>1){
                    xSpeed = Range.clip(robotXError * P_DRIVE_GAIN, 0.1, 1);
                }
                else if (robotXError<-1){
                    xSpeed = Range.clip(robotXError * P_DRIVE_GAIN, -1, -0.1);
                }

                if (robotYError>1){
                    ySpeed = Range.clip(robotYError * P_DRIVE_GAIN, 0.1, 1);
                }
                else if (robotYError<-1){
                    ySpeed = Range.clip(robotYError * P_DRIVE_GAIN, -1, -0.1);
                }

                telemetry.addLine("x error: " + robotXError);
                telemetry.addLine("y error: " + robotYError);
                telemetry.addLine("heading error: " + headingError);
                telemetry.addLine("current position x: " + x);
                telemetry.addLine("current position y: " + y);
                telemetry.addLine("x speed: " + xSpeed);
                telemetry.addLine("y speed: " + ySpeed);
                telemetry.addLine("h: " + h);
                telemetry.update();

                if (gamepad1.y){
                    moveRobot(0,0,0);
                    continue;
                }

                // Ramp up to max driving speed over one second
                if (runtime.seconds() < 1){
                    moveRobot(xSpeed * runtime.seconds(), ySpeed * runtime.seconds(), -turnSpeed * runtime.seconds());
                }
                else{
                    moveRobot(xSpeed, ySpeed, -turnSpeed); // negated turnSpeed b/c field has counterclockwise as positive
                }
                if (Math.abs(fieldXError) < 1 && Math.abs(fieldYError) < 1 && Math.abs(headingError) < 1){
                    break;
                }
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0, 0);

        }
    }

    protected void showNavigationTelemetry() {
        List<AprilTagDetection> tags = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", tags.size());

        // Step through the list of detections and display info for each one.
        AprilTagDetection target = null;// store target AprilTag
        for (AprilTagDetection detection : tags) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("Tag XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("Tag PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("Tag RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                telemetry.addLine(String.format("Field XYZ %6.1f %6.1f %6.1f  (inch)", detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("Field PRY %6.1f %6.1f %6.1f  (deg)", detection.robotPose.getOrientation().getPitch(), detection.robotPose.getOrientation().getRoll(), detection.robotPose.getOrientation().getYaw()));
            }
        }   // end for() loop

        Pose2D cur_position = getFieldPosition();
        telemetry.addLine(String.format("OTOS Field XYZ %6.1f %6.1f %6.1f  (inch)", cur_position.getX(DistanceUnit.INCH), cur_position.getY(DistanceUnit.INCH), cur_position.getHeading(AngleUnit.DEGREES)));

        SparkFunOTOS.Pose2D otos_position = otosSensor.getPosition();
        telemetry.addLine(String.format("OTOS XYZ %6.1f %6.1f %6.1f  (inch)", otos_position.x, otos_position.y, otos_position.h));

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addLine(String.format("IMU PRY %6.1f %6.1f %6.1f  (deg)", orientation.getPitch(), orientation.getRoll(), orientation.getYaw()));

        telemetry.update();
    }
}
