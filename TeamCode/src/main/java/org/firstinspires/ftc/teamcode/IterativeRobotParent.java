package org.firstinspires.ftc.teamcode;

import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.operations.ControlArm;
import org.firstinspires.ftc.teamcode.operations.IterativeDriveToLocation;
import org.firstinspires.ftc.teamcode.operations.NestedQOp;
import org.firstinspires.ftc.teamcode.operations.ParallelOperation;
import org.firstinspires.ftc.teamcode.operations.PrepareLaunch;
import org.firstinspires.ftc.teamcode.operations.PrepareLaunchColor;
import org.firstinspires.ftc.teamcode.operations.PrepareLoad;
import org.firstinspires.ftc.teamcode.operations.RobotOperation;
import org.firstinspires.ftc.teamcode.operations.ScanBay;
import org.firstinspires.ftc.teamcode.operations.SetIntakeSpeed;
import org.firstinspires.ftc.teamcode.operations.SetShootSpeed;
import org.firstinspires.ftc.teamcode.operations.Sleep;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;

public abstract class IterativeRobotParent extends OpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private List<LynxModule> allHubs = null;
    private DcMotorEx leftShoot = null;
    private DcMotorEx rightShoot = null;
    protected DcMotorEx carousel = null;
    private Servo carouselArm = null;
    private DcMotorEx intakeSpinny = null;
    private static final double CAROUSEL_ARM_OPEN = .615;
    private static final double CAROUSEL_ARM_CLOSED = .27;
    private ElapsedTime overCurrentDuration = null;
    protected boolean stalled = false;
    private SparkFunOTOS otosSensor;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal aprilTagVisionPortal;
    private PredominantColorProcessor colorSensor;

    private VisionPortal ballVisionPortal;
    public static final double P_TURN_GAIN = 0.02;// Larger is more responsive, but also less stable.
    public static final double P_DRIVE_GAIN = 0.03;// Larger is more responsive, but also less stable.
    public static final float SHOOT_GEAR_RATIO = 1f;
    public static float SHOOT_MAX_RPM = 4300f;
    public static final float SHOOT_TICKS_PER_ROTATION = 28 * SHOOT_GEAR_RATIO;
    public static final int SHOOT_NEAR = 1;
    public static final int SHOOT_FAR = 2;

    private OperationData activeOperation;

    private final LinkedList<OperationData> pendingOperations = new LinkedList<OperationData>();
    private final LinkedList<OperationData> completeOperations = new LinkedList<OperationData>();
    private final ElapsedTime operationsRunTime = new ElapsedTime();

    public static class OperationData {
        static int opCount = 0;
        public RobotOperation op;
        public int opId;
        public double startTime = -1;
        public double endTime = -1;
        public int loopCount;

        public OperationData(RobotOperation op) {
            opId = opCount++;
            this.op = op;
        }

        @NonNull
        @Override
        public String toString() {
            return(String.format(Locale.US, "%d:%s:%3.2f-%3.2f:%d",opId,op,startTime,endTime,loopCount));
        }
    }

    public void initHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf_drive"); // control hub 2
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb_drive"); // control hub 0
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf_drive"); // control hub 3
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb_drive"); // control hub 1

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        leftShoot = hardwareMap.get(DcMotorEx.class, "left shoot");
        rightShoot = hardwareMap.get(DcMotorEx.class, "right shoot");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        carouselArm = hardwareMap.get(Servo.class, "carousel arm");
        carouselArm.setPosition(CAROUSEL_ARM_CLOSED);
        intakeSpinny = hardwareMap.get(DcMotorEx.class, "intake spinny");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftShoot.setDirection(DcMotor.Direction.REVERSE);
        rightShoot.setDirection(DcMotor.Direction.FORWARD);
        carousel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        PIDFCoefficients coefficients = new PIDFCoefficients(15f, 3f, 0f, 3.5f);
        leftShoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        rightShoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);


        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }

        initOtos();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


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
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-7.5, -2.5, 90);
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
        builder.enableLiveView(true);

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

    public void initBallCam() {
        colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.7, 0.2, 0.7, -0.2))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.WHITE,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE
                )
                .build();
        ballVisionPortal = new VisionPortal.Builder()
                .enableLiveView(false)
                .addProcessor(colorSensor)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Ball Cam"))
                .build();
    }

    public PredominantColorProcessor.Result getBallAnalysis() {
        return colorSensor.getAnalysis();
    }

    public ArrayList<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }

    public void setCurrentPosition(double x, double y, double heading) {
        while (heading > 180) {
            heading -= 360;
        }
        while (heading < -180) {
            heading += 360;
        }
        otosSensor.setPosition(new SparkFunOTOS.Pose2D(x, y, heading));
    }


    /**
     * Don't access the OTOS sensor except through this
     *
     * @return field pose as computed from the OTOS sensor
     */
    public Pose2D getFieldPosition() {
        SparkFunOTOS.Pose2D otosPos = otosSensor.getPosition();
        double heading = otosPos.h;
        double x = otosPos.x;
        double y = otosPos.y;
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
    }

    protected boolean stallDetection() {
        double currentCurrent = carousel.getCurrent(CurrentUnit.AMPS);
        telemetry.addLine(String.format("current amps: %2.2f", currentCurrent));

        if (currentCurrent > 9) {
            if (overCurrentDuration == null) {
                overCurrentDuration = new ElapsedTime();
                overCurrentDuration.reset();
            } else if (overCurrentDuration.milliseconds() > 100) {
                overCurrentDuration = null;
                return true;
            }
        } else {
            overCurrentDuration = null;
        }

        return false;
    }

    public void stopCarousel() {
        carousel.setPower(0);
    }

    /**
     * @param sixths How many sixths of a rotation to move to where zero is the starting position
     *               with bay 1 facing the intake. Only goes forward, never in reverse to avoid
     *               jams. We use sixths because launch and load are 1/6 of a rotation off from
     *               each other
     */
    public void setCarouselPosition(int sixths) {
        // Ticks per revolution of carousel motor
        float CTR = 751.8f;

        int curPos = carousel.getCurrentPosition();
        float offset = curPos % CTR;
        float zero = curPos - offset;
        float targetPos = zero + sixths * CTR / 6;

        // If our target is more than a little behind our current position go the long way around
        // to prevent ball jams. Required based on hardware teams mechanical design
        if (targetPos < (curPos - 15)) {
            targetPos += CTR;
        }

        carousel.setTargetPosition((int) (targetPos + .5));
        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carousel.setPower(.5);
    }

    public void addCarouselTelemetry() {
        float CTR = 751.8f;
        int curPos = carousel.getCurrentPosition();
        float offset = curPos % CTR;
        float zero = curPos - offset;
        telemetry.addLine(String.format("curPos" + curPos));
        telemetry.addLine(String.format("offset" + offset));
        telemetry.addLine(String.format("zero" + zero));
    }

    public void liftLaunchArm() {
        carouselArm.setPosition(CAROUSEL_ARM_OPEN);
    }

    public void lowerLaunchArm() {
        carouselArm.setPosition(CAROUSEL_ARM_CLOSED);
    }

    /**
     * @param shootingSpeed 1 is for near, 2 is far, anything else is stop
     */
    public void setShootSpeed(int shootingSpeed) {
        if (shootingSpeed == 1) {
            float motorVel = (SHOOT_MAX_RPM / 60f) * SHOOT_TICKS_PER_ROTATION;
            leftShoot.setVelocity(motorVel);
            rightShoot.setVelocity(motorVel);
        } else if (shootingSpeed == 2) {
            float motorVel = 1.2f * (SHOOT_MAX_RPM / 60f) * SHOOT_TICKS_PER_ROTATION;
            leftShoot.setVelocity(motorVel);
            rightShoot.setVelocity(motorVel);
        } else {
            leftShoot.setVelocity(0);
            rightShoot.setVelocity(0);
        }
    }

    public void setIntakeSpeed(int speed) {
        if (speed == 1) {
            intakeSpinny.setPower(1);
        } else {
            intakeSpinny.setPower(0);
        }
    }

    public boolean isCarouselBusy() {
        return carousel.isBusy();
    }

    public void moveRobot(double x, double y, double yaw) {
        // make forward = positive y, right = positive x, and clockwise = positive yaw
        //x=-x;
        y = -y;
        yaw = -yaw;

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

    protected double signPreserveSquare(double value) {

        if (value > 0) {
            return value * value;
        } else {
            return -(value * value);
        }
    }

    protected boolean noPendingOperations() {
        return pendingOperations.isEmpty() && activeOperation == null;
    }

    protected void addOperation(RobotOperation newOp) {
        pendingOperations.add(new OperationData(newOp));
    }

    protected void clearOperations() {
        pendingOperations.clear();
        if (activeOperation != null){
            activeOperation.op.stop();
            activeOperation = null;
        }
    }

    public void emergency(){
        if (gamepad1.dpad_left){
            clearOperations();
        }
    }

    protected void displayOperations() {
        telemetry.addLine("----- OPERATIONS -----");
        telemetry.addLine(String.format(Locale.US, "PENDING: %d", pendingOperations.size()));
        telemetry.addLine(String.format(Locale.US, "CURRENT: %s", activeOperation));
        telemetry.addLine("COMPLETED:");
        for (OperationData op:completeOperations) {
            telemetry.addLine(op.toString());
            if (op.op instanceof NestedQOp){
                // cast the Operation Data object to NestedQOp to add NestedQOp completed ops list to telemetry
                telemetry.addLine(((NestedQOp)op.op).opsToString());
            }
        }
        telemetry.addLine("-------------------------------");
    }

    protected void operationLoop() {
        // If there is no active operation pull one from the list of pending operations
        if (activeOperation == null) {
            if (!pendingOperations.isEmpty()) {
                activeOperation = pendingOperations.removeFirst();
                activeOperation.startTime = operationsRunTime.seconds();
                activeOperation.op.init(this);
            }
        }

        // If there is an active operation, call its loop
        if (activeOperation != null) {
            activeOperation.loopCount++;
            activeOperation.op.loop();

            // If the current operation is finished stop it and clear the active operation field
            if (activeOperation.op.isFinished()) {
                // For debugging we track all operations that have been run to completion, we can
                // dump them out using displayOperations or review them in the debugger
                activeOperation.endTime = operationsRunTime.seconds();
                completeOperations.add(activeOperation);
                activeOperation.op.stop();
                activeOperation = null;
            }
        }

        // While developing and debugging show extra info about the operation queue
        displayOperations();
    }

    public void shootNum(int speed, int shoots) {
        addOperation(new SetShootSpeed(speed));
        if (shoots >= 1) {
            addOperation(new PrepareLaunch(2));
            addOperation(new Sleep(.5));
            addOperation(new ControlArm());
            addOperation(new Sleep(.3));
            if (shoots >= 2) {
                addOperation(new PrepareLaunch(3));
                addOperation(new ControlArm());
                addOperation(new Sleep(.3));
                if (shoots >= 3) {
                    addOperation(new PrepareLaunch(1));
                    addOperation(new ControlArm());
                }
            }
        }
    }

    public void shootNumQueue(int speed, int shoots, NestedQOp queue) {
        queue.addOperation(new SetShootSpeed(speed));
        if (shoots >= 1) {
            queue.addOperation(new PrepareLaunch(2));
            queue.addOperation(new ControlArm());
            queue.addOperation(new Sleep(.5));
            if (shoots >= 2) {
                queue.addOperation(new PrepareLaunch(3));
                queue.addOperation(new ControlArm());
                queue.addOperation(new Sleep(.5));
                if (shoots >= 3) {
                    queue.addOperation(new PrepareLaunch(1));
                    queue.addOperation(new ControlArm());
                    queue.addOperation(new Sleep(.5));
                }
            }
        }
    }

    public void shootInOrderStart(int speed) {
        addOperation(new SetShootSpeed(speed));

        // call prepare launch by color passing in index of pattern (e.g. index 3 for PPG is G, so it shoots bay 3 (holds G))
        addOperation(new PrepareLaunchColor(1));
        addOperation(new ControlArm());
        addOperation(new PrepareLaunchColor(2));
        addOperation(new ControlArm());
        addOperation(new PrepareLaunchColor(3));
        addOperation(new ControlArm());
    }

    public void intakeTasks(int intakeLine, boolean isRed){
        if (intakeLine == 0){
            return;
        }
        int xPos = -12;
        // stay -12 if intake line is 1
        if (intakeLine == 2) {
            xPos = 12;
        } else if (intakeLine == 3) {
            xPos = 35;
        }
        addOperation(new ParallelOperation(true,
                new IterativeDriveToLocation(0.8, xPos, -27, 180, isRed),
                new SetIntakeSpeed(1), new PrepareLoad(1)));
        for (int i = 1; i <= 3; i++) {
            addOperation(new PrepareLoad(i));
            addOperation(new ParallelOperation(true,
                    // the speed was originally .5 and the y target was -35-(5*i), just in case we want to go back
                    new IterativeDriveToLocation(0.4, xPos, -32.5 - (5*i), 180, isRed),
                    new ScanBay(i, .2, 2)));
            addOperation(new PrepareLoad(i+1));
            //addOperation(new IterativeDriveToLocation(0.5, xPos, -28 - (5*i), 180, isRed));
            //addOperation(new ScanBay(i, .2, 2));
        }
        addOperation(new SetIntakeSpeed(0));
    }


}
