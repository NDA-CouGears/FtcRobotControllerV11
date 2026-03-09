package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.experiment.ConfigManager;
import org.firstinspires.ftc.teamcode.operations.IterativeDriveToLocation;
import org.firstinspires.ftc.teamcode.operations.NestedQOp;
import org.firstinspires.ftc.teamcode.operations.SetShootSpeed;

import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Tournament")
public class TeleOp extends IterativeRobotParent {
    private boolean shootButtonPressed = false;
    private double shootingSpeed = 0;
    private int currentCarPos = 0;
    private boolean carButtonPressed = false;
    private boolean intakeButtonPressed = false;
    private boolean intakeOn = false;
    private boolean xPressed = false;
    private IterativeDriveToLocation holdOp;
    private NestedQOp shootHoldQueue;
    private double testShootSpeed = 0.5;
    public ConfigManager config = new ConfigManager();
    private Telemetry.Line line;


    @Override
    public void init() {
        initHardware();
        config.load();
    }

    protected void mecanumDrive() {
        //mecanum drive
        double max;
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        // just negated everything to make driving easier with launch as temporary "front"
        double axial = signPreserveSquare(gamepad1.left_stick_y * 1); // Remember, this is reversed!
        double lateral = signPreserveSquare(gamepad1.left_stick_x * -1); // Counteract imperfect strafing
        double yaw = (signPreserveSquare(gamepad1.right_stick_x)) * 0.5;

        moveRobot(lateral, axial, yaw);
    }

    public void shootArtifact() {
        if (gamepad2.left_bumper) {
            if (!shootButtonPressed) {
                shootButtonPressed = true;
                if (shootingSpeed == 0) {
                    shootingSpeed = -1;
                } else {
                    shootingSpeed = 0;
                }
            }
            setShootSpeedVar(shootingSpeed);
        }
        else if (gamepad2.right_bumper) {
            if (!shootButtonPressed) {
                shootButtonPressed = true;
                if (shootingSpeed == .35) {
                    shootingSpeed = -1;
                } else {
                    shootingSpeed = .35;
                }
            }
            setShootSpeedVar(shootingSpeed);
        }
        else {
            shootButtonPressed = false;
        }

        if (gamepad2.x && !xPressed) {
            xPressed = true;
            shootNum(shootingSpeed, 3);
        }
        else if (!gamepad2.x) {
            xPressed = false;
        }

        if (gamepad2.y){
            setShootSpeedVar(interpolate());
        }
        telemetry.addLine(String.format(Locale.US, "interpolateSpeed: %f", interpolate()));

    }

    public double interpolate(){
        Pose2D currentPos = getFieldPosition();
        double goalX = -60;
        // red goal y value
        double goalY = 60;
        if (config.blueAlliance){
            // blue goal y value: -60 (move to other side of the field)
            goalY *= -1;
        }
        // distance formula, I might have messed up :(
        double distance = Math.sqrt(Math.pow(currentPos.getX(DistanceUnit.INCH)-goalX,2)+Math.pow(currentPos.getY(DistanceUnit.INCH)-goalY,2));
        // speed as function of distance
        double speed = (distance / 84.852) * .35;

        return speed;
    }

    public void intakeBall() {
        if (gamepad2.a && !intakeButtonPressed) {
            intakeButtonPressed = true;
            intakeOn = !intakeOn;
            if (intakeOn){
                setIntakeSpeed(1);
            }
            else {
                setIntakeSpeed(0);
            }
        }
        else if (!gamepad2.a) {
            intakeButtonPressed = false;
        }
    }

    public void controlCarousel() {
        if (stalled && gamepad2.left_trigger == 0) {
            stalled = false;
        }
        if (stalled) {
            stopCarousel();
            return;
        }

        if (gamepad1.leftBumperWasPressed()){
            resetCarousel();
            setCarouselPosition(0,-2.5f);
        }
        if (gamepad1.rightBumperWasPressed()){
            resetCarousel();
            setCarouselPosition(0,2.5f);
        }

        if (gamepad2.left_trigger > .2 && !carButtonPressed){
            carButtonPressed = true;
            currentCarPos = (currentCarPos + 1) % 6;
            setCarouselPosition(currentCarPos);
        }
        else if (gamepad2.left_trigger <= .2) {
            carButtonPressed = false;
        }

        if (gamepad2.dpad_up) {
            telemetry.addLine("carousel arm open");
            liftLaunchArm();
        }
        else {
            telemetry.addLine("carousel arm closed");
            if (noPendingOperations()) {
                lowerLaunchArm();
            }
        }
    }

    @Override
    public void loop() {
        if (stallDetection()){
            stalled = true;
        }
        if (holdOp==null) {
            mecanumDrive();
        }
        shootArtifact();
        controlCarousel();
        intakeBall();
        lights();
        //shootAndHold();
        holdPositionShoot();
        emergency();
        testShotRange();

        operationLoop();

        // TEMPORARY: used to fine tune shoot speed
        super.init_loop();

        telemetry.update();
    }

    private void testShotRange(){
        telemetry.addLine("Use gamepad1 dpad to adjust near shoot speed");
        telemetry.addLine(String.format(Locale.US, "speed: %f", testShootSpeed));
        if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadDownWasPressed()) {
            testShootSpeed -= 0.05;
        }
        if (gamepad1.dpadRightWasPressed() || gamepad1.dpadUpWasPressed()) {
            testShootSpeed += 0.05;
        }

        if (gamepad1.aWasPressed()){
            setShootSpeedVar(testShootSpeed);
        }
    }

    public void holdPositionShoot(){
        /*
         - holding but shoot pressed
         - shooting but hold pressed
         */
        /*
        if (Math.abs(gamepad1.left_stick_x) > 0.1
                || Math.abs(gamepad1.left_stick_y) > 0.1
                || Math.abs(gamepad1.right_stick_x) > 0.1
                || Math.abs(gamepad1.right_stick_y) > 0.1) {
            if (shootHoldQueue!=null) shootHoldQueue.stop();
            if (holdOp!=null) holdOp.stop();
            shootHoldQueue = null;
            holdOp = null;
        }
        if (gamepad1.xWasPressed()){
            if (shootHoldQueue != null && holdOp != null){
                shootHoldQueue.stop();
                shootHoldQueue = null;
                holdOp = null;
            }
            holdOp = new IterativeDriveToLocation(0.8);
            addOperation(holdOp);
        }
        if (gamepad1.xWasReleased() && holdOp!=null){
            holdOp.stop();
        }
        */


        boolean bPressed = gamepad2.bWasPressed();
        if (bPressed && (shootHoldQueue == null)) {
            /*
            if (holdOp != null){
                holdOp.stop();
                holdOp = null;
            }
            */
            shootHoldQueue = new NestedQOp();
            shootNumQueue(1, 3, shootHoldQueue);
            // holdOp =  new IterativeDriveToLocation(0.8);
            // addOperation(new ParallelOperation(false, holdOp, shootHoldQueue));
            shootNum(interpolate(), 3);
            addOperation(new SetShootSpeed(0));
        }
        if (shootHoldQueue !=null && shootHoldQueue.isFinished()){
            holdOp = null;
            shootHoldQueue = null;
        }
    }



}
