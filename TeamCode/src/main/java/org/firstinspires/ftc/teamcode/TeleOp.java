package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.operations.IterativeDriveToLocation;
import org.firstinspires.ftc.teamcode.operations.NestedQOp;
import org.firstinspires.ftc.teamcode.operations.ParallelOperation;
import org.firstinspires.ftc.teamcode.operations.SetShootSpeed;

import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Tournament")
public class TeleOp extends IterativeRobotParent {
    private boolean shootButtonPressed = false;
    private int shootingSpeed = 0;
    private int currentCarPos = 0;
    private boolean carButtonPressed = false;
    private boolean intakeButtonPressed = false;
    private boolean intakeOn = false;
    private boolean xPressed = false;
    private IterativeDriveToLocation holdOp;
    private NestedQOp shootHoldQueue;


    @Override
    public void init() {
        initHardware();
    }

    protected void mecanumDrive() {
        //mecanum drive
        double max;
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        // just negated everything to make driving easier with launch as temporary "front"
        double axial = signPreserveSquare(gamepad1.left_stick_y * 0.9); // Remember, this is reversed!
        double lateral = signPreserveSquare(gamepad1.left_stick_x * -0.7); // Counteract imperfect strafing
        double yaw = (signPreserveSquare(gamepad1.right_stick_x)) * 0.5;

        moveRobot(lateral, axial, yaw);
    }

    public void shootArtifact() {
        if (gamepad2.left_bumper) {
            if (!shootButtonPressed) {
                shootButtonPressed = true;
                if (shootingSpeed == 1) {
                    shootingSpeed = 0;
                } else {
                    shootingSpeed = 1;
                }
            }
            setShootSpeed(shootingSpeed);
        }
        else if (gamepad2.right_bumper) {
            if (!shootButtonPressed) {
                shootButtonPressed = true;
                if (shootingSpeed == 2) {
                    shootingSpeed = 0;
                } else {
                    shootingSpeed = 2;
                }
            }
            setShootSpeed(shootingSpeed);
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
        if (stalled && gamepad2.right_trigger + gamepad2.left_trigger == 0) {
            stalled = false;
        }
        if (stalled) {
            stopCarousel();
            return;
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
        //shootAndHold();
        holdPositionShoot();
        emergency();

        operationLoop();

        // TEMPORARY: used to fine tune shoot speed
        super.init_loop();
        telemetry.addLine("Use gamepad1 dpad to adjust near RPM");
        telemetry.addLine(String.format(Locale.US, "Near RPM: %f", SHOOT_MAX_RPM));
        telemetry.addLine("Far RPM is near * 1.2");
        if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadDownWasPressed()) {
            SHOOT_MAX_RPM -= 10;
        }
        if (gamepad1.dpadRightWasPressed() || gamepad1.dpadUpWasPressed()) {
            SHOOT_MAX_RPM += 10;
        }

        telemetry.update();
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
            shootNum(1, 3);
            addOperation(new SetShootSpeed(0));
        }
        if (shootHoldQueue !=null && shootHoldQueue.isFinished()){
            holdOp = null;
            shootHoldQueue = null;
        }
    }



}
