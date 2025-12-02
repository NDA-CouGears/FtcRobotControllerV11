package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Tournament")
public class TeleOp extends IterativeRobotParent {
    private boolean shootButtonPressed = false;
    private int shootingSpeed = 0;
    private final ElapsedTime shootRunTime = new ElapsedTime();

    @Override
    public void init() {
        initHardware();
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
        } else if (gamepad2.right_bumper) {
            if (!shootButtonPressed) {
                shootButtonPressed = true;
                if (shootingSpeed == 2) {
                    shootingSpeed = 0;
                } else {
                    shootingSpeed = 2;
                }
            }
        } else {
            shootButtonPressed = false;
        }

        if (shootingSpeed == 1) {
            shootRunTime.reset();
            float motorVel = (SHOOT_MAX_RPM / 60) * SHOOT_TICKS_PER_ROTATION;
            leftShoot.setVelocity(motorVel);
            rightShoot.setVelocity(motorVel);
        } else if (shootingSpeed == 2) {
            shootRunTime.reset();
            float motorVel = 1.25f * (SHOOT_MAX_RPM / 60) * SHOOT_TICKS_PER_ROTATION;
            leftShoot.setVelocity(motorVel);
            rightShoot.setVelocity(motorVel);
        } else {
            leftShoot.setVelocity(0);
            rightShoot.setVelocity(0);
        }
        /*if (shootRunTime.seconds()>=MAX_SHOOT_TIME && shootButtonPressed){
            shootButtonPressed = false;
            leftShoot.setVelocity(0);
            rightShoot.setVelocity(0);
        } */
    }

    public void intakeBall() {
        if (gamepad2.a) {
            intakeSpinny.setPower(50);
        } else {
            intakeSpinny.setPower(0);
        }
    }

    public void controlCarousel() {
        if (stalled && gamepad2.right_trigger + gamepad2.left_trigger == 0) {
            stalled = false;
        }
        if (stalled) {
            carousel.setPower(0);
            return;
        }

        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        float RPM = 30;
        float carouselSpeed = (gamepad2.right_trigger - gamepad2.left_trigger);
        // carouselSpeed * (rotations per second * ticks per rotation * gear ratio)
        float carouselVelocity = carouselSpeed * (RPM / 60 * 28 * 26.9f);
        carousel.setVelocity(carouselVelocity);


        if (gamepad2.dpad_up) {
            telemetry.addLine("carousel arm open");
            carouselArm.setPosition(CAROUSEL_ARM_OPEN);
        } else if (carouselArm.getPosition() == CAROUSEL_ARM_OPEN) {
            telemetry.addLine("carousel arm closed");
            carouselArm.setPosition(CAROUSEL_ARM_CLOSED);
        }

        telemetry.update();
    }

    @Override
    public void loop() {
        if (stallDetection()){
            stalled = true;
        }
        mecanumDrive();
        shootArtifact();
        controlCarousel();
        intakeBall();

        operationLoop();
    }
}
