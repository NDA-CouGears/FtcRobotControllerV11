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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "Hardware Test", group = "Hardware")
public class HardwareTest extends RobotParent {
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* testServo = hardwareMap.get(Servo.class, "carousel arm");
        motor1 = hardwareMap.tryGet(DcMotorEx.class, "motor one");
        motor2 = hardwareMap.tryGet(DcMotorEx.class, "motor two"); */

        initHardware();

        /* if (motor1 != null && motor2 != null) {
            motor1.setDirection(DcMotorSimple.Direction.FORWARD);
            motor2.setDirection(DcMotorSimple.Direction.REVERSE);

            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } */

        // Wait for the game to start (driver presses START)
        waitForStart();

        double positionA = 0, positionB = 0, curPos = 0;
        boolean configMode = true;
        boolean useEncoders = true;
        boolean yButtonPressed = false;
        boolean xButtonPressed = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (useEncoders) {
                leftShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                leftShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }


            if (gamepad1.y) {
                if (!yButtonPressed) {
                    configMode = !configMode;
                    yButtonPressed = true;
                }
            } else {
                yButtonPressed = false;
            }
            if (gamepad1.x) {
                if (!xButtonPressed) {
                    useEncoders = !useEncoders;
                    xButtonPressed = true;
                }
            } else {
                xButtonPressed = false;
            }

            if (configMode) {
                telemetry.addLine("CONFIG MODE");
                telemetry.addLine(String.format("Cur Position: %1.2f", curPos));
                telemetry.addLine(String.format("A: %1.2f; B: %1.2f", positionA, positionB));

                curPos += gamepad1.left_stick_y / 50.0;
                curPos = Range.clip(curPos, 0, 1);

                carouselArm.setPosition(curPos);
                if (gamepad1.a) {
                    positionA = curPos;
                }
                if (gamepad1.b) {
                    positionB = curPos;
                }
            } else {
                telemetry.addLine("RUN MODE");
                telemetry.addLine(String.format("Using encoders %b", useEncoders));
                telemetry.addLine(String.format("A: %1.2f; B: %1.2f", positionA, positionB));
                if (gamepad1.a) {
                    carouselArm.setPosition(positionA);
                }
                if (gamepad1.b) {
                    carouselArm.setPosition(positionB);
                }
                if (gamepad1.dpad_up) {
                    hackCarousel(30);
                }
                if (gamepad1.dpad_right) {
                    hackCarousel(60);
                }
                if (gamepad1.dpad_down) {
                    hackCarousel(120);
                }
                if (gamepad1.dpad_left) {
                    hackCarousel(240);
                }

                telemetry.addLine(String.format("Positions 1: %d; 2: %d", leftShoot.getCurrentPosition(), rightShoot.getCurrentPosition()));
                telemetry.addLine(String.format("Power 1: %1.2f; 2: %1.2f", leftShoot.getPower(), rightShoot.getPower()));

                leftShoot.setPower(gamepad1.left_stick_y);
                rightShoot.setPower(gamepad1.left_stick_y);

            }

            telemetry.update();
        }
    }

    public void hackCarousel(float RPM) {
        // carouselSpeed * (rotations per second * ticks per rotation * gear ratio)
        float carouselVelocity = (RPM / 60 * 28 * 26.9f);

        int targetPos = carousel.getCurrentPosition() - 251;
        carousel.setTargetPosition(targetPos);

        carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carousel.setVelocity(carouselVelocity);

        while (carousel.isBusy() && (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)) {
        }

        carousel.setPower(0);
        carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left) {
        }
    }
}
