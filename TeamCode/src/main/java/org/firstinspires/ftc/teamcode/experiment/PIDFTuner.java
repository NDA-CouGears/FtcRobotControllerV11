package org.firstinspires.ftc.teamcode.experiment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.IterativeRobotParent;

import java.util.Locale;

public class PIDFTuner extends OpMode {
    private DcMotorEx leftShoot = null;
    private DcMotorEx rightShoot = null;

    public double p = 0;
    public double i = 0;
    public double d = 0;
    public double f = 0;

    double[] deltas = {10, 1, .1, .01, .001};
    int curDelta = 0;
    boolean useLeftMotor = true;

    private static final double lowSpeed = (IterativeRobotParent.SHOOT_MAX_RPM / 60f) * IterativeRobotParent.SHOOT_TICKS_PER_ROTATION;
    double [] speeds = {0.0d, lowSpeed, lowSpeed*1.2, lowSpeed};
    int curSpeed = 0;

    @Override
    public void init() {
        leftShoot = hardwareMap.get(DcMotorEx.class, "left shoot");
        rightShoot = hardwareMap.get(DcMotorEx.class, "right shoot");
        leftShoot.setDirection(DcMotor.Direction.REVERSE);
        rightShoot.setDirection(DcMotor.Direction.FORWARD);
        leftShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients coefficients = new PIDFCoefficients(p, i, d, f);
        rightShoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
        leftShoot.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }

    @Override
    public void loop() {
        // If a is pressed, switch motors, turning them both off to start
        if (gamepad1.aWasPressed()) {
            useLeftMotor = !useLeftMotor;
            leftShoot.setPower(0);
            rightShoot.setPower(0);
        }

        // If b is pressed cycle through available speeds
        if (gamepad1.bWasPressed()) {
            curSpeed++;
            curSpeed = curSpeed%speeds.length;
        }

        // If y is pressed cycle through available deltas
        if (gamepad1.yWasPressed()) {
            curDelta++;
            curDelta = curDelta%deltas.length;
        }

        // Use dpad up and down to adjust the p value
        if (gamepad1.dpadUpWasPressed()) {
            p += deltas[curDelta];
        }
        else if (gamepad1.dpadDownWasPressed()) {
            p -= deltas[curDelta];
        }

        // Use dpad left/right to adjust f value
        if (gamepad1.dpadRightWasPressed()) {
            f += deltas[curDelta];
        }
        else if (gamepad1.dpadLeftWasPressed()) {
            f -= deltas[curDelta];
        }

        // Which motor are we currently testing
        DcMotorEx curMotor = rightShoot;
        if (useLeftMotor) {
            curMotor = leftShoot;
        }

        // Update the coefficients
        PIDFCoefficients coefficients = new PIDFCoefficients(p, i, d, f);
        curMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);

        // Set the current target speed
        curMotor.setVelocity(speeds[curSpeed]);

        double readV = curMotor.getVelocity();
        telemetry.addLine(String.format(Locale.US, "Motor: %s", useLeftMotor ? "LEFT" : "RIGHT"));
        telemetry.addLine(String.format(Locale.US, "Target Speed: %.2f", speeds[curSpeed]));
        telemetry.addLine(String.format(Locale.US, "Actual Speed: %.2f", readV));
        telemetry.addLine(String.format(Locale.US, "Error Speed: %.2f", (speeds[curSpeed]-readV)));
        telemetry.addLine(String.format(Locale.US, "Delta: %f", deltas[curDelta]));
        telemetry.addLine("a = switch motors");
        telemetry.addLine("b = cycle speed 0->low->high->low->0");
        telemetry.addLine("y = cycle increment 10->1->.1->.01->.001");
        telemetry.addLine("dpad up/down = adjust P");
        telemetry.addLine("dpad left/right = adjust F");
    }
}
