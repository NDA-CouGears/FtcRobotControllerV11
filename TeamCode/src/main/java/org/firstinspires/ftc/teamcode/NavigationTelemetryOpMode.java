package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Navigation Data", group = "Test")
public class NavigationTelemetryOpMode extends RobotParent {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initAprilTag();
        waitForStart();
        imu.resetYaw();

        while (opModeIsActive()) {
            showNavigationTelemetry();
        }
    }
}
