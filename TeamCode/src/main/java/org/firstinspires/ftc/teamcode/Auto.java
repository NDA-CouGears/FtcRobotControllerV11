package org.firstinspires.ftc.teamcode;

//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto", group = "Tournament")

public class Auto extends RobotParent{
    int curMenu = 0;
    double config_x = 0, config_y = 0, config_h = 0;

    protected void configMenu(String heading) throws InterruptedException {
        boolean changed = false;

        // Display current menu state
        telemetry.addLine(heading);
        telemetry.addLine("dpad.y to select, left stick.y to change value");
        telemetry.addLine(String.format("%s CHANGE X %2.2f", curMenu == 0?"*":"-", config_x));
        telemetry.addLine(String.format("%s CHANGE Y %2.2f", curMenu == 1?"*":"-", config_y));
        telemetry.addLine(String.format("%s CHANGE H %2.2f", curMenu == 2?"*":"-", config_h));

        // Navigate between menu options
        if (gamepad1.dpad_down) {
            curMenu++;
            changed = true;
        }
        else if (gamepad1.dpad_up && curMenu > 0) {
            curMenu--;
            changed = true;
        }
        curMenu = curMenu % 3;

        // Update selected value
        double delta = 0;
        if (gamepad1.left_stick_y > 0.5) {
            delta = -1;
            changed = true;
        }
        else if (gamepad1.left_stick_y < -0.5) {
            delta = 1;
            changed = true;
        }

        // Apply any changes
        switch (curMenu) {
            case 0:
                config_x += delta;
                break;
            case 1:
                config_y += delta;
                break;
            case 2:
                config_h += delta;
                break;
        }

        // If there were changes pause briefly to make them not move to fast. We could do this for
        // each specific value with debouncing but too lazy
        if (changed)
            Thread.sleep(100);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        initAprilTag();
        imu.resetYaw();

        // Starting field pose for this run
        config_x = 55;
        config_y = -15;
        config_h = -90;

        while (!isStarted()) {
            configMenu("Set Starting Pose");
            telemetry.update();
        }

        setCurrentPosition(config_x, config_y, config_h);

        // Target field pose
        config_x = 35;
        config_y = -30;
        config_h = -90;

        while (opModeIsActive()) {
            driveToLocation(0.6, -24, -24, 135);
            Thread.sleep(500);
            driveToLocation(0.6, -48, -48, 135);
            Thread.sleep(500);

            driveToLocation(0.6, 0, -48, -180);
            Thread.sleep(500);
            driveToLocation(0.6, 55, -15, -90);

            telemetry.update();
        }

    }
}
