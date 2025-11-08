package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto", group = "Tournament")

public class Auto extends RobotParent{
    int curMenu = 0;
    double config_x = 55, config_y = -15, config_h = -90;

    protected void configMenu() throws InterruptedException {
        boolean changed = false;

        // Display current menu state
        telemetry.addLine("CONFIG MENU");
        telemetry.addLine(String.format("%s CHANGE X %2.2f", curMenu == 0?"*":"-", config_x));
        telemetry.addLine(String.format("%s CHANGE Y %2.2f", curMenu == 1?"*":"-", config_y));
        telemetry.addLine(String.format("%s CHANGE H %2.2f", curMenu == 2?"*":"-", config_h));

        // Navigate between menu options
        if (gamepad1.dpad_down) {
            curMenu++;
            changed = true;
        }
        else if (gamepad1.dpad_up) {
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

        while (!isStarted()) {
            configMenu();
            telemetry.update();
        }

        setCurrentPosition(config_x, config_y, config_h);

        while (opModeIsActive()) {
            configMenu();
            if (gamepad1.a)
                driveToLocation(.2,config_x,config_y,config_h);
            //showNavigationTelemetry();
            //driveToAprilTag(.2,20,25,0,0,.03);
            telemetry.update();
        }

    }
}
