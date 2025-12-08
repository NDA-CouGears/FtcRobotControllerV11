package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.experiment.ConfigManager;
import org.firstinspires.ftc.teamcode.operations.CarouselOperations;
import org.firstinspires.ftc.teamcode.operations.ControlArm;
import org.firstinspires.ftc.teamcode.operations.IterativeDriveToLocation;
import org.firstinspires.ftc.teamcode.operations.IterativeOtisAprilTagCalibration;
import org.firstinspires.ftc.teamcode.operations.IterativeScanObelisk;
import org.firstinspires.ftc.teamcode.operations.ParallelOperation;
import org.firstinspires.ftc.teamcode.operations.PrepareLaunch;
import org.firstinspires.ftc.teamcode.operations.PrepareLoad;
import org.firstinspires.ftc.teamcode.operations.ScanBay;
import org.firstinspires.ftc.teamcode.operations.SetShootSpeed;
import org.firstinspires.ftc.teamcode.operations.SetStartingPosition;
import org.firstinspires.ftc.teamcode.operations.Sleep;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "IterativeAuto", group = "Tournament")

public class IterativeAuto extends IterativeRobotParent {
    public ConfigManager config = new ConfigManager();

    @Override
    public void init() {
        config.load();

        initHardware();
        initAprilTag();
        initBallCam();

        CarouselOperations.resetColors();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        config.displayMenu(telemetry, gamepad1);
    }

    private void addTest() {
        addOperation(new PrepareLoad(1));
        addOperation(new Sleep(2));
        addOperation(new PrepareLoad(2));
        addOperation(new Sleep(2));
        addOperation(new PrepareLoad(3));
        addOperation(new Sleep(2));
        addOperation(new PrepareLaunch(1));
        addOperation(new Sleep(2));
        addOperation(new PrepareLaunch(2));
        addOperation(new Sleep(2));
        addOperation(new PrepareLaunch(3));
        addOperation(new Sleep(2));
    }

    private void blueTasks(boolean near) {
        /**
         * I think we have agreed on four phases to auto that we will get as far into as possible
         * with the time we have:
         *
         * Phase 1:
         * - Support red/blue near and far start
         * - Drive to shooting position and shoot three balls, no regard for color
         * - Is shooting position always same as starting position? Like should we start far but
         * shoot near if our partner is not in the way and our near shots are more accurate?
         *
         * Phase 2: phase 1 plus shoot order by color
         * - Add obelisk scan and shoot correct colors to phase 1
         * - Requires a new PrepareLaunch that takes a color instead of a bay
         *
         * Phase 3: phase 2 plus intake and shoot more
         * - Add april tag otis calibration
         * - Add drive to balls and intake them
         * - Reuse shoot three balls by color from phase 2
         * - Requires
         * -- fine tuning april tag calibration
         * -- fine tuning scan bay for accuracy and consistency
         * -- adding intake operations
         *
         * Phase 4: phase three but for more rows of balls
         * - Add configuration for which lines to go for
         */
        addOperation(new IterativeScanObelisk());
        if (config.startDelay > 0) {
            addOperation(new Sleep(config.startDelay));
        }
        if (near) {
            addOperation(new SetStartingPosition(-24, -24, -117));
        }
        else {
            addOperation(new SetStartingPosition(55,-15,-90));
        }
        addOperation(new IterativeDriveToLocation(0.6,-24,-24,-45));
        addOperation(new IterativeOtisAprilTagCalibration());
        addOperation(new IterativeDriveToLocation(0.6, -48,-48,-45));
        // shoot here
        addOperation(new IterativeDriveToLocation(0.6, -55,-15,-90));
    }

    private void redTasks(boolean near) {
        /**
         * Since red is just a mirror of blue around the X axis ee should be able to make red
         * operations programmatically from blue by:
         * x = x
         * y = -y
         * h = 180-h
         */
    }

    @Override
    public void start() {
        super.start();
        if (config.testMode) {
            addTest();
        }
        else if (config.blueAlliance) {
            blueTasks(config.startNear);
        }
        else {
            redTasks(config.startNear);
        }
    }

    @Override
    public void loop() {
        // If there was a stall shut down the carousel motor and end the op mode
        if (stallDetection()){
            stopCarousel();
            clearOperations();
            requestOpModeStop();
            return;
        }

        // Run any active operations
        operationLoop();

        // If there are no more operations shutdown this op mode
        if (noPendingOperations()) {
            requestOpModeStop();
        }

        telemetry.addLine("position " + getFieldPosition().getX(DistanceUnit.INCH) + ", " + getFieldPosition().getY(DistanceUnit.INCH) + ", " + getFieldPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addLine(String.format("Bay 1:%s 2:%s 3:%s", CarouselOperations.colors.get(0),CarouselOperations.colors.get(1),CarouselOperations.colors.get(2)));
        telemetry.update();
    }

}
