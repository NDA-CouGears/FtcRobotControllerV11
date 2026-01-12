package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.experiment.ConfigManager;
import org.firstinspires.ftc.teamcode.operations.CarouselOperations;
import org.firstinspires.ftc.teamcode.operations.DebugOperation;
import org.firstinspires.ftc.teamcode.operations.IterativeDriveToLocation;
import org.firstinspires.ftc.teamcode.operations.IterativeScanObelisk;
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
        initBallCam();
        initAprilTag();

        CarouselOperations.resetColors();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        config.displayMenu(telemetry, gamepad1);
    }

    private void addTest(boolean isRed) {
        addOperation(new SetStartingPosition(61,-14,90, isRed));
        addOperation(new IterativeDriveToLocation(0.8,41,-14, 90, isRed));
    }

    private void tasks(boolean near, boolean isRed) {
        /**
         * I think we have agreed on four phases to auto that we will get as far into as possible
         * with the time we have:
         *
         * Phase 1: done!! :)
         * - Support red/blue near and far start
         * - Drive to shooting position and shoot three balls, no regard for color
         * - Is shooting position always same as starting position? Like should we start far but
         * shoot near if our partner is not in the way and our near shots are more accurate?
         * - Look into DriveToLocation stall when it gets close
         *
         * Phase 2: phase 1 plus shoot order by color (done!! :) )
         * - Add obelisk scan and shoot correct colors to phase 1
         * - Requires a new PrepareLaunch that takes a color instead of a bay
         *
         * Phase 3: phase 2 plus intake and shoot more
         * - Add april tag otos calibration
         * - Add drive to balls and intake them
         * - Reuse shoot three balls by color from phase 2
         * - Requires
         * -- fine tuning april tag calibration, use builder.setCameraPose to match camera position
         * -- fine tuning scan bay for accuracy and consistency
         * -- adding intake operations
         *
         * Phase 4: phase three but for more rows of balls
         * - Add configuration for which lines to go for
         */
        addOperation(new SetShootSpeed(near? 1: 2));


        if (config.startDelay > 0) {
            addOperation(new Sleep(config.startDelay));
        }

        if (near) {
            addOperation(new SetStartingPosition(-59, -46, 45, isRed));
            if (config.scanObelisk) {
                addOperation(new IterativeDriveToLocation(0.8, -51, -31, 35, isRed));
                addOperation(new IterativeScanObelisk());
            }
        }
        else {
            addOperation(new SetStartingPosition(61,-14,90, isRed));
            if (config.scanObelisk) {
                addOperation(new IterativeScanObelisk());
            }
        }

        if (config.shootPos==1) {
            addOperation(new IterativeDriveToLocation(0.8, -51, -31, -20, isRed));
        }
        else if (config.shootPos==2){
            addOperation(new IterativeDriveToLocation(0.8,55,-14,-70, isRed));
        }


        //addOperation(new IterativeOtisAprilTagCalibration());
        //addOperation(new IterativeDriveToLocation(0.6, -48,-48,-45, isRed));
        // shoot here
        shootInOrderStart(config.shootPos==1? 1: 2);

        intakeTasks(config.intakeLine, isRed);
        if (config.shootPos == 1){
            addOperation(new IterativeDriveToLocation(0.8,-51,-30, -20, isRed));
            shootNum(1, 3);
        }
        else if (config.shootPos == 2){
            addOperation(new IterativeDriveToLocation(0.8, 55, -14, -70, isRed));
            shootNum(2, 2);
            addOperation(new IterativeDriveToLocation(0.8, 25, -14, -70, isRed));
        }
        addOperation(new SetShootSpeed(0));


    }

    @Override
    public void start() {
        super.start();
        if (config.testMode) {
            addTest(!config.blueAlliance);
        }
        else {
            tasks(config.startNear, !config.blueAlliance);
        }
    }

    @Override
    public void loop() {
        // If there was a stall shut down the carousel motor and end the op mode
        if (stallDetection()){
            stopCarousel();
            clearOperations();
            addOperation(new DebugOperation("stall detected"));
            //requestOpModeStop();
            return;
        }

        // Run any active operations
        operationLoop();

        // If there are no more operations shutdown this op mode
        if (noPendingOperations()) {
            //requestOpModeStop();
        }

        telemetry.update();
    }

}
