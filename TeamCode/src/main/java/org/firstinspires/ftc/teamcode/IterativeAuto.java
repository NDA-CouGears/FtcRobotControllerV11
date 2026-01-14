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

    private void tasks() {

        boolean startNear = config.startNear;
        boolean isRed = !config.blueAlliance;

        addOperation(new SetShootSpeed(startNear ? 1: 2));

        if (config.startDelay > 0) {
            addOperation(new Sleep(config.startDelay));
        }

        if (startNear) {
            // starting near logic: set near start pos and scan the obelisk (if we want)
            addOperation(new SetStartingPosition(-59, -46, 45, isRed));
            if (config.scanObelisk) {
                addOperation(new IterativeDriveToLocation(0.8, -51, -31, 35, isRed));
                addOperation(new IterativeScanObelisk());
            }
            else {
                IterativeScanObelisk.curPattern = IterativeScanObelisk.OBELISK_PATTERN.PPG;
            }
        }
        else {
            // starting far logic: set far start pos and scan the obelisk (if we want)
            addOperation(new SetStartingPosition(61,-14,90, isRed));
            if (config.scanObelisk) {
                addOperation(new IterativeScanObelisk());
            }
            else {
                IterativeScanObelisk.curPattern = IterativeScanObelisk.OBELISK_PATTERN.PPG;
            }
        }

        // drive to shoot position
        if (config.shootPos==SHOOT_NEAR) {
            if (!startNear){
                addOperation(new IterativeDriveToLocation(0.8, -24, -14, 90, isRed));
            }
            addOperation(new IterativeDriveToLocation(0.8, -51, -31, -20, isRed));
        }
        else if (config.shootPos==SHOOT_FAR){
            addOperation(new IterativeDriveToLocation(0.8,55,-14,-70, isRed));
        }

        // shoot
        shootInOrderStart(config.shootPos==SHOOT_NEAR? 1: 2);

        // intake and return to shooting position (if intake)
        if (config.intakeLine > 0) {
            intakeTasks(config.intakeLine, isRed);
            if (config.shootPos == SHOOT_NEAR) {
                addOperation(new IterativeDriveToLocation(0.8, -51, -31, -20, isRed));
                shootNum(1, 3);
            } else if (config.shootPos == SHOOT_FAR) {
                addOperation(new IterativeDriveToLocation(0.8, 55, -14, -70, isRed));
                shootNum(2, 2);
                addOperation(new IterativeDriveToLocation(0.8, 25, -14, -70, isRed));
            }
        }
        else if (config.shootPos == SHOOT_FAR){
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
            tasks();
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
