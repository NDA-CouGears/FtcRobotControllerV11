package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.operations.CarouselOperations;
import org.firstinspires.ftc.teamcode.operations.IterativeDriveToLocation;
import org.firstinspires.ftc.teamcode.operations.IterativeOtisAprilTagCalibration;
import org.firstinspires.ftc.teamcode.operations.IterativeScanObelisk;
import org.firstinspires.ftc.teamcode.operations.ScanBay;
import org.firstinspires.ftc.teamcode.operations.SetStartingPosition;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "IterativeAuto", group = "Tournament")

public class IterativeAuto extends IterativeRobotParent {
    @Override
    public void init() {
        initHardware();
        initAprilTag();
        initBallCam();
        //blueTasks(false);
        addTest();
    }

    private void addTest() {
        addOperation(new SetStartingPosition(0,0,0));
        //addOperation(new IterativeScanObelisk());
        /*
        addOperation(new IterativeDriveToLocation(0.6, 10, -20, -45));
        addOperation(new IterativeOtisAprilTagCalibration());
        addOperation(new IterativeDriveToLocation(0.6, -40, -30, 135));
        addOperation(new IterativeDriveToLocation(0.6, 0, -25, 90));
        addOperation(new IterativeDriveToLocation(0.6, -20, -40, 0));
        addOperation(new IterativeDriveToLocation(0.6, 55, -15, -90));
        addOperation(new IterativeDriveToLocation(0.6,0,10,0));
        addOperation(new IterativeDriveToLocation(0.6, 10,10,0));
        addOperation(new IterativeDriveToLocation(0.6,10,0,0));
        addOperation(new IterativeDriveToLocation(0.6,0,0, 0));
        addOperation(new IterativeDriveToLocation(0.6,0,0,90));
         */
        //addOperation(new PrepareLoad(2));
        //addOperation(new PrepareLaunch(2));
        //addOperation(new ScanBay(1));
        /*
        for (int i = 1; i < 9; i++){
            addOperation(new PrepareLoad(i%3 + 1));
            addOperation(new Sleep(2));
            addOperation(new PrepareLaunch(i%3 + 1));
            addOperation(new Sleep(2));

        }

         */
        addOperation(new ScanBay(1));


    }

    private void blueTasks(boolean near) {
        addOperation(new IterativeScanObelisk());
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


    @Override
    public void loop() {
        // If there was a stall shut down the carousel motor and end the op mode
        if (stallDetection()){
            carousel.setPower(0);
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
        telemetry.addLine(CarouselOperations.colors.get(0));
        telemetry.update();
    }

}
