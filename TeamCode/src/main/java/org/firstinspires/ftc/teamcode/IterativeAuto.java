package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "IterativeAuto", group = "Tournament")

public class IterativeAuto extends IterativeRobotParent {
    List<RobotOperation> operations = new ArrayList<RobotOperation>();
    int curOperation = -1; //start at -1 so we initialize the first operation first time through loop

    @Override
    public void init() {
        initHardware();
        initAprilTag();
        initBallCam();
        //blueTasks(false);
        addTest();
    }

    private void addTest() {
        operations.add(new SetStartingPosition(0,0,0));
        //operations.add(new IterativeScanObelisk());
        /*
        operations.add(new IterativeDriveToLocation(0.6, 10, -20, -45));
        operations.add(new IterativeOtisAprilTagCalibration());
        operations.add(new IterativeDriveToLocation(0.6, -40, -30, 135));
        operations.add(new IterativeDriveToLocation(0.6, 0, -25, 90));
        operations.add(new IterativeDriveToLocation(0.6, -20, -40, 0));
        operations.add(new IterativeDriveToLocation(0.6, 55, -15, -90));
        operations.add(new IterativeDriveToLocation(0.6,0,10,0));
        operations.add(new IterativeDriveToLocation(0.6, 10,10,0));
        operations.add(new IterativeDriveToLocation(0.6,10,0,0));
        operations.add(new IterativeDriveToLocation(0.6,0,0, 0));
        operations.add(new IterativeDriveToLocation(0.6,0,0,90));
         */
        //operations.add(new PrepareLoad(2));
        //operations.add(new PrepareLaunch(2));
        operations.add(new ScanBay(1));


    }

    private void blueTasks(boolean near) {
        operations.add(new IterativeScanObelisk());
        if (near) {
            operations.add(new SetStartingPosition(-24, -24, -117));
        }
        else {
            operations.add(new SetStartingPosition(55,-15,-90));
        }
        operations.add(new IterativeDriveToLocation(0.6,-24,-24,-45));
        operations.add(new IterativeOtisAprilTagCalibration());
        operations.add(new IterativeDriveToLocation(0.6, -48,-48,-45));
        // shoot here
        operations.add(new IterativeDriveToLocation(0.6, -55,-15,-90));
    }


    @Override
    public void loop() {
        if (stallDetection() || operations.isEmpty()){
            operations.clear();
            carousel.setPower(0);
            return;
        }
        if (curOperation == -1){
            curOperation++;
            RobotOperation robotCurOperation = operations.get(curOperation);
            robotCurOperation.init(this);

        }
        RobotOperation robotCurOperation = operations.get(curOperation);
        robotCurOperation.loop();
        if (robotCurOperation.isFinished() && (operations.size() - 1) > curOperation){
            curOperation++;
            robotCurOperation = operations.get(curOperation);
            robotCurOperation.init(this);
        }
        telemetry.addLine("position " + getFieldPosition().getX(DistanceUnit.INCH) + ", " + getFieldPosition().getY(DistanceUnit.INCH) + ", " + getFieldPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addLine(CarouselOperations.colors.get(0));
        telemetry.update();
    }

}
