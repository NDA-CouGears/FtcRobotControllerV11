package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Auto", group = "Tournament")

public class Auto extends RobotParent{
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        initAprilTag();
        imu.resetYaw();
        setCurrentPosition(55,-15,-90);
        waitForStart();

        if (opModeIsActive()){
            //driveToLocation(.2, -20, -30, -90);
            driveToLocation(.2,55,-15,-45);
            driveToLocation(.2,0,-15,-45);
            //showNavigationTelemetry();
            //driveToAprilTag(.2,20,25,0,0,.03);
        }

    }
}
