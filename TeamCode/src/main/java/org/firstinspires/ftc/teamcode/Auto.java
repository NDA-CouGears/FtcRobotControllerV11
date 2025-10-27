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
        waitForStart();
        imu.resetYaw();
        setCurrentPosition(55,-15,-90);

        while (opModeIsActive()){
            //driveToLocation(.2, 35, -15, 0);
            showNavigationTelemetry();
            driveToAprilTag(.2,20,25,0,0,.03);
        }

    }
}
