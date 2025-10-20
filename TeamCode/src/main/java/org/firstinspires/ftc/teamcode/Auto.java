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
        waitForStart();
        imu.resetYaw();

        if (opModeIsActive()){
            //driveStraight(.2,-72.0,0);
            //turnToHeading(.2,-45);
            //driveToAprilTag(.2,20,25,0,0,.03);
            driveToLocation(0.3,24, 24, 0);
        }

    }
}
