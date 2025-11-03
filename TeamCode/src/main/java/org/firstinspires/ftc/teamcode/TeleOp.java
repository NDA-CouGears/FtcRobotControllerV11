package org.firstinspires.ftc.teamcode;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Tournament")
public class TeleOp extends RobotParent {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            mecanumDrive();
            shootArtifact();
            controlCarousel();
            intakeBall();
        }
    }
}
