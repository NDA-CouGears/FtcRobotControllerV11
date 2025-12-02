package org.firstinspires.ftc.teamcode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Tournament")
public class TeleOp extends IterativeRobotParent {
    @Override
    public void init() {
        initHardware();
    }

    @Override
    public void loop() {
        if (stallDetection()){
            stalled = true;
        }
        mecanumDrive();
        shootArtifact();
        controlCarousel();
        intakeBall();

        operationLoop();
    }
}
