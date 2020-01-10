package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestGyro")
public class TestGyro extends LinearOpMode {
    AnimatronicsRobot robot = new AnimatronicsRobot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.robotInit(hardwareMap, telemetry);
        robot.enableEncoders(this);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        telemetry.addData(">", "Started...");
        telemetry.update();

        while(opModeIsActive()) {

            robot.manualDrive(this, gamepad1, gamepad2, telemetry);
            robot.printMetrics();
        }
    }
}
