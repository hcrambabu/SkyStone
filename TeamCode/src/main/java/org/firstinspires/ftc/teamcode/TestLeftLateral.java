package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestLeftLateral")
public class TestLeftLateral extends LinearOpMode {

    AnimatronicsRobot robot = new AnimatronicsRobot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.robotInit(hardwareMap, telemetry);
        robot.enableEncoders();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();
        telemetry.addData(">", "Started...");
        telemetry.update();

        if(opModeIsActive()) {
            //robot.lateralEncodeInchesDrive(this, -1.0f, 48, 10);
            //robot.lateralTimeDrive(this, -0.5, 5);
            robot.driveMecanumForTime(this, 0.0, -1.0, 0.0, 5);
        }

        while(opModeIsActive()) {
            idle();
        }
    }
}