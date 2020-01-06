package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TestLeftTurn")
public class TestLeftTurn extends LinearOpMode {

    AnimatronicsRobot robot = new AnimatronicsRobot();
    
    private static double turnInches = 48;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.robotInit(hardwareMap, telemetry);
        robot.enableEncoders(this);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();
        telemetry.addData(">", "Started...");
        telemetry.update();

        if(opModeIsActive()) {
            robot.encoderInchesDrive(this, 0.75f, 24, 24, 5);
            robot.encoderInchesDrive(this, 0.75f, -turnInches, turnInches, 5);

            robot.encoderInchesDrive(this, 0.75f, 24, 24, 5);
            robot.encoderInchesDrive(this, 0.75f, -turnInches, turnInches, 5);

            robot.encoderInchesDrive(this, 0.75f, 24, 24, 5);
            robot.encoderInchesDrive(this, 0.75f, -turnInches, turnInches, 5);

            robot.encoderInchesDrive(this, 0.75f, 24, 24, 5);
            robot.encoderInchesDrive(this, 0.75f, -turnInches, turnInches, 5);
        }

        while(opModeIsActive()) {
            idle();
        }
    }
}
