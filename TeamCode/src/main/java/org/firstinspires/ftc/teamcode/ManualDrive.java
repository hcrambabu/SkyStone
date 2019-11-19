package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ManualDrive", group="Linear Opmode")
public class ManualDrive extends LinearOpMode {

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

        while (opModeIsActive()) {
            robot.manualDrive(gamepad1, gamepad2, telemetry);
            robot.printMetrics();
        }
    }
}
