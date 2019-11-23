package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MecanumDrivingOpMode", group="Linear Opmode")
@Disabled
public class MecanumDrivingOpMode extends LinearOpMode {

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
            double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            robot.driveMecanum(forward, strafe, rotate);
        }
    }
}
