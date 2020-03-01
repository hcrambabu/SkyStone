package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name = "BlueFoundation")
public class BlueFoundation extends LinearOpMode {

    AutonomousCommon autonomous = new AutonomousCommon();
    private ElapsedTime runtime = new ElapsedTime();

    AnimatronicsRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        autonomous.initialize(this);
        autonomous.setColor(false);
        this.robot = autonomous.getRobot();
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();
        telemetry.addData(">", "Started...");
        telemetry.update();

        autonomous.foundationAutonomouse();

        while(opModeIsActive()) {
            idle();
        }
    }
}