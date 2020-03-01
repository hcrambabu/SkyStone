package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "JustPark")
public class JustPark extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        AutonomousCommon autonomousCommon = new AutonomousCommon();
        autonomousCommon.initialize(this);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();
        telemetry.addData(">", "Started...");
        telemetry.update();

        autonomousCommon.justPark();
    }
}
