package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Autonomous(name = "AutonomousDrive")
@TeleOp(name = "AutonomousDrive")
public class AutonomousDrive extends LinearOpMode {

    AnimatronicsRobot robot = new AnimatronicsRobot();
    Navigation navigation = new Navigation();
    String robotStartPosition;
    Navigation.Position startedPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.robotInit(hardwareMap, telemetry);
        robot.enableEncoders();
        navigation.initialize(this);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();
        telemetry.addData(">", "Started...");
        telemetry.update();

        if (opModeIsActive()) {

            // Move forward 2 feet
            robot.encoderInchesDrive(this, 0.5f, 22, 22, 5);
            // Turn Left camera towards wall
            robot.encoderInchesDrive(this, 0.5f, -24, 24, 5);
            robot.idleFor(this, 0.25f); // Give some time for vuforia to find target
            // Find where is the Robot
            Navigation.Position pos = navigation.getRobotPosition(this);
            System.out.println("*************** main: Position: "+ pos);
            if(pos != null) {
                System.out.println("*************** main: Position Name: "+ pos.name);
                startedPosition = pos;
                // Found Perimeter wall target
                if(pos.name.equals(Navigation.Target.BLUE_PERIMETER_1)) {
                    searchBlueSkystome();
                } else if(pos.name.equals(Navigation.Target.BLUE_PERIMETER_2) || (pos.name.equals(Navigation.Target.REAR_PERIMETER_1))) {
                    moveBlueFoundation();
                } else if(pos.name.equals(Navigation.Target.RED_PERIMETER_1) || (pos.name.equals(Navigation.Target.FRON_PERIMETER_2))) {
                    searchRedSkystome();
                } else if(pos.name.equals(Navigation.Target.RED_PERIMETER_2)) {
                    moveRedFoundation();
                }
            }
        }

//        while(opModeIsActive()) {
//            robot.manualDrive(gamepad1, gamepad2, telemetry);
//            navigation.getRobotPosition(this);
//        }

        while(opModeIsActive()) {
            idle();
        }
        navigation.shutdown();
    }

    private void moveBlueFoundation() {
        // Mostly found the BLUE_PERIMETER_2, so go close to it
        robot.encoderInchesDrive(this, 0.5f, 16, 16, 5);
        // turn towards Blue wall
        robot.encoderInchesDrive(this, 0.5f, -24, 24, 5);
        // Raise the foundation holding bars
        robot.setFoundationPower(1.0f);
        // Go back towards foundation to engage it
        robot.encoderInchesDrive(this, 0.5f, -12, -12, 5);
        // Down the foundation holding bars to lock the foundation
        robot.setFoundationPower(-1.0f);
        idle();
        robot.lateralEncodeInchesDrive(this, -1.0f, 48, 5);
        // Move close to blue wall.
        robot.encoderInchesDrive(this, 0.5f, 24, 24, 5);
        // Turn the foundation
        robot.encoderInchesDrive(this, 0.5f, -24, 24, 5);
        // Go back and push to wall
        robot.encoderInchesDrive(this, 0.5f, -12, -12, 5);

        robot.setFoundationPower(0.0f);

        // Go Close to Foundation - move backward - use distance sensor
        // Lock the Foundation
        // Take the foundation to Building Site - move forward - use distance sensor
    }

    private void moveRedFoundation() {
        // Go Close to left wall
        // TODO: should be go lateral or use Back wall red side Navigation? i.e REAR_PERIMETER_2
        // TODO: or Just use encoder drive with measurements
        robot.lateralTimeDrive(this, -0.5f, 2);
        // Go Close to Foundation - move backward - use distance sensor
        // Lock the Foundation
        // Take the foundation to Building Site - move forward - use distance sensor
    }

    private void searchBlueSkystome() {
        robot.encoderInchesDrive(this, 0.5f, 12, 12, 5);
        robot.encoderInchesDrive(this, 0.5f, 24, -24, 5);
        robot.encoderInchesDrive(this, 0.5f, -8, -8, 5);
        robot.encoderInchesDrive(this, 0.5f, 24, -24, 5);
        // Now search for Skystone while moving lateral right
//        robot.getRunTime().reset();
//        robot.noTimeDrive(0.2f, 0.2f);
//        Navigation.Position pos = navigation.getRobotPosition(this);
//        while((robot.getRunTime().seconds() < 4) &&
//                (pos == null || !pos.name.equals(Navigation.Target.STONE_TARGET))) { //|| pos.X < -1.0f || pos.X > 1.0f
//            if(pos != null) {
//                System.out.println("*************** searchBlueSkystome: Position Name: "+ pos.name);
//            }
//            pos = navigation.getRobotPosition(this);
//        }
//        robot.stopRobot();

        for(int i = 0; i < 12; i++) {
            Navigation.Position pos = navigation.getRobotPosition(this);
            if(pos != null && pos.name.equals(Navigation.Target.STONE_TARGET)) {
                break;
            }
            if(pos != null) {
                System.out.println("*************** searchBlueSkystome: Position Name: "+ pos.name);
            }
            robot.encoderInchesDrive(this, 0.5f, 4, 4, 5);
            robot.idleFor(this, 0.1f);
        }
        Navigation.Position pos = navigation.getRobotPosition(this);
        if(pos != null) {
            System.out.println("*************** searchBlueSkystome: Position Name: "+ pos.name);
        }

        robot.encoderInchesDrive(this, 0.5f, -24, 24, 4);
        robot.encoderInchesDrive(this, 0.5f, 12, 12, 5);

        robot.clawTimedPower(this, 1.0f, 3);
        robot.getRunTime().reset();
//        robot.getClawRotateMotor().setPower(1.0f);
//        while(robot.getRunTime().seconds() < 2) {
//            idle();
//        }
        robot.getClawRotateMotor().setPower(0.0f);
        robot.encoderInchesDrive(this, 0.5f, -6, -6, 5);
        robot.encoderInchesDrive(this, 0.5f, -24, 24, 5);
        robot.encoderInchesDrive(this, 0.5f, 96, 96, 5);



/*
        // Move backward until some distance to go to Loading.
        Navigation.Position pos = navigation.getCameraPositionFromTarget(this);
        while(pos == null || pos.Z < 24) {
            robot.noTimeDrive(-0.5f, -0.5f);
        }
        robot.stopRobot();

        // Turn the robot fully backward.
        robot.encoderInchesDrive(this, 0.5f, 24, -24, 4);

        // Now search for Skystone while moving lateral right
        pos = navigation.getCameraPositionFromTarget(this);
        while(pos == null || !pos.name.equals(Navigation.Target.STONE_TARGET) || pos.X < -1.0f || pos.X > 1.0f) {
            robot.lateralNoTimeDrive(0.5f);
        }
        robot.stopRobot();

        // Now go close to Skystone
        pos = navigation.getCameraPositionFromTarget(this);
        while(pos == null || !pos.name.equals(Navigation.Target.STONE_TARGET) || pos.Z > 4.0f) {
            robot.noTimeDrive(0.5f, 0.5f);
        }
        robot.stopRobot();
*/
        // TODO: pickup Skystone
    }

    private void searchRedSkystome() {

    }
}
