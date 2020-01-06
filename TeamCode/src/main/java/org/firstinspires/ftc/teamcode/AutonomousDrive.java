package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutonomousDrive")
//@TeleOp(name = "AutonomousDrive")
public class AutonomousDrive extends LinearOpMode {

    AnimatronicsRobot robot = new AnimatronicsRobot();
    Navigation navigation = new Navigation();
    Navigation.Position startedPosition;
    private ElapsedTime runtime = new ElapsedTime();
    private static double turnInches = 48;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.robotInit(hardwareMap, telemetry);
        robot.enableEncoders(this);
        navigation.initialize(this);

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();
        telemetry.addData(">", "Started...");
        telemetry.update();

        if (opModeIsActive()) {
            System.out.println("*************** main: started Autonomous....");
            robot.gyroDrive(this, 0.7f, -6, 0);
            robot.idleFor(this, 0.25f); // Give some time for vuforia to find target
            System.out.println("*************** main: completed 1st turn in Autonomous....");

            while(opModeIsActive()) {
                // Find where is the Robot
                Navigation.Position pos = navigation.getRobotPosition(this);
                System.out.println("*************** main: Position: " + pos);
                if (pos != null) {
                    System.out.println("*************** main: Position Name: " + pos.name);
                    startedPosition = pos;
                    // Found Perimeter wall target
                    if (pos.name.equals(Navigation.Target.BLUE_PERIMETER_1) || (pos.name.equals(Navigation.Target.FRON_PERIMETER_2))) {
                        quarySide(false);
                    } else if (pos.name.equals(Navigation.Target.BLUE_PERIMETER_2) || (pos.name.equals(Navigation.Target.REAR_PERIMETER_1))) {
                        //moveFoundation(false);
                    } else if (pos.name.equals(Navigation.Target.RED_PERIMETER_2) || (pos.name.equals(Navigation.Target.FRONT_PERIMETER_1))) {
                        quarySide(true);
                    } else if (pos.name.equals(Navigation.Target.RED_PERIMETER_1) || (pos.name.equals(Navigation.Target.REAR_PERIMETER_2))) {
                        //moveFoundation(true);
                    }
                    break;
                }
                robot.idleFor(this, 0.25f);
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

    private void moveFoundation(boolean isRed) {

        int turnSign = isRed? -1: 1;

        // Mostly found the BLUE_PERIMETER_2, go close to back wall - center of foundation...
        robot.encoderInchesDrive(this, 0.75f, turnSign*14, turnSign*14, 5);
        //robot.printMetrics();robot.idleFor(this, 5);
        // turn towards Blue wall
        robot.encoderInchesDrive(this, 0.75f, -turnInches, turnInches, 5);
        //robot.printMetrics();robot.idleFor(this, 5);


        // Raise the foundation holding bars
        robot.setFoundationPower(1.0f);
        // Go back towards foundation to engage it
        robot.encoderInchesDrive(this, 0.75f, -12, -12, 5);
        //robot.printMetrics();robot.idleFor(this, 5);
        // Down the foundation holding bars to lock the foundation
        robot.setFoundationPower(-1.0f);robot.idleFor(this, 0.15f);
        // Move close to blue wall.
        robot.encoderInchesDrive(this, 0.75f, 24, 24, 5);
        //unLock Foundation
        robot.setFoundationPower(0.3f); robot.idleFor(this, 0.25f);

/*
        // ##################### Do lateral to come out and park at line ##################### //
        robot.setFoundationPower(0.0f);
        // Do left lateral until line.....
        robot.lateralEncodeInchesDrive(this, turnSign*-1.0f, 72, 5);
        robot.setFoundationPower(-1.0f);robot.idleFor(this, 0.1f);
        robot.setFoundationPower(0.0f);
*/

        // ##################### Do lateral and push the foundation and then park at line ##################### //
        robot.setFoundationPower(0.0f);
        robot.encoderInchesDrive(this, 0.75f, 4, 4, 5);
        robot.setFoundationPower(-1.0f);
        robot.encoderInchesDrive(this, 0.75f, turnSign*-turnInches, turnSign*turnInches, 5);
        robot.setFoundationPower(0.0f);
        robot.encoderInchesDrive(this, 0.75f, 28, 28, 5);
        robot.encoderInchesDrive(this, 0.75f, turnSign*-turnInches, turnSign*turnInches, 5);
        robot.encoderInchesDrive(this, 0.75f, 40, 40, 5);
        robot.encoderInchesDrive(this, 0.75f, turnSign*-turnInches, turnSign*turnInches, 5);
        robot.encoderInchesDrive(this, 0.75f, 28, 28, 5);
        robot.encoderInchesDrive(this, 0.75f, turnSign*-turnInches, turnSign*turnInches, 5);
        robot.encoderInchesDrive(this, 0.75f, 46, 46, 5);

        robot.encoderInchesDrive(this, 0.75f, -4, -4, 5);
        robot.encoderInchesDrive(this, 0.75f, turnSign*-turnInches, turnSign*turnInches, 5);
        robot.encoderInchesDrive(this, 0.75f, 28, 28, 5);
        robot.encoderInchesDrive(this, 0.75f, turnSign*turnInches, turnSign*-turnInches, 5);
        robot.encoderInchesDrive(this, 0.75f, 24, 24, 5);
        robot.lateralEncodeInchesDrive(this, turnSign*-1.0f, 24, 5);
    }

    private void quarySide(boolean isRed) {
        int turnSign = isRed? -1: 1;

        //robot.gyroDrive(this, 0.7f, -6, 0);
        // Turn towards stones
        robot.gyroTurn(this, 0.5f, 170.0f);
        //robot.lateralEncodeInchesDrive(this, turnSign*-1.0f, 24, 5);
        robot.driveMecanumForTime(this, 0.0, turnSign*-1.0, 0.0, 0.5);
        robot.gyroTurn(this, 0.5f, 170.0f);

        // Search for 1st skystone
        searchSkystone(isRed);
/* No time for 2nd stone.... so go and park
        // Go back to start of the stones line
        robot.getClawRotateMotor().setPower(1.0f);
        robot.encoderInchesDrive(this, 0.5f, -38, -38, 5);
        robot.clawLiftTimedPower(this, -1.0f, 2.5);

        if(!isRed) { // turn camera towards stones
            robot.encoderInchesDrive(this, 0.75f, turnSign*turnInches*2, turnSign*-turnInches*2, 5);
        }

        // Search for 2nd skystone
        searchSkystone(isRed);

*/        // Goto Line
 //       robot.encoderInchesDrive(this, 1.0f, -18, -18, 5);
    }

    private void searchSkystone(boolean isRed) {
        int turnSign = isRed? -1: 1;

        robot.idleFor(this, 0.25f);
        Navigation.Position pos = null;
        for(int i = 0; i < 10; i++) {
            pos = navigation.getCameraPositionFromTraget(this, Navigation.Target.STONE_TARGET);
            if(pos != null) {
                break;
            }
            //robot.lateralEncodeInchesDrive(this, turnSign*1.0f, 12, 5);
            robot.driveMecanumForTime(this, 0.0, turnSign*1.0, 0.0, 0.25f);
            robot.gyroTurn(this, 0.5f, 180.0f);
            robot.idleFor(this, 0.25f);
        }

        if(pos != null) {
            System.out.println(String.format("*************** searchSkystone: Position Name:%s, X:%.1f, Y:%.1f, Z:%.1f, ", pos.name, pos.X, pos.Y, pos.Z));

            runtime.reset();
            robot.startStoneIntake();
            robot.driveMecanum(0.5, 0.0f, 0.0f);
            while(opModeIsActive() && runtime.seconds() < 5 && !robot.isStoneCollected()) {
                idle();
            }
            robot.stopRobot();
            robot.stopStoneIntake();
        } else {
            System.out.println("*************** searchSkystone: Nothing found.....");
        }
    }

}
