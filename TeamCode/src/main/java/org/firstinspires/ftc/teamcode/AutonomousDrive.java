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
    private static double GYRO_ERROR = (168.0 - 180.0)/180.0;
    private static double GYRO_90 = 90.0 + (90.0 * GYRO_ERROR);
    private static double GYRO_180 = 180.0 + (180.0 * GYRO_ERROR);
    private static double GYRO_270 = 270.0 + (270.0 * GYRO_ERROR);
    private static double GYRO_360 = 360.0 + (360.0 * GYRO_ERROR);

    private static double GYRO_355 = 355.0 + (355.0 * GYRO_ERROR);

    private static boolean isPhoneFront = true;

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
            robot.gyroDrive(this, 0.7f, -12, 0);
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

        robot.gyroTurn(this, 0.5f, 0.0);
        robot.gyroDrive(this, 0.7f, -16, GYRO_355);
        robot.setFoundationPower(1.0f);
        robot.liftAndDropStone(this);
        robot.gyroDrive(this, 0.7f, 36, GYRO_355);
    }

    private void quarySide(boolean isRed) {
        int turnSign = isRed? -1: 1;

        robot.gyroDrive(this, 0.7f, -6, 0);
        // Turn towards stones
        robot.gyroTurn(this, 0.5f, GYRO_180);
        //robot.lateralEncodeInchesDrive(this, turnSign*-1.0f, 24, 5);
        robot.driveMecanumForTime(this, 0.0, turnSign*-1.0, 0.0, 0.75);
        robot.gyroTurn(this, 0.5f, GYRO_180);

        // Search for 1st skystone
        searchSkystone(isRed);
        double angleTowardsBackWall = isRed ? GYRO_90 : GYRO_270;
        robot.gyroTurn(this, 0.5f, angleTowardsBackWall);
        robot.gyroDrive(this, 0.7f, 60, angleTowardsBackWall);
        robot.gyroTurn(this, 0.5f, angleTowardsBackWall);
        goCloseToBackWall(isRed);
        moveFoundation(isRed);
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
        for(int i = 0; i < 15; i++) {
            pos = navigation.getCameraPositionFromTraget(this, Navigation.Target.STONE_TARGET);
            if(pos != null) {
                break;
            }
            //robot.lateralEncodeInchesDrive(this, turnSign*1.0f, 12, 5);
            robot.driveMecanumForTime(this, 0.0, turnSign*1.0, 0.0, 0.25f);
            if(i > 0 && i %3 == 0){
                robot.gyroDrive(this, 0.7f, -3, GYRO_180);
            }
            robot.gyroTurn(this, 0.5f, GYRO_180);
            robot.idleFor(this, 0.25f);
//            robot.gyroHold(this, 0.5, GYRO_180, 0.25);
        }

        if(pos != null) {
            System.out.println(String.format("*************** searchSkystone: Position Name:%s, X:%.1f, Y:%.1f, Z:%.1f, ", pos.name, pos.X, pos.Y, pos.Z));
        } else {
            System.out.println("*************** searchSkystone: Nothing found.....");
        }
        AnimatronicsRobot.WheelsPosition startWheelPos = robot.getCurrentWheelsPosition();
        runtime.reset();
        robot.startStoneIntake();
        robot.driveMecanum(0.5, 0.0f, 0.0f);
        while(opModeIsActive() && runtime.seconds() < 5 && !robot.isStoneCollected()) {
            idle();
        }
        robot.stopRobot();
        robot.stopStoneIntake();
        robot.getClawServo().setPower(1.0f);
        AnimatronicsRobot.WheelsPosition endWheelPos = robot.getCurrentWheelsPosition();
        // Go Back to staring of the stone search
        double distanceTravveledToFGetStone = robot.distanceTravelled(endWheelPos, startWheelPos);
        double stoneDist = pos != null ? pos.Z: 12.0;
        distanceTravveledToFGetStone = distanceTravveledToFGetStone + stoneDist - 12;
        robot.encoderInchesDrive(this, 0.75f, distanceTravveledToFGetStone, distanceTravveledToFGetStone, 5);

    }

    private void goCloseToBackWall(boolean isRed) {

        String target = isRed ? Navigation.Target.REAR_PERIMETER_2 : Navigation.Target.REAR_PERIMETER_1;
        gyroGoCloseToTarget(target, 24, 10);
    }

    private void gyroGoCloseToTarget(String target, double closeDistance, double timeout) {

        int angle = robot.getRobotAngle();
        double leftSpeed = 0.5;
        double rightSpeed = 0.5;
        double  error;
        double  steer;
        double  max;

        Navigation.Position pos = navigation.getCameraPositionFromTraget(this, target);
        runtime.reset();
        robot.setWheelsSpeed(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
        while(opModeIsActive() && runtime.seconds() < timeout &&
                (pos == null || pos.Z > closeDistance)) {

//            error = robot.getError(angle);
//            steer = robot.getSteer(error, 0.05);
//
//            leftSpeed = leftSpeed - steer;
//            rightSpeed = rightSpeed + steer;
//
//            // Normalize speeds if either one exceeds +/- 1.0;
//            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//            if (max > 1.0)
//            {
//                leftSpeed /= max;
//                rightSpeed /= max;
//            }
//            robot.setWheelsSpeed(leftSpeed, leftSpeed, rightSpeed, rightSpeed);

            robot.idleFor(this, 0.15f);
            pos = navigation.getCameraPositionFromTraget(this, target);
        }
        robot.stopRobot();
    }

}
