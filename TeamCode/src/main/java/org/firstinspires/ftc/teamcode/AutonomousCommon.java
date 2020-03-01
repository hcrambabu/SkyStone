package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

public class AutonomousCommon {

    AnimatronicsRobot robot = new AnimatronicsRobot();
    StoneDetector stoneDetector = new StoneDetector();

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtimeFromStart = new ElapsedTime();
    private static double turnInches = 48;

    private static double GYRO_ERROR = (330.0 - 360.0)/360.0;
    private static double GYRO_90 = 81;//90.0 + (90.0 * GYRO_ERROR);
    private static double GYRO_180 = 163; //180.0 + (180.0 * GYRO_ERROR);
    private static double GYRO_270 = 245;//270.0 + (270.0 * GYRO_ERROR);
    private static double GYRO_360 = 328; //360.0 + (360.0 * GYRO_ERROR);

    private static double GYRO_355 = 355.0 + (355.0 * GYRO_ERROR);

    public static final double TURN_INCHES_90 = AnimatronicsRobot.TURN_INCHES_90;
    public static final double TURN_SPEED = 0.6f;
    public static final double DRIVE_SPEED = 1.0f;

    private static final double ROBOT_HALF_SIZE = 9;

    private static boolean isPhoneFront = true;

    private LinearOpMode opMode;

    private boolean isRed;
    private List<Recognition> stones = null;
    private int firstSkyStoneIndex = 2;
    private boolean isSkyStoneDetected = false;

    public void initialize(LinearOpMode opMode) {
        this.runtimeFromStart.reset();
        this.opMode = opMode;

        stoneDetector.initialize(this.opMode);
        robot.robotInit(opMode.hardwareMap, opMode.telemetry);
        robot.enableEncoders(this.opMode);
        robot.setLiftZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public AnimatronicsRobot getRobot() {
        return robot;
    }

    public boolean isRed() {
        return isRed;
    }

    public void setColor(boolean isRed) {
        this.isRed = isRed;
    }

    public List<Recognition> scanStones() {

        runtime.reset();
        List<Recognition> currStones = stoneDetector.getVisibleStones();
        if(currStones != null) {
            this.stones = currStones;
        }
        while(currStones == null && (runtime.seconds() < 3.0f)) {
            this.opMode.idle();
            currStones = stoneDetector.getVisibleStones();
            if(currStones != null) {
                this.stones = currStones;
            }
        }

        System.out.println("******************** "+ this.stones);

        if(this.stones == null) {
            System.out.println("******************** This is disaster....., could not find any stones......");
            return null;
        }

        if(this.stones.get(0).getLeft() > 400) {
            isRed = false;
        } else {
            isRed = true;
        }


        for(int i = 0; i < this.stones.size(); i++) {
            Recognition rec = this.stones.get(i);
            if(rec.getLabel().equalsIgnoreCase(StoneDetector.LABEL_SECOND_ELEMENT)) {
                firstSkyStoneIndex = isRed ? (this.stones.size()-1-i) : i;
                break;
            }
        }

        if(stones.size() > 1 || firstSkyStoneIndex != 0) {
            isSkyStoneDetected = true;
        }

        return this.stones;
    }

    private void confirmSkyStoneIndex() {

        if(isSkyStoneDetected) {
            return;
        }

        runtime.reset();
        List<Recognition> currStones = stoneDetector.getVisibleStones();
        while(currStones == null && (runtime.seconds() < 2.0f)) {
            this.opMode.idle();
            currStones = stoneDetector.getVisibleStones();
        }

        if(currStones == null) {
            return;
        }

        firstSkyStoneIndex = 1;
        for(int i = 0; i < currStones.size(); i++) {
            Recognition rec = currStones.get(i);
            if(rec.getLabel().equalsIgnoreCase(StoneDetector.LABEL_SECOND_ELEMENT)) {
                firstSkyStoneIndex = isRed ? (this.stones.size()-1-i) : i;
                break;
            }
        }
    }

    public void justPark() {

        robot.driveMecanumForTime(this.opMode, DRIVE_SPEED, 0.0, 0.0, 0.3f);
    }

    public void quarrySideOnlyAutonomous() {
        int turnSign = isRed? -1: 1;
        fullFullAutonomous(false);
    }

    public void quarrySideFullAutonomous() {
        int turnSign = isRed ? -1 : 1;
        fullFullAutonomous(true);
    }

    private void fullFullAutonomous(boolean shouldMoveFoundation) {
        int turnSign = isRed ? -1 : 1;
        boolean isFoundationDone = false;

        this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, 18, 18, 2.0f);
        confirmSkyStoneIndex();

        double stoneAngle = turnToStoneAngle();
        if(firstSkyStoneIndex > 0) {
            this.robot.gyroTurn(this.opMode, TURN_SPEED, stoneAngle);
        }

        boolean isStonePicked = pickupStone(true, -4);
        this.robot.gyroTurn(this.opMode, TURN_SPEED, -GYRO_90*turnSign);
        if(isStonePicked) {
            double goToLineDistance = 24;
            this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, -goToLineDistance, -goToLineDistance, 2.0f);
            this.robot.gyroTurn(this.opMode, TURN_SPEED, -GYRO_90 * turnSign);

            double backFoundationDistance = 44;
            this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, -backFoundationDistance, -backFoundationDistance, 2.0f);
            if(shouldMoveFoundation) {
                this.robot.gyroTurn(this.opMode, TURN_SPEED, -GYRO_180*turnSign);
                moveFoundation(false);
                isFoundationDone = true;
            }
            dropStone();
            this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, 18, 18, 2.0f, 0.0f);
            closelift();
            this.robot.gyroTurn(this.opMode, TURN_SPEED, -GYRO_90*turnSign);
            this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, 18, 18, 2.0f);
            this.robot.gyroTurn(this.opMode, TURN_SPEED, -GYRO_90*turnSign);

            this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, 48+ROBOT_HALF_SIZE, 48+ROBOT_HALF_SIZE, 3.0f);

        } else {
            this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, 24+ROBOT_HALF_SIZE, 24+ROBOT_HALF_SIZE, 2.0f);

        }
        this.robot.gyroTurn(this.opMode, TURN_SPEED, stoneAngle);
        isStonePicked = false;
        double secondStonePickupCorrection = 0;
        if(isRed) {
            secondStonePickupCorrection = 4;
        } else {
            secondStonePickupCorrection = -4;
        }
        isStonePicked = pickupStone(true, secondStonePickupCorrection);
        this.robot.gyroTurn(this.opMode, TURN_SPEED, -GYRO_90*turnSign);
        if(isStonePicked) {
            double goToLineDistance = 48;
            this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, -goToLineDistance, -goToLineDistance, 3.0f);
            this.robot.gyroTurn(this.opMode, TURN_SPEED, -GYRO_90 * turnSign);
            double backFoundationDistance = 48;
            this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, -backFoundationDistance, -backFoundationDistance, 3.0f);
            if(shouldMoveFoundation && !isFoundationDone) {
                this.robot.gyroTurn(this.opMode, TURN_SPEED, -GYRO_180*turnSign);
                moveFoundation(false);
                isFoundationDone = true;
            }
            dropStone();
            this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, 18, 18, 2.0f, 0.0f);
            closelift();
            this.robot.gyroTurn(this.opMode, TURN_SPEED, -GYRO_90*turnSign);
            this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, 24, 24, 2.0f);
        } else {
            this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED,
                    -(48), -(48), 3.0f);
            // TODO: Not able to pickup 2nd SkyStone..... So no need to drop

            if(shouldMoveFoundation && !isFoundationDone) {
                this.robot.gyroTurn(this.opMode, TURN_SPEED, -GYRO_90 * turnSign);
                this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, -48, -48, 3.0f);
                this.robot.gyroTurn(this.opMode, TURN_SPEED, -GYRO_180*turnSign);
                moveFoundation(false);
                isFoundationDone = true;
                this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, 48, 48, 3.0f);
            }
        }
        isStonePicked = false;
    }

    public void foundationAutonomouse() {
        int turnSign = isRed? -1: 1;

        this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, -24, -24, 2.0f);
        this.robot.lateralEncodeInchesDrive(this.opMode, 1.0f * turnSign, 9.0f, 2.0f);
        this.robot.gyroTurn(this.opMode, TURN_SPEED, 0);
        moveFoundation(true);
        this.robot.lateralEncodeInchesDrive(this.opMode, 1.0f * turnSign, 24.0f, 2.0f);
        this.robot.gyroTurn(this.opMode, TURN_SPEED, GYRO_90*turnSign);
        this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, 42, 42, 2.0f);
    }

    private void moveFoundation(boolean isFoundationOnly) {
        int turnSign = isRed? -1: 1;
        int foundationSign = isFoundationOnly ? 1 : -1;

        this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, -12, -12, 2.0f);
        robot.setFoundationPosition(1.0f);
        try{Thread.sleep(800);} catch (Exception ex){}
        this.robot.encoderInchesDrive(this.opMode, 1.0, -4*turnSign, 4*turnSign, 1.0f);
        this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, 20, 20, 2.0f);
        this.robot.gyroTurn(this.opMode, TURN_SPEED, GYRO_90*turnSign*foundationSign);
        robot.setFoundationPosition(0.0f);
        this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, -18, -18, 2.0f);
    }

    private void dropStone() {
        robot.getStoneMoverServo().setPosition(1);
        robot.setLiftPower(1.0f);
        robot.idleFor(this.opMode, 0.5);
        robot.getStoneHolderServo().setPosition(0.0);
    }

    private void closelift() {
        robot.setLiftPower(0.0f);
        robot.getStoneMoverServo().setPosition(0);
        robot.idleFor(this.opMode, 0.2);
    }

    private double turnToStoneAngle() {
        int turnSign = isRed? -1: 1;
        double turnAngle = 0;
        if(firstSkyStoneIndex == 1) {
            turnAngle =  -15 * turnSign;
        } else if(firstSkyStoneIndex == 2) {
            turnAngle =  -35 * turnSign;
        }
        return turnAngle;
    }

    public boolean pickupStone(boolean comeToOriginal, double correction) {

        boolean isStonePicked = false;

        AnimatronicsRobot.WheelsPosition pos_1 = this.robot.getCurrentWheelsPosition();
        this.robot.startStoneIntake();
        this.robot.setWheelsSpeed(0.5, 0.5, 0.5, 0.5);
        runtime.reset();
        while((!this.robot.isStoneCollected()) && runtime.seconds() < 2.0f) {
            this.opMode.idle();
        }
        this.robot.stopRobot();
        isStonePicked = this.robot.isStoneCollected();
        if(isStonePicked) {
            this.robot.stopStoneIntake();
            robot.getStoneHolderServo().setPosition(1.0f);
        } else {
            this.robot.spitStoneOut();
        }
        AnimatronicsRobot.WheelsPosition pos_2 = this.robot.getCurrentWheelsPosition();
        double distaceTravelled = this.robot.distanceTravelled(pos_1, pos_2) + correction;
        if(comeToOriginal) {
            this.robot.encoderInchesDrive(this.opMode, DRIVE_SPEED, -distaceTravelled, -distaceTravelled, 2.0f);
        }
        this.robot.stopStoneIntake();

        return isStonePicked;
    }
}
