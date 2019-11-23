package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Rectangle;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AnimatronicsRobot {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor scissorsLiftMotor;
    private CRServo clawHoldingMotor;
    private CRServo clawTwistMotor;
    private DcMotor clawLiftMotor;
    private CRServo clawRotateMotor;
    private DcMotor lFoundationMotor;
    private DcMotor rFoundationMotor;
    private Servo   stoneServoMotor;

    private TouchSensor scissorsTouch;
    private TouchSensor clawTouch;
    private DistanceSensor frontDistance;
    private DistanceSensor backDistance;

    static final double WHEEL_COUNTS_PER_MOTOR_REV = 1120;
    static final double WHEEL_DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (WHEEL_COUNTS_PER_MOTOR_REV * WHEEL_DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private ElapsedTime runtime = new ElapsedTime();

    private static final double XY_WHEELS_THRESHOLD_POWER = 0.0f;
    private static final double XY_CLAW_THRESHOLD_POWER = 0.5f;

    private static final double LEFT_FRONT_ERROR_PER_REV = 0.;
    private static final double LEFT_BACK_ERROR_PER_REV = 0.0;
    private static final double RIGHT_FRONT_ERROR_PER_REV = 0.0;
    private static final double RIGHT_BACK_ERROR_PER_REV = 0.0;

    private static final double LEFT_FRONT_LATERAL_ERROR_PER_REV = 0.0;
    private static final double LEFT_BACK_LATERAL_ERROR_PER_REV = 0.0;
    private static final double RIGHT_FRONT_LATERAL_ERROR_PER_REV = 0.0;
    private static final double RIGHT_BACK_LATERAL_ERROR_PER_REV = 0.0;

    private static final double LEFT_FRONT_LATERAL_ERROR_POWER = 1.0;
    private static final double LEFT_BACK_LATERAL_ERROR_POWER = 1.0;
    private static final double RIGHT_FRONT_LATERAL_ERROR_POWER = 1.0;
    private static final double RIGHT_BACK_LATERAL_ERROR_POWER = 1.0;

    public static final double QUARTER_TURN_INCHES = 22.25;

    private MecanumDrive mecanumDrive;

    public AnimatronicsRobot() {

    }

    public void robotInit(HardwareMap hardwareMap, Telemetry telemetry) {

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_drive");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");

        scissorsLiftMotor = hardwareMap.get(DcMotor.class, "liftmotor1");
        clawLiftMotor = hardwareMap.get(DcMotor.class, "clawlift");

        clawHoldingMotor = hardwareMap.get(CRServo.class, "clawmotor");
        clawTwistMotor = hardwareMap.get(CRServo.class, "clawtwist");

        clawRotateMotor = hardwareMap.get(CRServo.class, "clawrotate");

        lFoundationMotor = hardwareMap.get(DcMotor.class, "left_foundation");
        rFoundationMotor = hardwareMap.get(DcMotor.class, "right_foundation");
        stoneServoMotor = hardwareMap.get(Servo.class, "stone_servo");

        scissorsTouch = hardwareMap.get(TouchSensor.class, "scissors_touch");
        clawTouch = hardwareMap.get(TouchSensor.class, "claw_touch");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scissorsLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lFoundationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFoundationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        scissorsLiftMotor.setDirection(DcMotor.Direction.REVERSE);

        clawHoldingMotor.setDirection(CRServo.Direction.FORWARD);
        clawTwistMotor.setDirection(CRServo.Direction.FORWARD);
        clawLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        clawRotateMotor.setDirection(CRServo.Direction.FORWARD);

        lFoundationMotor.setDirection(DcMotor.Direction.FORWARD);
        rFoundationMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    private void setWheelsSpeed(double leftForwardPower, double leftBackPower,
                                double rightForwardPower, double rightBackPower) {
        leftFrontMotor.setPower(leftForwardPower);
        rightFrontMotor.setPower(rightForwardPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
    }

    public void manualDrive(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {

        double gp1_lsy = 0, gp1_lsx = 0, gp1_rsy = 0, gp1_rsx = 0;
        if(gamepad1.left_stick_y >= XY_WHEELS_THRESHOLD_POWER ||  gamepad1.left_stick_y <= -XY_WHEELS_THRESHOLD_POWER) gp1_lsy = gamepad1.left_stick_y;
        if(gamepad1.left_stick_x >= XY_WHEELS_THRESHOLD_POWER ||  gamepad1.left_stick_x <= -XY_WHEELS_THRESHOLD_POWER) gp1_lsx = gamepad1.left_stick_x;
        if(gamepad1.right_stick_y >= XY_WHEELS_THRESHOLD_POWER ||  gamepad1.right_stick_y <= -XY_WHEELS_THRESHOLD_POWER) gp1_rsy = gamepad1.right_stick_y;
        if(gamepad1.right_stick_x >= XY_WHEELS_THRESHOLD_POWER ||  gamepad1.right_stick_x <= -XY_WHEELS_THRESHOLD_POWER) gp1_rsx = gamepad1.right_stick_x;

        double leftForwardPower = -(gp1_lsy - gp1_lsx);
        double leftBackPower = -(gp1_lsy + gp1_lsx);
        double rightForwardPower = -(gp1_rsy + gp1_rsx);
        double rightBackPower = -(gp1_rsy - gp1_rsx);

        double liftPower = gamepad1.left_trigger - gamepad1.right_trigger;
        double clawliftPower = gamepad2.left_trigger - gamepad2.right_trigger;
        double clawPower = gamepad2.left_stick_y;

        double clawrotatePower = 0;
        if(gamepad2.right_stick_y >= XY_CLAW_THRESHOLD_POWER || gamepad2.right_stick_y <= -XY_CLAW_THRESHOLD_POWER) clawrotatePower = gamepad2.right_stick_y;
        double clawtwistPower = 0;
        if(gamepad2.right_stick_x >= XY_CLAW_THRESHOLD_POWER || gamepad2.right_stick_x <= -XY_CLAW_THRESHOLD_POWER) clawtwistPower = gamepad2.right_stick_x;

        if(liftPower < 0 && scissorsTouch.isPressed()) { // Already reached to end.
            liftPower = 0;
        }

        // Send calculated power to wheels
        setWheelsSpeed(leftForwardPower, leftBackPower, rightForwardPower, rightBackPower);
        scissorsLiftMotor.setPower(liftPower);
        clawLiftMotor.setPower(clawliftPower);
        clawHoldingMotor.setPower(clawPower);
        clawRotateMotor.setPower(clawrotatePower);
        clawTwistMotor.setPower(clawtwistPower);

        double foundationPower = 0;
        if(gamepad1.dpad_up || gamepad2.dpad_up) foundationPower = 1.0f;
        else if(gamepad1.dpad_down || gamepad2.dpad_down) foundationPower = -1.0f;
        lFoundationMotor.setPower(foundationPower);
        rFoundationMotor.setPower(foundationPower);

        if(gamepad1.dpad_left || gamepad2.dpad_left) stoneServoMotor.setPosition(0.0);
        else if(gamepad1.dpad_right || gamepad2.dpad_right) stoneServoMotor.setPosition(1.0);
    }

    private void setWheelsRunMode(DcMotor.RunMode mode) {
        leftFrontMotor.setMode(mode);
        rightFrontMotor.setMode(mode);
        leftBackMotor.setMode(mode);
        rightBackMotor.setMode(mode);
    }

    public void enableEncoders() {
        setWheelsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scissorsLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setWheelsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissorsLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mecanumDrive = new MecanumDrive(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
    }


    public void driveMecanum(double forward, double strafe, double rotate) {
        mecanumDrive.driveMecanum(forward, strafe, rotate);
    }

    public void driveMecanumForTime(LinearOpMode opMode, double forward, double strafe, double rotate, double timeoutS) {
        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {
            mecanumDrive.driveMecanum(forward, strafe, rotate);
        }
        stopRobot();
    }

    public void timeDrive(LinearOpMode opMode, double leftSpeed, double rightSpeed, double timeoutS) {

        runtime.reset();
        setWheelsSpeed(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
        while (opMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {
            opMode.idle();
        }
        stopRobot();
    }

    public void stopRobot() {
        setWheelsSpeed(0, 0, 0, 0);
    }

    public void noTimeDrive(double leftSpeed, double rightSpeed) {
        setWheelsSpeed(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
    }

    public void lateralNoTimeDrive(double speed) {

        boolean right = speed > 0;
        double absSpeed = Math.abs(speed);
        if(right) {
            setWheelsSpeed(absSpeed, -absSpeed, -absSpeed, absSpeed);
        } else {
            setWheelsSpeed(-absSpeed, absSpeed, absSpeed, -absSpeed);
        }
    }

    public void lateralTimeDrive(LinearOpMode opMode, double speed, double timeoutS) {

        boolean right = speed > 0;
        double absSpeed = Math.abs(speed);

        runtime.reset();

        if(right) {
            setWheelsSpeed(absSpeed, -absSpeed, -absSpeed, absSpeed);
        } else {
            setWheelsSpeed(-absSpeed, absSpeed, absSpeed, -absSpeed);
        }

        while (opMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {
            opMode.idle();
        }
        stopRobot();
    }

    public void lateralEncodeInchesDrive(LinearOpMode opMode, double speed, double inches, double timeoutS) {
        lateralEncodePositionDrive(opMode, speed, (int) (inches * COUNTS_PER_INCH), timeoutS);
    }

    public void lateralEncodePositionDrive(LinearOpMode opMode, double speed, int position, double timeoutS) {
        boolean right = speed > 0;
        double absSpeed = Math.abs(speed);

        int newLFTarget;
        int newLBTarget;
        int newRFTarget;
        int newRBTarget;
        int lfError = (int)(position * LEFT_FRONT_LATERAL_ERROR_PER_REV);
        int lbError = (int)(position * LEFT_BACK_LATERAL_ERROR_PER_REV);
        int rfError = (int)(position * RIGHT_FRONT_LATERAL_ERROR_PER_REV);
        int rbError = (int)(position * RIGHT_BACK_LATERAL_ERROR_PER_REV);
        if(right) {
            newLFTarget = leftFrontMotor.getCurrentPosition() + position - lfError;
            newLBTarget = leftBackMotor.getCurrentPosition() - position - lbError;
            newRFTarget = rightFrontMotor.getCurrentPosition() - position - rfError;
            newRBTarget = rightBackMotor.getCurrentPosition() + position - rbError;
        } else {
            newLFTarget = leftFrontMotor.getCurrentPosition() - position - lfError;
            newLBTarget = leftBackMotor.getCurrentPosition() + position - lbError;
            newRFTarget = rightFrontMotor.getCurrentPosition() + position - rfError;
            newRBTarget = rightBackMotor.getCurrentPosition() - position - rbError;
        }

        leftFrontMotor.setTargetPosition(newLFTarget);
        leftBackMotor.setTargetPosition(newLBTarget);
        rightFrontMotor.setTargetPosition(newRFTarget);
        rightBackMotor.setTargetPosition(newRBTarget);
        setWheelsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        setWheelsSpeed(absSpeed*LEFT_FRONT_LATERAL_ERROR_POWER, absSpeed*LEFT_BACK_LATERAL_ERROR_POWER,
                absSpeed*RIGHT_FRONT_LATERAL_ERROR_POWER, absSpeed*RIGHT_BACK_LATERAL_ERROR_POWER);
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy())) {
            opMode.idle();
        }
        stopRobot();

        setWheelsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderInchesDrive(LinearOpMode opMode, double speed, double leftInches, double rightInches, double timeoutS) {

        encoderPositionDrive(opMode, speed, (int) (leftInches * COUNTS_PER_INCH), (int) (rightInches * COUNTS_PER_INCH), timeoutS);
    }

    public void encoderPositionDrive(LinearOpMode opMode, double speed, int leftPos, int rightPos, double timeoutS) {
        int newLFTarget;
        int newLBTarget;
        int newRFTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            int lfError = (int)(leftPos * LEFT_FRONT_ERROR_PER_REV);
            int lbError = (int)(leftPos * LEFT_BACK_ERROR_PER_REV);
            int rfError = (int)(rightPos * RIGHT_FRONT_ERROR_PER_REV);
            int rbError = (int)(rightPos * RIGHT_BACK_ERROR_PER_REV);

            // Determine new target position, and pass to motor controller
            newLFTarget = leftFrontMotor.getCurrentPosition() + leftPos - lfError;
            newLBTarget = leftBackMotor.getCurrentPosition() + leftPos - lbError;
            newRFTarget = rightFrontMotor.getCurrentPosition() + rightPos - rfError;
            newRBTarget = rightBackMotor.getCurrentPosition() + rightPos - rbError;
            leftFrontMotor.setTargetPosition(newLFTarget);
            leftBackMotor.setTargetPosition(newLBTarget);
            rightFrontMotor.setTargetPosition(newRFTarget);
            rightBackMotor.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            setWheelsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            double absoluteSpeed = Math.abs(speed);
            // reset the timeout time and start motion.
            runtime.reset();
            setWheelsSpeed(absoluteSpeed, absoluteSpeed, absoluteSpeed, absoluteSpeed);
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy())) {
                opMode.idle();
            }
            stopRobot();

            // Turn off RUN_TO_POSITION
            setWheelsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setClawLiftPosition(LinearOpMode opMode, double speed, int targetPosition, double timeoutS) {
        if (opMode.opModeIsActive()) {
            clawLiftMotor.setTargetPosition(targetPosition);
            clawLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            clawLiftMotor.setPower(Math.abs(speed));
            while (opMode.opModeIsActive() && clawLiftMotor.isBusy() && (runtime.seconds() < timeoutS)) {
                opMode.idle();
            }
            clawLiftMotor.setPower(0);
            clawLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void clawLiftTimedPower(LinearOpMode opMode, double speed, double timeoutS) {

        runtime.reset();
        clawLiftMotor.setPower(speed);
        while (opMode.opModeIsActive() && (runtime.seconds() < timeoutS)) {
            opMode.idle();
        }
        clawLiftMotor.setPower(0);
    }

    public void clawLiftSetPower(double speed) {
        clawLiftMotor.setPower(speed);
    }

    public void setScissorsLiftPosition(LinearOpMode opMode, double speed, int targetPosition, double timeoutS) {
        if (opMode.opModeIsActive()) {
            scissorsLiftMotor.setTargetPosition(targetPosition);
            scissorsLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            scissorsLiftMotor.setPower(Math.abs(speed));
            while (opMode.opModeIsActive() && scissorsLiftMotor.isBusy() && (runtime.seconds() < timeoutS)) {
                opMode.idle();
            }
            scissorsLiftMotor.setPower(0);
            scissorsLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setFoundationPower(double foundationPower) {
        lFoundationMotor.setPower(foundationPower);
        rFoundationMotor.setPower(foundationPower);
    }

    public void setFoundationPower(double foundationPower, boolean isLeft) {
        if(isLeft) {
            lFoundationMotor.setPower(foundationPower);
        } else {
            rFoundationMotor.setPower(foundationPower);
        }
    }

    public double getFrontDistance() {
        return frontDistance.getDistance(DistanceUnit.INCH);
    }

    public double getBackDistance() {
        return backDistance.getDistance(DistanceUnit.INCH);
    }

    public boolean isClawTouchPressed() {
        return clawTouch.isPressed();
    }


    private int lf = 0, lb = 0, rf = 0, rb = 0;
    public void printMetrics() {
        System.out.println("***************  lf: " + (leftFrontMotor.getCurrentPosition()-lf)/COUNTS_PER_INCH+
                        ", lb: " + (leftBackMotor.getCurrentPosition()-lb)/COUNTS_PER_INCH +
                        ", rf: " + (rightFrontMotor.getCurrentPosition()-rf)/COUNTS_PER_INCH +
                        ", rb: " + (rightBackMotor.getCurrentPosition()-rb)/COUNTS_PER_INCH);
        lf = leftFrontMotor.getCurrentPosition();
        lb = leftBackMotor.getCurrentPosition();
        rf = rightFrontMotor.getCurrentPosition();
        rb = rightBackMotor.getCurrentPosition();
        System.out.println("***************  lift: " + scissorsLiftMotor.getCurrentPosition() +
                ", worm: " + clawLiftMotor.getCurrentPosition());
        System.out.println("***************  front: " + getFrontDistance() +
                ", back: " + getBackDistance());
    }

    public WheelsPosition getCurrentWheelsPosition() {
        return new WheelsPosition(leftFrontMotor.getCurrentPosition(),
                leftBackMotor.getCurrentPosition(),
                rightFrontMotor.getCurrentPosition(),
                rightBackMotor.getCurrentPosition());
    }

    public double distanceTravelled(WheelsPosition start, WheelsPosition end) {

        int lfPos = end.lf - start.lf;
        int lbPos = end.lb - start.lb;
        int rfPos = end.rf - start.rf;
        int rbPos = end.rb - start.rb;

        lfPos += (int)(lfPos * LEFT_FRONT_ERROR_PER_REV);
        lbPos += (int)(lbPos * LEFT_BACK_ERROR_PER_REV);
        rfPos += (int)(rfPos * RIGHT_FRONT_ERROR_PER_REV);
        rbPos += (int)(rbPos * RIGHT_BACK_ERROR_PER_REV);

        double lfDstance = (double)lfPos / COUNTS_PER_INCH;
        double lbDstance = (double)lbPos / COUNTS_PER_INCH;
        double rfDstance = (double)rfPos / COUNTS_PER_INCH;
        double rbDstance = (double)rbPos / COUNTS_PER_INCH;

        return (lfDstance + lbDstance + rfDstance + rbDstance) / 4;
    }

    public void idleFor(LinearOpMode opMode, double sec) {
        runtime.reset();
        while (runtime.seconds() < sec) {
            opMode.idle();
        }
    }

    public CRServo getClawRotateMotor() {
        return clawRotateMotor;
    }

    public CRServo getClawHoldingMotor() {
        return clawHoldingMotor;
    }

    public CRServo getClawTwistMotor() {
        return clawTwistMotor;
    }

    public Servo getStomeServo() {
        return stoneServoMotor;
    }

    class WheelsPosition {
        public int lf, lb, rf, rb;

        public WheelsPosition(int lf, int lb, int rf, int rb) {
            this.lf = lf;
            this.lb = lb;
            this.rf = rf;
            this.rb = rb;
        }
    }
}