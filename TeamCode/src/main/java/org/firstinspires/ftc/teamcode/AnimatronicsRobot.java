package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Rectangle;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AnimatronicsRobot {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor liftMotor;

    private DcMotor leftCollector;
    private DcMotor rightCollector;

    private DcMotor foundationMotor;

    private Servo   clawTwistServo;
    private CRServo clawServo;

    private DistanceSensor distanceSensor;
    private ModernRoboticsI2cGyro gyroSensor;

    static final double WHEEL_COUNTS_PER_MOTOR_REV = 1120;
    static final double WHEEL_DRIVE_GEAR_REDUCTION = 45.0f/80.0f;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (WHEEL_COUNTS_PER_MOTOR_REV * WHEEL_DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    private ElapsedTime runtime = new ElapsedTime();

    private static final double XY_WHEELS_THRESHOLD_POWER = 0.0f;

    private MecanumDrive mecanumDrive;

    private boolean holdTheStone = false;

    public AnimatronicsRobot() {

    }

    public void robotInit(HardwareMap hardwareMap, Telemetry telemetry) {

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_drive");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftback_drive");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightback_drive");

        leftCollector = hardwareMap.get(DcMotor.class, "collect1");
        rightCollector = hardwareMap.get(DcMotor.class, "collect2");

        liftMotor = hardwareMap.get(DcMotor.class, "lift");
        foundationMotor = hardwareMap.get(DcMotor.class, "foundation");

        clawTwistServo = hardwareMap.get(Servo.class, "clawtwist");
        clawServo = hardwareMap.get(CRServo.class, "claw");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        gyroSensor = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //foundationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightCollector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        foundationMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        clawServo.setDirection(DcMotor.Direction.FORWARD);

        leftCollector.setDirection(DcMotor.Direction.FORWARD);
        rightCollector.setDirection(DcMotor.Direction.REVERSE);
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

        setWheelsSpeed(leftForwardPower, leftBackPower, rightForwardPower, rightBackPower);

        if(gamepad2.x) {
            clawTwistServo.setPosition(0.0f);
        } else if(gamepad2.y) {
            clawTwistServo.setPosition(1.0f);
        }

        if(gamepad2.a) holdTheStone = true;
        if(gamepad2.b) holdTheStone = false;

        if(holdTheStone) {
            clawServo.setPower(1.0f);
        } else {
            clawServo.setPower(-1.0f);
        }

        double liftPower = gamepad1.left_trigger - gamepad1.right_trigger;
        liftMotor.setPower(liftPower);

        if(gamepad1.dpad_up || gamepad2.dpad_up){
            foundationMotor.setPower(1.0f);
        }
        else if(gamepad1.dpad_down || gamepad2.dpad_down) {
            foundationMotor.setPower(-1.0f);
        } else if(gamepad1.dpad_left || gamepad1.dpad_right || gamepad2.dpad_left || gamepad2.dpad_right) {
            foundationMotor.setPower(0.0f);
        }

        double collectporPower = gamepad2.left_trigger - gamepad2.right_trigger;
        leftCollector.setPower(collectporPower);
        rightCollector.setPower(collectporPower);
    }

    private void setWheelsRunMode(DcMotor.RunMode mode) {
        leftFrontMotor.setMode(mode);
        rightFrontMotor.setMode(mode);
        leftBackMotor.setMode(mode);
        rightBackMotor.setMode(mode);
    }

    public void enableEncoders(LinearOpMode opMode) throws InterruptedException {
        setWheelsRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setWheelsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyroSensor.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!opMode.isStopRequested() && gyroSensor.isCalibrating())  {
            Thread.sleep(50);
            opMode.idle();
        }
        gyroSensor.resetZAxisIntegrator();
        mecanumDrive = new MecanumDrive(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor, gyroSensor);
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

        if(right) {
            newLFTarget = leftFrontMotor.getCurrentPosition() + position;
            newLBTarget = leftBackMotor.getCurrentPosition() - position;
            newRFTarget = rightFrontMotor.getCurrentPosition() - position;
            newRBTarget = rightBackMotor.getCurrentPosition() + position;
        } else {
            newLFTarget = leftFrontMotor.getCurrentPosition() - position ;
            newLBTarget = leftBackMotor.getCurrentPosition() + position;
            newRFTarget = rightFrontMotor.getCurrentPosition() + position;
            newRBTarget = rightBackMotor.getCurrentPosition() - position;
        }

        leftFrontMotor.setTargetPosition(newLFTarget);
        leftBackMotor.setTargetPosition(newLBTarget);
        rightFrontMotor.setTargetPosition(newRFTarget);
        rightBackMotor.setTargetPosition(newRBTarget);
        setWheelsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        setWheelsSpeed(absSpeed, absSpeed, absSpeed, absSpeed);
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

            // Determine new target position, and pass to motor controller
            newLFTarget = leftFrontMotor.getCurrentPosition() + leftPos;
            newLBTarget = leftBackMotor.getCurrentPosition() + leftPos;
            newRFTarget = rightFrontMotor.getCurrentPosition() + rightPos;
            newRBTarget = rightBackMotor.getCurrentPosition() + rightPos;
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

    public void setLiftPosition(LinearOpMode opMode, double speed, int targetPosition, double timeoutS) {
        if (opMode.opModeIsActive()) {
            liftMotor.setTargetPosition(targetPosition);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            liftMotor.setPower(Math.abs(speed));
            while (opMode.opModeIsActive() && liftMotor.isBusy() && (runtime.seconds() < timeoutS)) {
                opMode.idle();
            }
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setFoundationPower(double foundationPower) {
        foundationMotor.setPower(foundationPower);
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
        System.out.println("***************  lift: " + liftMotor.getCurrentPosition());
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

        double lfDstance = (double)lfPos / COUNTS_PER_INCH;
        double lbDstance = (double)lbPos / COUNTS_PER_INCH;
        double rfDstance = (double)rfPos / COUNTS_PER_INCH;
        double rbDstance = (double)rbPos / COUNTS_PER_INCH;

        return (lfDstance + lbDstance + rfDstance + rbDstance) / 4;
    }

    public void idleFor(LinearOpMode opMode, double sec) {
        runtime.reset();
        try {
            while (runtime.seconds() < sec) {
                //opMode.idle();
                Thread.sleep(50);
            }
        } catch (InterruptedException ex){}
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

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( LinearOpMode opMode,
                            double speed,
                            double distance,
                            double angle) {

        int     newLF, newLB, newRF, newRB;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLF = leftFrontMotor.getCurrentPosition() + moveCounts;
            newLB = leftBackMotor.getCurrentPosition() + moveCounts;
            newRF = rightFrontMotor.getCurrentPosition() + moveCounts;
            newRB = rightBackMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftFrontMotor.setTargetPosition(newLF);
            leftBackMotor.setTargetPosition(newLB);
            rightFrontMotor.setTargetPosition(newRF);
            rightBackMotor.setTargetPosition(newRB);

            setWheelsRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            setWheelsSpeed(speed, speed, speed, speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftFrontMotor.setPower(leftSpeed);
                leftBackMotor.setPower(leftSpeed);
                rightFrontMotor.setPower(rightSpeed);
                rightBackMotor.setPower(rightSpeed);

                // Display drive status for the driver.
//                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
//                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
//                telemetry.addData("Actual",  "%7d:%7d",      robot.leftDrive.getCurrentPosition(),
//                        robot.rightDrive.getCurrentPosition());
//                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
//                telemetry.update();
            }

            // Stop all motion;
            stopRobot();

            // Turn off RUN_TO_POSITION
            setWheelsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn ( LinearOpMode opMode, double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            //telemetry.update();
            opMode.idle();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( LinearOpMode opMode, double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            //telemetry.update();
        }

        // Stop all motion;
        stopRobot();
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftFrontMotor.setPower(leftSpeed);
        leftBackMotor.setPower(leftSpeed);
        rightFrontMotor.setPower(rightSpeed);
        rightBackMotor.setPower(rightSpeed);

        // Display it for the driver.
//        telemetry.addData("Target", "%5.2f", angle);
//        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
//        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyroSensor.getIntegratedZValue();
        System.out.println("*************** getError: robotError:"+ robotError + ", robotAngle:"+gyroSensor.getIntegratedZValue());
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    public void startStoneIntake() {
        leftCollector.setPower(-0.7f);
        rightCollector.setPower(-0.7f);
    }

    public void stopStoneIntake() {
        leftCollector.setPower(0.0f);
        rightCollector.setPower(0.0f);
    }

    public boolean isStoneCollected() {
        System.out.println("*************** isStoneCollected: distance:"+ distanceSensor.getDistance(DistanceUnit.INCH));
        return distanceSensor.getDistance(DistanceUnit.INCH) < 2.0f;
    }
}
