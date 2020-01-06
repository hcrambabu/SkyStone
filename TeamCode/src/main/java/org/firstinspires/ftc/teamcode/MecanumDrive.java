package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class MecanumDrive {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor backLeft;
    private ModernRoboticsI2cGyro gyroSensor;

    public MecanumDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight,
                        ModernRoboticsI2cGyro gyroSensor) {
        this.frontLeft = frontLeft;
        this.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.frontRight = frontRight;
        this.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.backLeft = backLeft;
        this.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.backRight = backRight;
        this.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.gyroSensor = gyroSensor;
        this.gyroSensor.resetZAxisIntegrator();
    }

    void setSpeeds(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
        double largest = 1.0;
        largest = Math.max(largest, Math.abs(flSpeed));
        largest = Math.max(largest, Math.abs(frSpeed));
        largest = Math.max(largest, Math.abs(blSpeed));
        largest = Math.max(largest, Math.abs(brSpeed));

        frontLeft.setPower(flSpeed / largest);
        frontRight.setPower(frSpeed / largest);
        backLeft.setPower(blSpeed / largest);
        backRight.setPower(brSpeed / largest);
    }

    void driveMecanum(double forward, double strafe, double rotate) {
        double frontLeftSpeed = forward + strafe + rotate;
        double frontRightSpeed = forward - strafe - rotate;
        double backLeftSpeed = forward - strafe + rotate;
        double backRightSpeed = forward + strafe - rotate;

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    void driveMecanumGyro(double forward, double strafe, double rotate) {
        double frontLeftSpeed = forward + strafe + rotate;
        double frontRightSpeed = forward - strafe - rotate;
        double backLeftSpeed = forward - strafe + rotate;
        double backRightSpeed = forward + strafe - rotate;

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    void reportEncoders(Telemetry telemetry) {
        telemetry.addData("Encoders", "%d %d %d %d",
                frontLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(),
                backRight.getCurrentPosition());


    }
}