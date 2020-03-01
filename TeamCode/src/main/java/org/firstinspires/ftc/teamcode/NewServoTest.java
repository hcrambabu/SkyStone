package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "NewServoTest")
public class NewServoTest extends LinearOpMode {

    private ModernRoboticsI2cGyro gyroSensor;
    private Servo servoMotor;
    private CRServo electroMagnet;
    private DcMotor dcMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        gyroSensor = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        servoMotor = hardwareMap.get(Servo.class, "servo");
        electroMagnet = hardwareMap.get(CRServo.class, "magnet");
        dcMotor = hardwareMap.get(DcMotor.class, "motor");
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyroSensor.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!this.isStopRequested() && gyroSensor.isCalibrating())  {
            try {
                Thread.sleep(50);
                this.idle();
            }catch (Exception e){}
        }
        gyroSensor.resetZAxisIntegrator();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        telemetry.addData(">", "Started...");
        telemetry.update();

        while(opModeIsActive()) {

            if(gamepad1.dpad_up) {
                servoMotor.setPosition(0);
            } else if(gamepad1.dpad_right || gamepad1.dpad_left) {
                servoMotor.setPosition(0.5);
            } else if(gamepad1.dpad_down) {
                servoMotor.setPosition(1.0);
            }

            if(gamepad1.x) {
                electroMagnet.setPower(0.0f);
            } else if(gamepad1.a) {
                electroMagnet.setPower(1.0f);
            } else if(gamepad1.b) {
                electroMagnet.setPower(-1.0f);
            }

            dcMotor.setPower(gamepad1.left_stick_y);

            if(gamepad1.right_bumper) {
                servoMotor.setDirection(Servo.Direction.FORWARD);
            } else if(gamepad1.left_bumper) {
                servoMotor.setDirection(Servo.Direction.REVERSE);
            }
            String direction = servoMotor.getDirection() == Servo.Direction.FORWARD? "FORWARD":"REVERSE";
            telemetry.addData("gyro", gyroSensor.getIntegratedZValue());
            telemetry.addData("servo", servoMotor.getPosition() + ", Direction:" + direction);
            telemetry.addData("magnet", electroMagnet.getPower());
            telemetry.addData("motor", dcMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
