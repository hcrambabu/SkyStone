package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Navigation {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private static final String VUFORIA_KEY =
            "AXx8B3P/////AAABmUvCh01qr0NYuiZsl4XR5wxHpJiI+AQBbtiTScffb3UpHwbjqT0gTnTtblgQyH6abrP5hIOA/Y4wgs8bU+1LwD/bor01NOM30m6KKqBS0hrGh0Z8IZu+1sNQyNzgm5dZNUKFI7UzEGUTlEL0L8r1v2++74NQkE8ZZFs6WyUjEowkDBpYQQE0ANXA5qDl0g2Rd7S3Y4rk9HgRJrJaZ0ojGT0uNzHdjkO7gpPYFsEDfAPVz7Pguzw7psyDlPvRmnKajnomWiCVEortJir77e1fgPSCnLobhrXL8b9PN3vaLu8ow0GxMbmJJN1ni0m+vzguiRaNy4JhDDgevKJuN4bv5CLqIt1EDMDG9ROrxcq3OJMR";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private VuforiaTrackables targetsSkyStone;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    private HashMap<String, VuforiaTrackable> trackableHashMap = new HashMap<>();
    private boolean isShutdowned = false;

    public void initialize(LinearOpMode opMode) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName(Target.STONE_TARGET);
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName(Target.BLUE_REAR_BRIDGE);
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName(Target.RED_REAR_BRIDGE);
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName(Target.RED_FRONT_BRIDGE);
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName(Target.BLUE_FRONT_BRIDGE);
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName(Target.RED_PERIMETER_1);
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName(Target.RED_PERIMETER_2);
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName(Target.FRONT_PERIMETER_1);
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName(Target.FRON_PERIMETER_2);
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName(Target.BLUE_PERIMETER_1);
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName(Target.BLUE_PERIMETER_2);
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName(Target.REAR_PERIMETER_1);
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName(Target.REAR_PERIMETER_2);

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsSkyStone);
        for (VuforiaTrackable trackable : allTrackables) {
            trackableHashMap.put(trackable.getName(), trackable);
        }

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 5.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 8.0f * mmPerInch;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
        isShutdowned = false;
    }

    public Position getRobotPosition(LinearOpMode opMode) {
        Position currPosition = null;
        String name = "";

        if(opMode.opModeIsActive() && ! isShutdowned) {
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    name = trackable.getName();
                    opMode.telemetry.addData("Visible Target", name);
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                currPosition = new Position(name, translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                opMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        currPosition.X, currPosition.Y, currPosition.Z);
            } else {
                opMode.telemetry.addData("Visible Target", "none");
            }
            opMode.telemetry.update();
        }

        return currPosition;
    }

    public Position getCameraPosition(LinearOpMode opMode) {
        Position currPosition = null;
        String name = "";
        OpenGLMatrix vuforiaCameraFromTarget = null;

        if(opMode.opModeIsActive() && ! isShutdowned) {
            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    name = trackable.getName();
                    opMode.telemetry.addData("Visible Target", name);
                    System.out.println("*************** getCameraPositionFromTarget: Visible Target: " + name);
                    targetVisible = true;

                    vuforiaCameraFromTarget = ((VuforiaTrackableDefaultListener) trackable.getListener())
                            .getVuforiaCameraFromTarget();
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible && vuforiaCameraFromTarget != null) {
                VectorF translation = vuforiaCameraFromTarget.getTranslation();
                currPosition = new Position(name, translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                opMode.telemetry.addData("CamFromTarget (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        currPosition.X, currPosition.Y, currPosition.Z);
            } else {
                opMode.telemetry.addData("Visible Target", "none");
            }
            opMode.telemetry.update();
        }

        return currPosition;
    }

    public String getViewingTarget(LinearOpMode opMode) {
        String target = null;
        if(opMode.opModeIsActive() && ! isShutdowned) {
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    target = trackable.getName();
                    targetVisible = true;
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                opMode.telemetry.addData("Visible Target", target);
            } else {
                opMode.telemetry.addData("Visible Target", "none");
            }
            opMode.telemetry.update();
        }
        return target;
    }

    public Position getCameraPositionFromTraget(LinearOpMode opMode, String target) {
        Position currPosition = null;
        VuforiaTrackable trackable = trackableHashMap.get(target);

        if(trackable != null){
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                OpenGLMatrix vuforiaCameraFromTarget = ((VuforiaTrackableDefaultListener) trackable.getListener()).getVuforiaCameraFromTarget();
                if (vuforiaCameraFromTarget != null) {
                    VectorF translation = vuforiaCameraFromTarget.getTranslation();
                    currPosition = new Position(trackable.getName(), translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                    opMode.telemetry.addData("Visible Target", trackable.getName());
                    opMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            currPosition.X, currPosition.Y, currPosition.Z);
                }
            } else {
                System.out.println("*************** "+ target + " NOT VISIBLE");
                opMode.telemetry.addData("Visible Target", "none");
            }
        } else {
            System.out.println("*************** "+ target + " NULL");
            opMode.telemetry.addData("Visible Target", "NULL");
        }
        opMode.telemetry.update();
        return currPosition;
    }

    public Position getRobotPositionFromTraget(LinearOpMode opMode, String target) {
        Position currPosition = null;
        VuforiaTrackable trackable = trackableHashMap.get(target);
        if(trackable != null) {
            if(((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    VectorF translation = robotLocationTransform.getTranslation();
                    currPosition = new Position(trackable.getName(), translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                    opMode.telemetry.addData("Visible Target", trackable.getName());
                    opMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            currPosition.X, currPosition.Y, currPosition.Z);
                }
            } else {
                System.out.println("*************** "+ target + " NOT VISIBLE");
                opMode.telemetry.addData("Visible Target", "none");
            }
        } else {
            System.out.println("*************** "+ target + " NULL");
            opMode.telemetry.addData("Visible Target", "NULL");
        }
        opMode.telemetry.update();
        return currPosition;
    }

    public void shutdown() {
        // Disable Tracking when we are done;
        if(targetsSkyStone != null)
            targetsSkyStone.deactivate();
        isShutdowned = true;
    }

    interface Target {
        public static String STONE_TARGET = "Stone Target";
        public static String BLUE_REAR_BRIDGE = "Blue Rear Bridge";
        public static String RED_REAR_BRIDGE = "Red Rear Bridge";
        public static String RED_FRONT_BRIDGE = "Red Front Bridge";
        public static String BLUE_FRONT_BRIDGE = "Blue Front Bridge";
        public static String RED_PERIMETER_1 = "Red Perimeter 1";
        public static String RED_PERIMETER_2 = "Red Perimeter 2";
        public static String FRONT_PERIMETER_1 = "Front Perimeter 1";
        public static String FRON_PERIMETER_2 = "Front Perimeter 2";
        public static String BLUE_PERIMETER_1 = "Blue Perimeter 1";
        public static String BLUE_PERIMETER_2 = "Blue Perimeter 2";
        public static String REAR_PERIMETER_1 = "Rear Perimeter 1";
        public static String REAR_PERIMETER_2 = "Rear Perimeter 2";
    }

    class Position {
        double X;
        double Y;
        double Z;
        String name;

        public Position(String name, double x, double y, double z) {
            this.name = name;
            X = x;
            Y = y;
            Z = z;
        }
    }
}
