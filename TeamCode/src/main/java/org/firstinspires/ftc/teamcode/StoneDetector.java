package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class StoneDetector {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Stone";
    public static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "AXx8B3P/////AAABmUvCh01qr0NYuiZsl4XR5wxHpJiI+AQBbtiTScffb3UpHwbjqT0gTnTtblgQyH6abrP5hIOA/Y4wgs8bU+1LwD/bor01NOM30m6KKqBS0hrGh0Z8IZu+1sNQyNzgm5dZNUKFI7UzEGUTlEL0L8r1v2++74NQkE8ZZFs6WyUjEowkDBpYQQE0ANXA5qDl0g2Rd7S3Y4rk9HgRJrJaZ0ojGT0uNzHdjkO7gpPYFsEDfAPVz7Pguzw7psyDlPvRmnKajnomWiCVEortJir77e1fgPSCnLobhrXL8b9PN3vaLu8ow0GxMbmJJN1ni0m+vzguiRaNy4JhDDgevKJuN4bv5CLqIt1EDMDG9ROrxcq3OJMR";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private boolean isShutdowned = false;
    private LinearOpMode opMode;

    public void initialize(LinearOpMode opMode) {
        this.opMode = opMode;
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            opMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
        isShutdowned = false;
        /** Wait for the game to begin */
        opMode.telemetry.addData(">", "StoneDetector Initialized");
        opMode.telemetry.update();
    }

    public List<Recognition> getVisibleStones() {

        if(!opMode.isStopRequested() && ! isShutdowned) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        opMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        opMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        opMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                    opMode.telemetry.update();

                    Collections.sort(updatedRecognitions, new Comparator<Recognition>() {
                        @Override
                        public int compare(Recognition o1, Recognition o2) {
                            return Float.compare(o1.getLeft(), o2.getLeft());
                        }
                    });
                    System.out.println("******************** returing (1) "+ updatedRecognitions);
                    return updatedRecognitions;
                }
            }
        }
        System.out.println("******************** returing (2) NULL");
        return null;
    }

    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }

        isShutdowned = true;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.5;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
