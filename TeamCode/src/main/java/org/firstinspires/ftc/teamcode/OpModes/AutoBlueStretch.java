/*
 * Program Name:
 * Alliance:
 * Starting position
 * Functions of the program:
 *  - STEP1 =   gets the foundation into the build site
 *  - STEP2
 *
 *
 *
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

import java.util.List;

@Autonomous(name = "Auto - Blue Stretch", group = "Leviathan")
@Disabled

public class AutoBlueStretch extends LinearOpMode {
    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;
    private State state = State.RING_DETECT;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "ARLYRsf/////AAABmWpsWSsfQU1zkK0B5+iOOr0tULkAWVuhNuM3EbMfgb1+zbcOEG8fRRe3G+iLqL1/iAlTYqqoLetWeulG8hkCOOtkMyHwjS/Ir8/2vUVgC36M/wb9a7Ni2zuSrlEanb9jPVsNqq+71/uzTpS3TNvJI8WeICQNPAq3qMwmfqnCphVlC6h2ZSLsAR3wcdzknFmtpApdOp1jHJvITPeD/CMdAXjZDN0XJwJNQJ6qtaYSLGC23vJdQ2b1aeqnJauOvswapsG7BlmR7m891VN92rNEcOX7WmMT4L0JOM0yKKhPfF/aSROwIdNtSOpQW4qEKVjw3aMU1QDZ0jj5SnRV8RPO0hGiHtXy6QJcZsSj/Y6q5nyf";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public AutoBlueStretch(){

    }   // end of TestAuto constructor

    public void runOpMode(){
        double startTime = 0;
        double timeElapsed;
        double armPosition;
        int position = 1;
        ElapsedTime runTime = new ElapsedTime();

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        initVuforia();
        initTfod();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);

        /*
         * Initialize the drive class
         */
        DriveMecanum drive = new DriveMecanum(robot, opMode);

        /*
         * Calibrate / initialize the gyro sensor
         */

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2.5, 1.78);
        }

        armPosition = robot.motorWobbleArm.getCurrentPosition();
        robot.servoWobbleGrab.setPosition(0.8);
        telemetry.addData("Z Value = ", drive.getZAngle());
        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();


        while(!opModeIsActive()){

            if (tfod != null ) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        telemetry.addData("Size = ", updatedRecognitions.size());
                        telemetry.addData("Label = ", recognition.getLabel());
                        if (recognition.getLabel() == "Quad") position = 3;
                        else if (recognition.getLabel() == "Single") position = 2;
                        else position = 1;
                    }     //  for(Recognition recognition)
                    telemetry.addData("Position = ", position);
                    telemetry.update();
                }   else {
                    position = 1;
                }   // if(updatedRecognitions != null)

            }   // if(tfod != null)

        }

        waitForStart();
        startTime = runTime.time();

        while(opModeIsActive()) {

            switch (state) {
                case RING_DETECT:

                    // ****** Add code for determining which target zone to use *****
                    timeElapsed = runTime.time() - startTime;

                    if (tfod != null && timeElapsed < 1) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());

                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());

                                telemetry.addData("Size = ", updatedRecognitions.size());
                                telemetry.addData("Label = ", recognition.getLabel());
                                if (recognition.getLabel() == "Quad") position = 3;
                                else if (recognition.getLabel() == "Single") position = 2;
                                else position = 1;
                            }     //  for(Recognition recognition)
                            telemetry.addData("Position = ", position);
                            telemetry.update();
                        }   // if(updatedRecognitions != null)
                    }   else {
                        state = State.PATH_DECISION;
                    }  // if(tfod != null)

                    break;

                case PATH_DECISION:

                    telemetry.addData("Position = ", position);
                    telemetry.update();

                    // Decide the next state based on the number of rings on the floor
                    if (position == 3){
                        state = State.WOBBLE1C;
                    } else if (position == 2) {
                        state = State.WOBBLE1B;
                    } else {
                        state = State.WOBBLE1A;
                    }

                    break;

                case WOBBLE1A:
                    // Drive to target zone A
                    drive.robotCorrect(0.3, 0, 1.7);

                    // Place wobble goal
                    robot.motorWobbleArm.setPower(0.3);
                    sleep(650);
                    robot.motorWobbleArm.setPower(0);

                    robot.servoWobbleGrab.setPosition(0.2);
                    //robot.motorWobbleArm.setTargetPosition(0);

                    sleep(500);
                    // Return wobble arm t start position
                    robot.motorWobbleArm.setPower(-0.45);
                    sleep(500);
                    robot.motorWobbleArm.setPower(0);

                    // get ready for second wobble goal
                    drive.robotCorrect(0.3, 180, 0.8);
                    drive.robotCorrect(0.3, 225, 0.5);
                   // drive.robotCorrect(0.3, 180, 0.2);
                    state = State.RESET_START;

                    break;

                case WOBBLE1B:
                    // Drive to target zone B
                    drive.robotCorrect(0.3, -45, 0.5);

                    // Drive to target zone B
                    drive.robotCorrect(0.3, 0, 1.75);

                    //Strafe target zone B
                    drive.robotCorrect(0.5, 90, 0.85);

                    // Place wobble goal
                    robot.motorWobbleArm.setPower(0.3);
                    sleep(650);

                    robot.motorWobbleArm.setPower(0);
                    robot.servoWobbleGrab.setPosition(0.2);
                    //robot.motorWobbleArm.setTargetPosition(0);

                    sleep(300);

                    robot.motorWobbleArm.setPower(-0.45);
                    sleep(500);
                    robot.motorWobbleArm.setPower(0);

                    drive.PIDRotate(0, 0.3);
                    drive.robotCorrect(0.3, 180, 0.6);

//                    drive.PIDRotate(0, 0.3);

                    // head towards the reset wall
                    drive.robotCorrect(0.5, 225, 1.4);

                    drive.robotCorrect(0.5, 180, 0.3);

                    state = State.RESET_START;
                    break;

                case WOBBLE1C:
                    // Drive to target zone B
                    drive.robotCorrect(0.3, -45, 0.5);

                    // Drive to target zone A
                    drive.robotCorrect(0.3, 0, 2.2);
                    drive.robotCorrect(0.3, 90, 0.5);

                    // Drive to target zone B
                    drive.robotCorrect(0.5, 45, 0.3);

                    // Place wobble goal
                    robot.motorWobbleArm.setPower(0.3);
                    sleep(650);
                    robot.motorWobbleArm.setPower(0);

                    robot.servoWobbleGrab.setPosition(0.2);
                    //robot.motorWobbleArm.setTargetPosition(0);

                    sleep(300);

                    // Raising arm

                    robot.motorWobbleArm.setPower(-0.45);
                    sleep(400);
                    robot.motorWobbleArm.setPower(0);
                    // Park
                    drive.PIDRotate(0,0.3);
//                    drive.robotCorrect(0.5, 90, 0.5);
                    drive.robotCorrect(0.3, 180, 0.5);
                    drive.robotCorrect(0.3, 225, 0.8);
                    drive.robotCorrect(0.3, 180, 1.4);

                    state = State.RESET_START;

                    break;

                case RESET_START:

                    //reset robot position into the corner
//                    drive.PIDRotate(0, 0.3);
                    drive.robotCorrect(0.2, 225, 0.5);
                    drive.robotCorrect(0.2, -90, 0.5);
                    drive.robotCorrect(0.2, 180, 0.5);

                    // strafe to the wobble goal
                    drive.robotCorrect(0.3, 90, 1.25);
                    drive.PIDRotate(0, 0.3);
                    sleep(300);

                    // drive forward into the wobble goal
                    drive.robotCorrect(0.3, 0, 0.6);

                    // Decide the next state based on the number of rings on the floor
                    if (position == 3){
                        state = State.WOBBLE2C;
                    } else if (position == 2) {
                        state = State.WOBBLE2B;
                    } else {
                        state = State.WOBBLE2A;
                    }
                    break;

                case WOBBLE2A:

                    //Strafe to second wobble goal
                    drive.robotCorrect(0.4, -53, 1.5);

                    // Push wobble to target zone
//                    drive.robotCorrect(0.3, 0, 0.3);
                    drive.robotCorrect(0.3, 180, 0.5);

                    // drive to shooting location
                    drive.PIDRotate(0, 0.3);
                    drive.robotCorrect(0.3, 90, 0.5);
//                    drive.robotCorrect(0.3, 0,.3);

                    state = State.PREP_SHOOOTER;

                    break;

                case WOBBLE2B:
                    drive.robotCorrect(0.3, 0, 0.8);

                    //Strafe to second wobble goal
                    drive.robotCorrect(0.3, -45, .6);

                    // Push wobble to target zone
                    drive.robotCorrect(0.3, 0, 0.3);

                    // drive to the parking zone
                    drive.robotCorrect(0.3, 180, 0.6);

                    state = State.PREP_SHOOOTER;

                    break;

                case WOBBLE2C:
                    drive.robotCorrect(0.3, 0, 1.4);

                    //Strafe to Target Zone C
                    drive.robotCorrect(0.3, -45, 1.5);

                    // Push wobble to target zone
                    drive.robotCorrect(0.3, 0, 0.3);

                    // drive to the parking zone
                    drive.robotCorrect(0.3, 165, .8);

                    state = State.PREP_SHOOOTER;

                    break;

                case PREP_SHOOOTER:
                    drive.setDrivePower(-.1, .2,.2,-.1);
                    while (drive.getZAngle()<175) {

                    }
                    drive.motorsHalt();

//                    drive.PIDRotate(175, 0.5);
                    robot.motorShooter.setPower(0.80);
                    sleep(2000);

                    state = State.SHOOT;
                    break;

                case SHOOT:
                    robot.motorIntake.setPower(1);
                    sleep (5000);
                    robot.motorIntake.setPower(0);
                    robot.motorShooter.setPower(0);
                    state = State.PARK;
                    break;

                case PARK:
                    drive.robotCorrect(0.3, 180,0.7);
                    state = State.HALT;

                    break;

                case HALT:

                    // Stop all motors
                    drive.motorsHalt();

                    // End the program
                    requestOpModeStop();

                    break;
            }   // end of the switch state

        }   // end of while opModeIsActive()

    }   // end of runOpMode method

    /*
     * Enumuerate the states of the machine
     */
    enum State {
        RING_DETECT, PATH_DECISION, WOBBLE1A, WOBBLE1B, WOBBLE1C, RESET_START, WOBBLE2A, WOBBLE2B, WOBBLE2C, PARK, PREP_SHOOOTER, SHOOT, HALT;
    }   // end of enum State

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}   // end of AutoBlueStretch.java class
