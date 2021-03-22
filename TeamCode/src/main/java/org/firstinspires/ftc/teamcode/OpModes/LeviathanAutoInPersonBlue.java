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

@Autonomous(name = "In Person Auto - Blue", group = "Leviathan")

public class LeviathanAutoInPersonBlue extends LinearOpMode {

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

    public LeviathanAutoInPersonBlue(){

    }   // end of TestAuto constructor

    public void runOpMode(){
        double startTime = 0;
        double timeElapsed;
        double armPosition;
        String startPosition = "";
        boolean initReady = false;
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

        // close the grabber on the wobble goal
        robot.servoWobbleGrab.setPosition(0.8);
        robot.servoRingStopper.setPosition(robot.SERVO_SHOOTER_DOWN);
        robot.servoLinear.setPosition(robot.SERVO_LINEAR_INITIALIZED);

        // Let the user know the robot is initialized
        telemetry.addData("Z Value = ", drive.getZAngle());
        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.addData("press X for Wall Position", "");
        telemetry.addData("press Y for Field Side Position", "");
        telemetry.update();

        while(!initReady) {
            if(gamepad1.x){
                startPosition = "WALL";
                initReady = true;
            }
            if(gamepad1.y){
                startPosition = "FIELD";
                initReady = true;
            }
        }   // end of while(!initReady)

        telemetry.addData("starting position = ", startPosition);

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
                        telemetry.addData("READY TO RUN! ", "GOOD LUCK LEVIATHAN!");
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        telemetry.addData("Size = ", updatedRecognitions.size());
                        telemetry.addData("Label = ", recognition.getLabel());
                        if (recognition.getLabel().equals("Quad")) position = 3;
                        else if (recognition.getLabel().equals("Single")) position = 2;
                        else position = 1;
                    }     //  for(Recognition recognition)
                    telemetry.addData("READY TO RUN! ", "GOOD LUCK LEVIATHAN!");
                    telemetry.addData("Position = ", position);
                    telemetry.update();
                }   else {
                    position = 1;
                }   // if(updatedRecognitions != null)

            }   // if(tfod != null)
        } // end of while(!opModeIsActive)

        waitForStart();
        startTime = runTime.time();

        while(opModeIsActive()) {

            switch (state) {

                case RING_DETECT:
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
                                if (recognition.getLabel().equals("Quad")) position = 3;
                                else if (recognition.getLabel().equals("Single")) position = 2;
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
                    if(opModeIsActive()) {
                        if(startPosition.equals("WALL")){
                            drive.shooterControl(robot.AUTO_SHOOTER_RPM);
                            robot.servoLinear.setPosition(robot.SERVO_LINEAR_AUTO_SHOOT);
                            if (position == 3) {
                                state = State.WOBBLE1C;
                            } else if (position == 2) {
                                state = State.WOBBLE1B;
                            } else {
                                state = State.WOBBLE1A;
                            }   // end if (position == 3)
                        } else {
                            // drive forward to position to move towards target zone
                            drive.driveDistance(0.5, 0, 40);

                            // prep for shooting
                            drive.shooterControl(robot.AUTO_SHOOTER_PS_RPM);
                            robot.servoLinear.setPosition(robot.SERVO_LINEAR_PS_SHOOT);

                            // decide which target zone to place the wobble goal
                            if (position == 3) {
                                state = State.WOBBLE2C;
                            } else if (position == 2) {
                                state = State.WOBBLE2B;
                            } else {
                                state = State.WOBBLE2A;
                            }   // end if (position == 3)
                        }
                    }   // end if(opModeIsActive)

                    break;

                case WOBBLE1A:

                    drive.shooterControl(robot.AUTO_SHOOTER_RPM);

                    robot.servoLinear.setPosition(robot.SERVO_LINEAR_HG_SHOOT);
                    // Strafe diagonally towards outside wall - avoids rings on the field
                    drive.robotCorrect(0.5, -45, 0.65);

                    // Drive to target zone A
                    drive.driveDistance(0.5, 0, 55);

                    // Drive back to shoot
                    drive.driveDistance(0.5, 180, 10);

                    if (opModeIsActive()) {
                        state = State.SHOOT_RINGS;
                    } else {
                        state = State.HALT;
                    }   // end if (opModeIsActive)

                    break;

                case WOBBLE1B:

                    drive.shooterControl(robot.AUTO_SHOOTER_RPM);

                    // Drive to target zone B
                    drive.driveDistance(0.8, 0, 60);

                    // Strafe to place the wobble goal
                    drive.robotCorrect(0.5, 45, 1.5);

                    // Return to shooting position
                    // Drive back to avoid hitting the wobble goal
                    drive.driveDistance(0.8, 180, 30);

                    // Strafe towards the wall - avoiding rings on the field
                    drive.driveDistance(.6, -90, 28);

                    if (opModeIsActive()) {
                        state = State.SHOOT_RINGS;
                    } else {
                        state = State.HALT;
                    }   // end if (opModeIsActive)

                    break;

                case WOBBLE1C:

                    robot.servoLinear.setPosition(robot.SERVO_LINEAR_HG_SHOOT);
                    drive.shooterControl(robot.AUTO_SHOOTER_RPM);

                    // Drive to target zone C
                    drive.driveDistance(0.7, 0, 100);

                    // Strafe diagonally towards outside wall - avoids rings on the field
                    drive.robotCorrect(0.5, -45, 0.65);

                    // Drive back to shoot
                    drive.driveDistance(0.7, 180, 50);

                    if (opModeIsActive()) {
                        state = State.SHOOT_RINGS;
                    } else {
                        state = State.HALT;
                    }   // end if (opModeIsActive)

                    break;

                case SHOOT_RINGS:
                    drive.shooterControl(robot.AUTO_SHOOTER_RPM);
                    drive.PIDRotate(15, 0.5);
                    drive.shootRings();
                    drive.shooterControl(0);
                    drive.PIDRotate(0, 0.5);
                    if (opModeIsActive()) {
                        state = State.PARK;
                    } else {
                        state = State.HALT;
                    }   // end if (opModeIsActive)
                    break;

                case SHOOT:

                    robot.motorIntake.setPower(1);
                    drive.PIDRotate(175, 0.3);

                    // fire the rings
                    robot.servoRingStopper.setPosition(robot.SERVO_SHOOTER_UP);
                    sleep(200);
                    robot.motorIntake.setPower(1);
                    sleep(1500);
                    robot.servoRingStopper.setPosition(robot.SERVO_SHOOTER_DOWN);
                    // Decide the next state based on the number of rings on the floor
                    if(opModeIsActive()){
                            state = State.PARK;
                    } else state = State.HALT;  // end if(opModeIsActive)

                    break;

                case FIELD_HG:
                    robot.servoLinear.setPosition(robot.SERVO_LINEAR_HG_SHOOT);
                    drive.shooterControl(robot.AUTO_SHOOTER_RPM);
                    sleep(1000);
                    drive.PIDRotate(-5, 0.5);
                    drive.shootRings();
                    state = State.PARK;
                    break;

                case WOBBLE2A:
                    //Arc turn towards the target zone
                    drive.setDrivePower(0.5,0.13, 0.13, 0.5);
                    sleep(2500);
                    drive.motorsHalt();

                    // Push wobble to target zone
                    drive.driveSimpleDistance(0.5, 0, 10);

                    //Arc turn towards the target zone
                    drive.setDrivePower(-0.5,-0.1, -0.1, -0.5);
                    sleep(2500);
                    drive.motorsHalt();

                    state = State.FIELD_HG;

                    break;

                case WOBBLE2B:
                    //Arc turn into the second target zone
                    drive.setDrivePower(0.5,0.38, 0.38, 0.5);
                    sleep(2500);
                    drive.motorsHalt();

                    // drive to park location
                    drive.driveSimpleDistance(0.5, 180, 30);

                    state = State.FIELD_HG;

                    break;

                case WOBBLE2C:
                    // Arc turn to the last target zone
                    drive.setDrivePower(0.5,0.35, 0.35, 0.5);
                    sleep(3000);
                    drive.motorsHalt();

                    // push the wobble goal into the target zone
                    drive.driveSimpleDistance(0.8, 0, 15);

                    // back away from the wobble goals
                    drive.driveSimpleDistance(0.5, 180, 15);

                    // Arc turn to the last target zone
                    drive.setDrivePower(-0.5,-0.35, -0.35, -0.5);
                    sleep(2000);
                    drive.motorsHalt();

                    // Reorient robot angle to face the line.
                    drive.PIDRotate(0, 1);

                    state = State.FIELD_HG;

                    break;

                case PARK:
                    // Add function to wait till there is 5 seconds left
                    // Reset Linear servo
                    robot.servoLinear.setPosition(robot.SERVO_LINEAR_INTAKE);
                    // Strafing
                    drive.driveDistance(0.5, 90, 30);
                    // Drive forward to the line
                    drive.driveDistance(0.5, 0, 20);
                    state = State.HALT;

                    break;

                case HALT:

                    // stop shooter motor
                    drive.shooterControl(0);

                    // stop intake motor
                    robot.motorIntake.setPower(0);

                    if (tfod != null) {
                        tfod.shutdown();
                    }

                    // Stop all motors
                    drive.motorsHalt();

                    // End the program
                    requestOpModeStop();

                    break;
            }   // end of the switch state

        }   // end of while opModeIsActive()

        // shut down all of the robot motors and operations
        drive.shooterControl(0);        // shut down the shooter motor

        robot.motorIntake.setPower(0);          // shut off the intake motor
        drive.motorsHalt();                     // shut off the drive motors

        if (tfod != null) {
            tfod.shutdown();                    // disable Tensorflow Object Detection
        }

        requestOpModeStop();                    // request the entire opMode to Stop

    }   // end of runOpMode method

    /*
     * Enumerate the states of the machine
     */
    enum State {
        TEST, FIELD_HG, RING_DETECT, PATH_DECISION, WOBBLE1A, WOBBLE1B, WOBBLE1C, RESET_START, WOBBLE2A, WOBBLE2B, WOBBLE2C, PARK, PREP_SHOOTER, SHOOT, HALT, SHOOT_RINGS;
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

}   // end of TestAuto.java class