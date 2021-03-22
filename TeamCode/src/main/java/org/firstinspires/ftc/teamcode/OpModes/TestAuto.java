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

import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@Autonomous(name = "Test Autonomous", group = "Programming Class")
@Disabled

public class TestAuto extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;
    private State state = State.STEP1;

    public TestAuto(){

    }   // end of TestAuto constructor

    public void runOpMode(){
        double startTime;
        double timeElapsed;

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

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

        telemetry.addData("Z Value = ", drive.getZAngle());
        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            switch (state) {
                case STEP1:
                    // strafe over to foundation area
                    drive.PIDRotate(90, 0.5);
                    telemetry.addData("completed the task", "taking a 5 second break");
                    telemetry.update();
                    drive.getZAngle();
                    drive.robotCorrect(.8, 90, 3);

                    // raise our arm

                    // flash green lights

                    // change state to STEP2
                    state = State.STEP2;

                    break;

                case STEP2:
                    // driving close to the foundation
                    drive.PIDRotate(-90, 0.5);

                    // lower arm

                    // flash red lights
                    state = State.STEP3;

                    break;

                case STEP3:
                    // driving close to the foundation
                    drive.PIDRotate(0, 0.5);

                    // lower arm

                    // flash red lights
                    state = State.STEP4;

                    break;

                case STEP4:
                    // flash some lights

                    // Stop all motors
                    drive.motorsHalt();
                    sleep(5000);

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
        STEP1, STEP2, STEP3, STEP4;
    }   // end of enum State

}   // end of TestAuto.java class
