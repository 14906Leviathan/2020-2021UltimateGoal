package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@TeleOp(name = "Teleop Mode", group = "Leviathan")

public class LeviathanTeleop extends LinearOpMode {
    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;


    @Override
    public void runOpMode(){
        double v1, v2, v3, v4, robotAngle, powerLevel=1;
        double modePower = 1;
        double theta;
        double r;
        double rightX, rightY;
        double peakPower = 0.30;
        boolean fieldCentric = false;
        boolean intake = false;
        boolean intakeForward = true;
        boolean intakeHalf = false;
        boolean shooter = false;
        double armPosition = 0.2;
        double currentRPM;
        ElapsedTime runTime = new ElapsedTime();
        double elapsedTime;
        double currentTime=0;
        double rotationAngle = 0;
        double linearServoPosition = robot.SERVO_LINEAR_INTAKE;

        //      robot.servoIntake.setPosition(0);

        robot.init(hardwareMap);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        /*
         * Initialize the drive class
         */
        DriveMecanum drive = new DriveMecanum(robot, opMode);

        robot.servoRingStopper.setPosition(0.6);


        waitForStart();
        currentTime = runTime.time();

        while (opModeIsActive()) {

            /*
             * Mecanum Drive Control section
             */
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAngularOrientation().firstAngle;
            } else {
                theta = 0 + rotationAngle;      // do not adjust for the angular position of the robot
            }
            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            rightY = -gamepad1.right_stick_y;
            r = -Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta)) + rightX + rightY) * powerLevel;
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta)) - rightX + rightY) * powerLevel;
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta)) + rightX + rightY) * powerLevel;
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta)) - rightX + rightY) * powerLevel;

            robot.motorLF.setPower(Range.clip((v1 * modePower), -1, 1));
            robot.motorRF.setPower(Range.clip((v2 * modePower), -1, 1));
            robot.motorLR.setPower(Range.clip((v3 * modePower), -1, 1));
            robot.motorRR.setPower(Range.clip((v4 * modePower), -1, 1));

            elapsedTime = runTime.time() - currentTime;

            /*
             * Field Centric drive controls
             * Behaviors
             *  - Pressing gamepad1.x && gamepad1.y at the same time will reset the IMU position
             *  - Pressing gamepad1.x by itself will enable field centric drive
             *  - Pressing gamepad1.y by itself will disable field centric drive
             */

            if (gamepad1.left_bumper & (elapsedTime > 0.5)){
                currentTime = runTime.time();
                if (shooter){
                    shooter = false;
                } else shooter = true;
            }

            if (gamepad1.x && gamepad1.y){
                // reset the IMU => need to add the code
            } else if (gamepad1.x){    // open the grab servo
                fieldCentric = true;
            } else if (gamepad1.y){                // close the grab servo
                fieldCentric = false;
            }

            if (gamepad1.dpad_down) {
                robot.motorWobbleArm.setPower(0.2);
            } else if(gamepad1.dpad_up) {
                robot.motorWobbleArm.setPower(-0.2);
            } else {
                robot.motorWobbleArm.setPower(0);
            }

            if (gamepad1.dpad_left) {
                robot.servoWobbleGrab.setPosition(robot.SERVO_WOBBLE_GRAB_OPEN);
            } else if (gamepad1.dpad_right) {
                robot.servoWobbleGrab.setPosition(robot.SERVO_WOBBLE_GRAB_CLOSE);
            }

            if(gamepad2.dpad_up){
                linearServoPosition = linearServoPosition + 0.01;
                sleep(200);
            } else if(gamepad2.dpad_down){
                linearServoPosition = linearServoPosition - 0.01;
                sleep(200);
            }

            if(gamepad2.x) {
                linearServoPosition = robot.SERVO_LINEAR_TELEOP_SHOOT;
            }
            if (gamepad2.y){
                linearServoPosition = robot.SERVO_LINEAR_INTAKE;
            }

            robot.servoLinear.setPosition(linearServoPosition);

            if (gamepad1.left_trigger >0.2 & elapsedTime > 0.5) {
                currentTime = runTime.time();
                if (intake) {
                    intake = false;
                } else {
                    intake = true;
                }
                intakeForward = true;
            } else if (gamepad1.right_trigger > 0.2) {
                currentTime = runTime.time();
                if (intake) {
                    intake = false;
                } else {
                    intake = true;
                }
                intakeForward = false;
            }

            if (intake){
                if (intakeForward) {
                    if(intakeHalf){
                        robot.motorIntake.setPower(0.5);
                    } else {
                        robot.motorIntake.setPower(1);
                    }
                } else {
                    robot.motorIntake.setPower(-1);
                }
            } else {
                robot.motorIntake.setPower(0);
            }

            if (gamepad1.right_bumper) {
                robot.servoRingStopper.setPosition(robot.SERVO_SHOOTER_UP);
                sleep(100);
                intake = true;
                intakeHalf = true;
                intakeForward = true;

            } else {
                robot.servoRingStopper.setPosition(robot.SERVO_SHOOTER_DOWN);
                intakeHalf = false;
               // intake = false;
            }

            if (shooter){
                drive.shooterControl(robot.TARGET_SHOOTER_RPM);
            } else {
                drive.shooterControl(0);
            }

            telemetry.addData("V1 = ", v1);
            telemetry.addData("V2 = ", v2);
            telemetry.addData("V3 = ", v3);
            telemetry.addData("V4 = ", v4);

            telemetry.addData("linearservo = ", robot.servoLinear.getPosition());
            telemetry.addData("motorLF = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("motorLR = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("motorRF = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("motorRR = ", robot.motorRR.getCurrentPosition());
            telemetry.addData("dpad_up = ", gamepad1.dpad_up);
            telemetry.addData("dpad_down = ", gamepad1.dpad_down);
            telemetry.addData("dpad_left = ", gamepad1.dpad_left);
            telemetry.addData("dpad_right = ", gamepad1.dpad_right);
            telemetry.addData("Left Stick X = ", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y = ", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X = ", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y = ", gamepad1.right_stick_y);
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}
