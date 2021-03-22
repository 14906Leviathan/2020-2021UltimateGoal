package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@TeleOp(name = "Teleop Mode - Working", group = "Leviathan")
@Disabled

public class TeleOpTest extends LinearOpMode {
    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;


    @Override
    public void runOpMode(){
        double v1, v2, v3, v4, robotAngle, powerLevel=0.6;
        double modePower = 1;
        double theta;
        double r;
        double rightX, rightY;
        double peakPower = 0.30;
        boolean fieldCentric = true;
        double armPosition = 0.2;
        double currentRPM;
        ElapsedTime runTime = new ElapsedTime();
        boolean shooter = false;
        double shooterPower = 0.80;
        double currentTick=0;
        double currentTime=0;

        //      robot.servoIntake.setPosition(0);

        robot.init(hardwareMap);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        /*
         * Initialize the drive class
         */
        DriveMecanum drive = new DriveMecanum(robot, opMode);

        waitForStart();

        while (opModeIsActive()) {

            /*
             * Mecanum Drive Control section
             */

            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAngularOrientation().firstAngle + 90;
            } else {
                theta = 0;      // do not adjust for the angular position of the robot
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

            /*
             * Field Centric drive controls
             * Behaviors
             *  - Pressing gamepad1.x && gamepad1.y at the same time will reset the IMU position
             *  - Pressing gamepad1.x by itself will enable field centric drive
             *  - Pressing gamepad1.y by itself will disable field centric drive
             */

            if (gamepad1.left_bumper){
                robot.motorShooter.setPower(0.8);
            } else robot.motorShooter.setPower(0);

            if (gamepad1.x && gamepad1.y){
                // reset the IMU => need to add the code
            } else if (gamepad1.x){    // open the grab servo
                fieldCentric = true;
            } else if (gamepad1.y){                // close the grab servo
                fieldCentric = false;
            }

            if (gamepad1.dpad_down) {
                robot.motorWobbleArm.setPower(0.3);
            } else if(gamepad1.dpad_up) {
                robot.motorWobbleArm.setPower(-0.3);
            } else {
                robot.motorWobbleArm.setPower(0);
            }

            if (gamepad1.dpad_left) {
                robot.servoWobbleGrab.setPosition(0.05);
            } else if (gamepad1.dpad_right) {
                robot.servoWobbleGrab.setPosition(0.9);
            }

            if (gamepad1.left_trigger >0.2) {
                robot.motorIntake.setPower(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.2) {
                robot.motorIntake.setPower(-gamepad1.right_trigger);
            } else robot.motorIntake.setPower(0);


            telemetry.addData("V1 = ", v1);
            telemetry.addData("V2 = ", v2);
            telemetry.addData("V3 = ", v3);
            telemetry.addData("V4 = ", v4);


            telemetry.addData("dpad_up = ", gamepad1.dpad_up);
            telemetry.addData("dpad_down = ", gamepad1.dpad_down);
            telemetry.addData("dpad_left = ", gamepad1.dpad_left);
            telemetry.addData("dpad_right = ", gamepad1.dpad_right);
            telemetry.addData("Left Stick X = ", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y = ", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X = ", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y = ", gamepad1.right_stick_y);
            //telemetry.addData("IMU Value: ", theta);
            telemetry.update();


        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}
