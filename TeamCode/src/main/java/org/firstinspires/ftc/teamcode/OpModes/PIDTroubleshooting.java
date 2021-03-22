package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanum;

@TeleOp(name = "PID Troubleshooting", group = "Leviathan")
@Disabled

public class PIDTroubleshooting extends LinearOpMode {
    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;


    @Override
    public void runOpMode(){
        ElapsedTime runTime = new ElapsedTime();

        robot.init(hardwareMap);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        /*
         * Initialize the drive class
         */
        DriveMecanum drive = new DriveMecanum(robot, opMode);
        double rfStart = robot.motorRF.getCurrentPosition();
        double lfStart = robot.motorLF.getCurrentPosition();
        double rrStart = robot.motorRR.getCurrentPosition();
        double lrStart = robot.motorLR.getCurrentPosition();

        waitForStart();

//        drive.robotCorrect(0.5, 90, 2);
//        drive.PIDRotate(180, 0.3);
//        telemetry.addData("Head 45 degrees after turning 180", "");
//        sleep(2000);
        drive.robotCorrect(0.5, 45, 1);
//        telemetry.addData("Turn 180 degrees for a second time", "");
//        sleep(2000);
//        drive.PIDRotate(180, 0.3);
        drive.driveDistance(0.5, 0, 24);

        while (opModeIsActive()) {
            telemetry.addData("Gyro360 Method +180 Value = ", drive.gyro360(180));
            telemetry.addData("Gyro360 Method -180 Value = ", drive.gyro360(-180));
            telemetry.addData("MotorRF Encoder Tick Count = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("MotorRR Encoder Tick Count = ", robot.motorRR.getCurrentPosition());
            telemetry.addData("MotorLF Encoder Tick Count = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("MotorLR Encoder Tick Count = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("Distance Traveled = ", drive.calcDistance(90, rfStart, rrStart, lfStart, lrStart));
            telemetry.addData("Gyro value = ", drive.getZAngle());
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}
