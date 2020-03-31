package org.firstinspires.ftc.teamcode.basedrive.retired;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.basedrive.hwmap.SoftwareBot;
import org.firstinspires.ftc.teamcode.basedrive.hwmap.ElectroBot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Electrobot: Auto Drive By Encoder", group = "Electrobot")
@Disabled
public class AutoOp_V1 extends LinearOpMode {

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    /* Declare OpMode members. */
    private ElectroBot robot = new ElectroBot();   // Initializing our robot
    private ElapsedTime runtime = new ElapsedTime();
    private PIDFCoefficients pidDrive = new PIDFCoefficients(3, 0.4, 0, 0);
    private PIDFCoefficients pidTurn = new PIDFCoefficients(.03, .003, 0, 0);

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        /*FL  = hardwareMap.get(DcMotor.class, "FL");
        BL  = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");*/
        Log.i(ElectroBot.LOGTAG, "Initializing Robot: ");
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        Log.i(ElectroBot.LOGTAG, "Initialized robot");
        telemetry.addData("Status", "Initialized Robot");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        Log.i(ElectroBot.LOGTAG, "Wait for start");
        waitForStart();
        Log.i(ElectroBot.LOGTAG, "Go forward 60in");
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.logDriveEncoderPosition();
        robot.setDriveMotorPower(0.6, 0.6, 0.6, 0.6);
        robot.setTravelDistance(60);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);
        //this.sleep(1000);
        Log.i(ElectroBot.LOGTAG, "Travel Left 48 in");
        robot.setDriveMotorPower(0.6, 0.6, 0.6, 0.6);
        robot.travelLeft(48);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        checkDriveMotors();
        Log.i(ElectroBot.LOGTAG, "travel right 48 in");
        robot.setDriveMotorPower(0, 0, 0, 0);
        robot.setDriveMotorPower(0.6, 0.6, 0.6, 0.6);
        robot.travelRight(-48);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        checkDriveMotors();
        Log.i(ElectroBot.LOGTAG, "Go back 60");
        robot.setDriveMotorPower(0, 0, 0, 0);
        robot.setDriveMotorPower(0.6, 0.6, 0.6, 0.6);
        robot.setTravelDistance(60);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        robot.logDriveEncoderPosition();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    protected void checkDriveMotors() {
        while ((opModeIsActive()) && robot.isRobotMoving()) {
            robot.logDriveEncoderPosition();
            int[] motorTicks = robot.getDriveMotorEncoderPositions();
            telemetry.addData("Positioning", ":%7d :%7d :%7d :%7d",
                    motorTicks[0], motorTicks[1], motorTicks[2], motorTicks[3]);
            telemetry.update();
        }
    }
}
