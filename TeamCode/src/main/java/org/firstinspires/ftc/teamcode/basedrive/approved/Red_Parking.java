package org.firstinspires.ftc.teamcode.basedrive.approved;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.basedrive.hwmap.ElectroBot;
import org.firstinspires.ftc.teamcode.basedrive.hwmap.SoftwareBot;

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

@Autonomous(name = "RED:PARKING", group = "Electrobot")
//@Disabled
public class Red_Parking extends LinearOpMode {

    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.35;
    private static final double flMaxVel = 2760.0;
    private static final double frMaxVel = 2660.0;
    private static final double blMaxVel = 2700.0;
    private static final double brMaxVel = 2700.0;
    private static final double posPIDF = 5.0;
    int ssPos1;
    int ssPos2;
    /* Declare OpMode members. */
    private ElectroBot robot = new ElectroBot();  // Initializing our robot
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        Log.i(SoftwareBot.LOGTAG, "Initializing Robot: ");
        robot.init(hardwareMap);

        Log.i(ElectroBot.LOGTAG, "Setting Position PIDF");
        robot.setPositionPIDF(posPIDF);
        // Send telemetry message to signify robot waiting;
        Log.i(SoftwareBot.LOGTAG, "Initialized robot");
        telemetry.addData("Status", "Initialized Robot");    //
        telemetry.update();

        Log.i(SoftwareBot.LOGTAG, "Calibrating Gyro");
        telemetry.addData("Status:", "Calbrating Gyro");
        telemetry.update();
        while (!robot.isGyroCalibrated()) {
            this.sleep(50);
            this.idle();
        }


        // Wait for the game to start (driver presses PLAY)
        Log.i(ElectroBot.LOGTAG, "Wait for start");
        telemetry.update();
        telemetry.addData("Status", "Press Play to begin");
        telemetry.update();
        waitForStart();
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Log.i(ElectroBot.LOGTAG, "Strafe right to not hit the wall and crack the polycarbonate");
        //robot.logDriveEncoderPosition();
        moveRobot(ElectroBot.DIRECTIONS.RIGHT, 3.0);

        Log.i(ElectroBot.LOGTAG, "Drive forward to park under bridge");
        //robot.logDriveEncoderPosition();
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 14.0);

        moveRobot(ElectroBot.DIRECTIONS.LEFT, 3.0);

        telemetry.addData("Status: ", "Auto Complete");
        telemetry.update();

        this.sleep(30000);

    }

    protected void checkDriveMotors() {
        double z_Angle = 0.0;
        while ((opModeIsActive()) && robot.isRobotMoving()) {
            z_Angle = robot.getHeading();
            telemetry.addData("Z Angle:", z_Angle);
            telemetry.update();
            Log.i(ElectroBot.LOGTAG, "Current Heading: " + z_Angle);

        }
    }

    protected void moveRobot(ElectroBot.DIRECTIONS direction, double distance) {
        moveRobot(direction, distance, DRIVE_SPEED);
    }

    private void moveRobot(ElectroBot.DIRECTIONS direction, double distance, double speed) {
        switch (direction) {
            case FORWARD:
                robot.setTravelDistance(distance);
                break;
            case BACKWARD:
                robot.setTravelDistance(-1 * distance);
                break;
            case LEFT:
                robot.travelLeft(distance);
                break;
            case RIGHT:
                robot.travelRight(distance);
                break;
        }
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(speed, speed, speed, speed);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);
    }
}

