package org.firstinspires.ftc.teamcode.basedrive.softwarebot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.basedrive.hwmap.ElectroBot;
import org.firstinspires.ftc.teamcode.basedrive.hwmap.SoftwareBot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "SOFTWAREBOT_PID", group = "Electrobot")
@Disabled
public class SoftwareBotSamplingBlue extends LinearOpMode {

    static final double DRIVE_SPEED = 0.9;
    static final double TURN_SPEED = 0.5;
    static final double START_SPEED = 0.1;
    static final double SPEED_INCR = 0.05;
    /* Declare OpMode members. */
    private SoftwareBot robot = new SoftwareBot();   // Initializing our robot
    private ElapsedTime     runtime = new ElapsedTime();


    private static final double flMaxVel = 2760.0;
    private static final double frMaxVel = 2660.0;
    private static final double blMaxVel = 2700.0;
    private static final double brMaxVel = 2700.0;
    private static final double posPIDF = 10.0;
    private static double P, I, D, F;
    int pos1;
    int pos2;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        double wallHeading = 0;
        double bridgeHeading = 0;
        double turnError = 0;

        Log.i(SoftwareBot.LOGTAG, "Initializing Robot: ");
        robot.init(hardwareMap);

        Log.i(ElectroBot.LOGTAG, "Setting Position PIDF");

        // Send telemetry message to signify robot waiting;
        Log.i(SoftwareBot.LOGTAG, "Initialized robot");
        telemetry.addData("Status", "Initialized Robot");    //
        telemetry.update();
        robot.resetDriveMotorEncoders();

        Log.i(SoftwareBot.LOGTAG, "Initializing IMU");
        telemetry.addData("Status: ", "Initializing Gyro");
        robot.initIMU();
        //while (!robot.isGyroCalibrated()) {
        //    this.sleep(50);
        //    this.idle();
        //}

        F = 32767.0 / 2700.0;
        P = 0.1 * F;
        I = 0.1 * P;
        D = 0.0;

        robot.setMotorPIDF("FL", P, I, D, F);
        robot.setMotorPIDF("FR", P, I, D, F);
        robot.setMotorPIDF("BL", P, I, D, F);
        robot.setMotorPIDF("BR", P, I, D, F);
        robot.setPositionPIDF(posPIDF);

        // Wait for the game to start (driver presses PLAY)
        Log.i(ElectroBot.LOGTAG, "Wait for start");
        telemetry.addData("Status", "Press Play to begin");
        waitForStart();
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bridgeHeading = robot.getRobotHeading();

        Log.i(SoftwareBot.LOGTAG, "Starting Heading: " + bridgeHeading);

        //move towards building zone
        Log.i(ElectroBot.LOGTAG, "moving towards blocks");
        robot.logDriveEncoderPosition();
        robot.setTravelDistance(-20);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(START_SPEED, START_SPEED, START_SPEED, START_SPEED);
        checkDriveMotors(DRIVE_SPEED, bridgeHeading);
        //checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Log.i(SoftwareBot.LOGTAG, "Forward Complete");

        robot.turnClockwise(85, DRIVE_SPEED);
        Log.i(SoftwareBot.LOGTAG, "Turn complete");
        wallHeading = robot.getRobotHeading();
        Log.i(SoftwareBot.LOGTAG, "Heading to Keep: " + wallHeading);

        //sleep(5000);

        robot.travelRight(-6);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(START_SPEED, START_SPEED, START_SPEED, START_SPEED);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Log.i(SoftwareBot.LOGTAG, "Right Complete" + ":: Cur Robot Heading: " + robot.getRobotHeading());


        robot.travelLeft(-10);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(START_SPEED, START_SPEED, START_SPEED, START_SPEED);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Log.i(SoftwareBot.LOGTAG, "Left Complete" + ":: Cur Robot Heading: " + robot.getRobotHeading());
        sleep(5000);

       /* if(abs(robot.getRobotHeading() - wallHeading) > 1 ) {
            double headingError = wallHeading - robot.getRobotHeading();
            if (headingError > 0)
                robot.turnCounterClockwise(headingError, TURN_SPEED);
            if(headingError < 0)
                robot.turnClockwise(-1 * turnError, TURN_SPEED);
        }

        */


        robot.setTravelDistance(90);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(START_SPEED, START_SPEED, START_SPEED, START_SPEED);
        checkDriveMotors(DRIVE_SPEED, wallHeading);
        robot.setDriveMotorPower(0, 0, 0, 0);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Log.i(SoftwareBot.LOGTAG, "Back Complete");


        this.sleep(30000);
    }


    private void checkDriveMotors()
    {
        double local_speed = START_SPEED;

        while ((opModeIsActive()) && robot.isRobotMoving())
        {
            if (local_speed < DRIVE_SPEED)
                local_speed += SPEED_INCR;

            robot.setDriveMotorPower(local_speed, local_speed, local_speed, local_speed);
            int[] motorTicks = robot.getDriveMotorEncoderPositions();
            telemetry.addData("Positioning", ":%7d :%7d :%7d :%7d",
                    motorTicks[0], motorTicks[1], motorTicks[2], motorTicks[3]);
            telemetry.update();

        }
        robot.logDriveEncoderPosition();
    }

    private void checkDriveMotors(double speed, double heading) {
        double local_speed = START_SPEED;
        double headingError = 0;
        double steer = 0.0;
        while ((opModeIsActive()) && robot.isRobotMoving()) {
            if (local_speed < speed)
                local_speed += SPEED_INCR;

            headingError = robot.checkHeading(heading);
            steer = -1 * (robot.getSteer(headingError, 0.1));

            Log.i(SoftwareBot.LOGTAG, "Heading Error: " + headingError + ":: Steer : " + steer + ":: Heading: " + robot.getRobotHeading());

            //setDriveMotorPower(double flPower, double frPower, double blPower, double brPower)
            robot.setDriveMotorPower(local_speed - steer, local_speed + steer, local_speed - steer, local_speed + steer);

        }

    }
}

