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

@Autonomous(name = "RED:BUILDING:WALL", group = "Electrobot")
//@Disabled
public class Red_Build_Wall extends LinearOpMode {

    static final double DRIVE_SPEED = 0.45;
    static final double DRIVE_SPEED_FAST = 0.55;
    static final double TURN_SPEED = 0.5;
    private static final double flMaxVel = 2760.0;
    private static final double frMaxVel = 2660.0;
    private static final double blMaxVel = 2700.0;
    private static final double brMaxVel = 2700.0;
    private static final double posPIDF = 5.0;
    /* Declare OpMode members. */
    private ElectroBot robot = new ElectroBot();   // Initializing our robot
    private ElapsedTime runtime = new ElapsedTime();

    @Override

    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        Log.i(ElectroBot.LOGTAG, "Initializing Robot: ");
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        Log.i(ElectroBot.LOGTAG, "Initialized robot");
        telemetry.addData("Status", "Initialized Robot");    //
        telemetry.update();


        /*SetVelocity PIDF based on max velocity of each motor.
        robot.setMotorPIDF("FL", 0.1 * (32767.0/flMaxVel), 0.1 *(0.1*(32767.0/flMaxVel)), 0, (32767.0/flMaxVel));
        robot.setMotorPIDF("FR", 0.1 * (32767.0/frMaxVel), 0.1 *(0.1*(32767.0/frMaxVel)), 0, (32767.0/frMaxVel));
        robot.setMotorPIDF("BL", 0.1 * (32767.0/blMaxVel), 0.1 *(0.1*(32767.0/blMaxVel)), 0, (32767.0/blMaxVel));
        robot.setMotorPIDF("BR", 0.1 * (32767.0/brMaxVel), 0.1 *(0.1*(32767.0/brMaxVel)), 0, (32767.0/brMaxVel));
        Log.i(ElectroBot.LOGTAG, "Setting Position PIDF");
        robot.setPositionPIDF(posPIDF);
        */

        // Wait for the game to start (driver presses PLAY)
        robot.resetDriveMotorEncoders();
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Press Play to begin");
        telemetry.update();
        waitForStart();

        //move towards WaffleMaker
        Log.i(ElectroBot.LOGTAG, "moving towards waffle maker");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 25.0);


        Log.i(ElectroBot.LOGTAG, "shifting to edge of waffle maker");
        moveRobot(ElectroBot.DIRECTIONS.LEFT, 13.0);

        //move towards WaffleMaker
        Log.i(ElectroBot.LOGTAG, "continue at slower speed");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 6.0, 0.3);

        Log.i(ElectroBot.LOGTAG, "At the Waffle maker, dropping anchors");
        robot.setWGToDownPosition();  //putting servo down
        this.sleep(500);

        //move towards team wall
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 5.0);

        Log.i(SoftwareBot.LOGTAG, "Turn waffle grabber 90 deg while travelling towards wall");
        robot.turnBiasClockwise(83, 0.7, 0);
        this.sleep(10);

        //Pull up anchors from foundation
        Log.i(ElectroBot.LOGTAG, "Pulling up anchors from Waffle maker");
        robot.setWGToUpPosition();
        this.sleep(500);

        //travel to be in the middle of waffle maker
        Log.i(ElectroBot.LOGTAG, "Travel right towards middle of waffle grabber");
        moveRobot(ElectroBot.DIRECTIONS.RIGHT, 15.0);

        //Push waffle grabber towards wall
        Log.i(ElectroBot.LOGTAG, "push waffle grabber towards Wall");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 16);

        Log.i(ElectroBot.LOGTAG, "Strafe left towards team wall");
        moveRobot(ElectroBot.DIRECTIONS.LEFT, 3);


        Log.i(ElectroBot.LOGTAG, "Travel towards bridge to park under");
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 45.0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        Log.i(ElectroBot.LOGTAG, "Building Auto complete");
        //This sleep to avoid accidental restart of Auto.
        this.sleep(30000);
    }

    protected void moveRobot(ElectroBot.DIRECTIONS direction, double distance) {
        moveRobot(direction, distance, DRIVE_SPEED);
    }


    private void checkDriveMotors() {
        while ((opModeIsActive()) && robot.isRobotMoving()) {
            //robot.logDriveEncoderPosition();
            int[] motorTicks = robot.getDriveMotorEncoderPositions();
            telemetry.addData("Positioning", ":%7d :%7d :%7d :%7d",
                    motorTicks[0], motorTicks[1], motorTicks[2], motorTicks[3]);
            telemetry.update();

        }
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
