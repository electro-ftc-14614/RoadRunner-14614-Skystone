package org.firstinspires.ftc.teamcode.basedrive.approved;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.basedrive.hwmap.ElectroBot;
import org.firstinspires.ftc.teamcode.basedrive.hwmap.SoftwareBot;

/**
 * This is Autonomous code to reposition the foundation into building zone and then park under the bridge
 */

@Autonomous(name = "BLUE:BUILDING:WALL", group = "Electrobot")
//@Disabled
public class Blue_Build_Wall extends LinearOpMode {

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
        moveRobot(ElectroBot.DIRECTIONS.RIGHT, 13.0);

        //move towards WaffleMaker
        Log.i(ElectroBot.LOGTAG, "continue at slower speed");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 6.0, 0.3);

        Log.i(ElectroBot.LOGTAG, "At the Waffle maker, dropping anchors");
        robot.setWGToDownPosition();  //putting servo down
        this.sleep(500);

        Log.i(ElectroBot.LOGTAG, "Pulling foundation towards wall");
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 5.0);

        Log.i(SoftwareBot.LOGTAG, "Turn waffle grabber 90 deg");
        robot.turnBiasCounterClockwise(83, 0, 0.7);
        this.sleep(10);

        //Pull up anchors from foundation
        Log.i(ElectroBot.LOGTAG, "Pulling up anchors from Waffle maker");
        robot.setWGToUpPosition();
        this.sleep(500);

        //travel to be in the middle of waffle maker
        //travel to be in the middle of waffle maker
        Log.i(ElectroBot.LOGTAG, "Travel right towards middle of waffle grabber");
        moveRobot(ElectroBot.DIRECTIONS.LEFT, 15.0);

        //Push waffle grabber towards wall
        Log.i(ElectroBot.LOGTAG, "push waffle grabber towards Wall");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 16);

        Log.i(ElectroBot.LOGTAG, "Strafe left towards team wall");
        moveRobot(ElectroBot.DIRECTIONS.RIGHT, 2);


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