package org.firstinspires.ftc.teamcode.basedrive.testing;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.basedrive.hwmap.ElectroBot;

import static java.lang.Math.abs;

/**
 * This is Blue side Autonomous for Electro 14614.
 * 1. Robot starts on Quarry Side
 * 2. Align the robot along the wall and line up color sensor to be on the tabs
 * 3. Robot moves and samples the first stone.
 * 4. If it's Yellow, moves right and samples second stone, else assigns skystone positions as 1,4
 * 5. If it's also yellow, then assigns skystone as 3, 6
 * 6. Else assigns skystones as 2, 5
 *
 * 7. Following steps are performed for first position
 *  --  Intakes the skystone
 *  --  Travels under the bridge to foundation
 *  --  Moves the foundation into Building zone
 *  --  Places the stone on the foundation
 *  --  Travels back to Quarry side to intake second stone
 *
 *  8. Intakes the second stone based on the position values determined during sampling
 *  9. Travels completly under the team bridge to count skystone scoring
 * 10. Travels back to be under the bridge for parking points.
 *
 * Check comments in the code to get the steps.
 */

@Autonomous(name = "DriveModeTest", group = "1Test")
@Disabled
public class driveModeTest extends LinearOpMode {

    static final double DRIVE_SPEED = 0.55;
    static final double TURN_SPEED = 0.3;
    static final double START_SPEED = 0.3;
    static final double SPEED_INCR = 0.075;
    static final double END_SPEED = 0.3;
    static final double END_SPEEDf = 0.3;
    static final double END_SPEEDb = -0.3;
    private static final double posPIDF = 15;

    private double F = 0.0;

    private int ssPos1;
    private int ssPos2;

    private double bridgeHeading = 0;
    private double quarryHeading = 0;
    private double headingToUse = 0.0;
    private boolean isForward = false;
    private boolean isStrafe = false;
    private boolean isRight = false;

    /* Declare OpMode members. */
    private ElectroBot robot = new ElectroBot();  // Initializing our robot
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        Log.i(ElectroBot.LOGTAG, "BQB: Initializing Robot: ");
        robot.init(hardwareMap);

        int encoderPos;


        Log.i(ElectroBot.LOGTAG, "BQB: Setting velocity PIDF values");
        F = 32767.0 / 2700.0;
        //robot.setMotorPIDF("FL", 0.1 * F, 0.01 * F, 0.0, F);
        //robot.setMotorPIDF("BL", 0.1 * F, 0.01 * F, 0.0, F);
        //robot.setMotorPIDF("FR", 0.1 * F, 0.01 * F, 0.0, F);
        //robot.setMotorPIDF("BR", 0.1 * F, 0.01 * F, 0.0, F);

        Log.i(ElectroBot.LOGTAG, "BQB: Setting Position PIDF");
        //robot.setPositionPIDF(posPIDF);

        // Send telemetry message to signify robot waiting;
        Log.i(ElectroBot.LOGTAG, "BQB: Initialized robot");
        telemetry.addData("Status", "Initialized Robot");    //
        telemetry.update();

        Log.i(ElectroBot.LOGTAG, "BQB: Calibrating Gyro");
        telemetry.addData("Status:", "Calbrating Gyro");
        telemetry.update();
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /*while (!robot.isGyroCalibrated())
        {
            this.sleep(50);
            this.idle();
        }
*/

        // Wait for the game to start (driver presses PLAY)
        Log.i(ElectroBot.LOGTAG, "BQB: Wait for start");
        telemetry.update();

        bridgeHeading = robot.getYaw();
        headingToUse = bridgeHeading;

        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Log.i(ElectroBot.LOGTAG, "Heading to USE: " + headingToUse);

        telemetry.addData("Status", "Press Play to begin");
        telemetry.update();
        waitForStart();

        moveRobot(ElectroBot.DIRECTIONS.FORWARD,80,0.7,true);





        Log.i(ElectroBot.LOGTAG, "starting Move Forward");
        //moveRobot(ElectroBot.DIRECTIONS.FORWARD,96,0.7,true);
        this.sleep(200);
        //moveRobot(ElectroBot.DIRECTIONS.RIGHT,60,1,false);

        //Log.i(ElectroBot.LOGTAG, "Starting the backward motion");
        //this.sleep(3000);

        //moveRobot(ElectroBot.DIRECTIONS.BACKWARD,96,1,true);
       // this.sleep(200);
        //moveRobot(ElectroBot.DIRECTIONS.LEFT,60,1,false);
        this.sleep(10000);
    }

    private void moveRobot(ElectroBot.DIRECTIONS direction, double distance, double speed, boolean checkHeading) {
        switch (direction) {
            case FORWARD:
                robot.setTravelDistance(distance);
                isForward = true;
                isStrafe = false;
                isRight = false;
                break;
            case BACKWARD:
                robot.setTravelDistance(-1 * distance);
                isForward = false;
                isStrafe = false;
                isRight = false;
                break;
            case LEFT:
                robot.travelLeft(distance);
                isForward = false;
                isRight = false;
                isStrafe = true;
                break;
            case RIGHT:
                robot.travelRight(distance);
                isForward = false;
                isRight = true;
                isStrafe = true;

                break;
        }
        Log.i(ElectroBot.LOGTAG, "Setimng to RUN_TO_POSITION");
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Log.i(ElectroBot.LOGTAG, "Setimng START_SPEED: " + START_SPEED);
        robot.setDriveMotorPower(START_SPEED,START_SPEED,START_SPEED,START_SPEED);
        if (checkHeading&&!isStrafe) {
            Log.i(ElectroBot.LOGTAG, "going into checkDriveMotorWithHeading");
            checkDriveMotorsWithHeading(speed, distance);
        } else if(isStrafe&&checkHeading){
            checkDriveMotorsStrafeHeading(speed,distance,isRight);
        }
        else {
            checkDriveMotors(speed, distance);
        }
        robot.setDriveMotorPower(0, 0, 0, 0);
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Log.i(ElectroBot.LOGTAG, "stop and reset encoders");
    }

    private void checkDriveMotorsStrafeHeading(double speed, double targetpos,boolean isRight){
       double z_angle = 0;
       double error;
       double steer;
       double local_speedf = START_SPEED;
       double local_speedb = -START_SPEED;
       double encoderticks1 = robot.getDriveMotorEncoderPositions()[1];
       double encoderticks2 = robot.getDriveMotorEncoderPositions()[2];
       targetpos=robot.COUNTS_PER_INCH*targetpos;
       while((opModeIsActive())&&robot.isRobotMoving()){
           error = headingToUse - robot.getYaw();
           if (local_speedf < speed)
               local_speedf += SPEED_INCR;
           if(local_speedf>=END_SPEEDf&&(encoderticks1>=.75*targetpos)&&(encoderticks2>=.75*targetpos)){
               local_speedf-= SPEED_INCR+0.075;
           }
           if (local_speedb > speed)
               local_speedb -= SPEED_INCR;
           if(local_speedb<=END_SPEEDb&&(encoderticks1<=.75*targetpos)&&(encoderticks2<=.75*targetpos)){
               local_speedb+= SPEED_INCR+0.075;
           }
           if (abs(error) > 0.5) {
               steer = robot.getSteer(-1 * error, 0.045);

               if(!isRight)
               {
                   robot.setDriveMotorPower(local_speedf - steer, local_speedb + steer, local_speedb + steer, local_speedf - steer);
               }
               else{
                   robot.setDriveMotorPower(local_speedb + steer, local_speedf - steer, local_speedf - steer, local_speedb + steer);
               }
           }
           else if(isRight)
               robot.setDriveMotorPower(local_speedf, local_speedb, local_speedb, local_speedf);
           else
               robot.setDriveMotorPower(local_speedb, local_speedf, local_speedf, local_speedb);

           //z_angle = robot.getYaw();
           //telemetry.addData("Z Angle:", z_angle);
           telemetry.addData("Backward Speed: ", local_speedb);
           telemetry.addData("Forward Speed:", local_speedf);

           telemetry.update();
       }

    }
    /*
     * Checks if the motors have reached their target position
     */
    private void checkDriveMotors(double speed,double targetpos) {
        double z_Angle = 0.0;
        double local_speed = START_SPEED;
        double encoderticks1 = robot.getDriveMotorEncoderPositions()[1];
        double encoderticks2 = robot.getDriveMotorEncoderPositions()[2];
        targetpos=robot.COUNTS_PER_INCH*targetpos;
        while ((opModeIsActive()) && robot.isRobotMoving()) {
            //robot.logDriveEncoderPosition();
            //int[] motorTicks = robot.getDriveMotorEncoderPositions();
            //telemetry.addData("Positioning", ":%7d :%7d :%7d :%7d", motorTicks[0], motorTicks[1], motorTicks[2], motorTicks[3]);
            if(encoderticks1/targetpos<0.75){
                if (local_speed < speed)
                    local_speed += SPEED_INCR;

            }
            else{
                if(local_speed>=START_SPEED){
                    local_speed-=(2*SPEED_INCR);
                }
            }



            //if(local_speed>=END_SPEED&&(encoderticks1>=.75*targetpos)&&(encoderticks2>=.75*targetpos)){
            //    local_speed-= SPEED_INCR+0.25;
            //}
            robot.setDriveMotorPower(local_speed, local_speed, local_speed, local_speed);
            z_Angle = robot.getYaw();
            telemetry.addData("Z Angle:", z_Angle);
            telemetry.update();
            //Log.i(ElectroBot.LOGTAG, "Current Heading: " + z_Angle);

        }
    }

    private void checkDriveMotorsWithHeading(double speed,double targetpos) {
        double z_Angle = 0.0;
        double local_speed = START_SPEED;
        double error = 0;
        double steer = 0;
        double encoderticks1 = robot.getDriveMotorEncoderPositions()[1];
        double encoderticks2 = robot.getDriveMotorEncoderPositions()[2];
        targetpos=robot.COUNTS_PER_INCH*targetpos;
        while ((opModeIsActive()) && robot.isRobotMoving()) {
            //robot.logDriveEncoderPosition();
            //int[] motorTicks = robot.getDriveMotorEncoderPositions();
            //telemetry.addData("Positioning", ":%7d :%7d :%7d :%7d", motorTicks[0], motorTicks[1], motorTicks[2], motorTicks[3]);
            error = headingToUse - robot.getYaw();
            if(encoderticks1/targetpos<0.75){
                if (local_speed < speed)
                    local_speed += SPEED_INCR;

            }
            else{
                if(local_speed>=START_SPEED){
                    local_speed-=(SPEED_INCR);
                }
            }
            Log.i(ElectroBot.LOGTAG,"LocalSpeed"+local_speed);
            encoderticks1=robot.getDriveMotorEncoderPositions()[1];
/*
            else if(local_speed>=END_SPEED&&(encoderticks1>=.75*targetpos)&&(encoderticks2>=.75*targetpos)){
               Log.i(ElectroBot.LOGTAG, "Decreasing speed");
                local_speed-= SPEED_INCR;

            }*/
            if (abs(error) > 0.5) {
                steer = robot.getSteer(-1 * error, 0.045);
//                Log.i(ElectroBot.LOGTAG, "Heading Error: " + error);
//                Log.i(ElectroBot.LOGTAG, "Heading Steer: " + steer);
//                Log.i(ElectroBot.LOGTAG, "LocalSpeed:" + local_speed);
                if(!isForward)
                {
                    robot.setDriveMotorPower(local_speed - steer, local_speed + steer, local_speed - steer, local_speed + steer);
                }
                else{
                    robot.setDriveMotorPower(local_speed + steer, local_speed - steer, local_speed + steer, local_speed - steer);
                }
            } else
                robot.setDriveMotorPower(local_speed, local_speed, local_speed, local_speed);
            //z_Angle = robot.getYaw();
            //telemetry.addData("Z Angle:", z_Angle);
            //telemetry.update();
            //Log.i(ElectroBot.LOGTAG, "Current Heading: " + z_Angle);

        }
    }}






