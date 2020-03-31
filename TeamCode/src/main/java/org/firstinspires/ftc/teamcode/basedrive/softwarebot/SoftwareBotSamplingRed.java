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

@Autonomous(name = "Red: Quarry Bridge", group = "Electrobot")
@Disabled
public class SoftwareBotSamplingRed extends LinearOpMode {

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    /* Declare OpMode members. */
    private SoftwareBot robot = new SoftwareBot();   // Initializing our robot
    private ElapsedTime     runtime = new ElapsedTime();

    private static final double flMaxVel = 2760.0;
    private static final double frMaxVel = 2660.0;
    private static final double blMaxVel = 2700.0;
    private static final double brMaxVel = 2700.0;
    private static final double posPIDF = 5.0;
    int ssPos1;
    int ssPos2;
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

        ssPos1 = getFirstSkyStonePosition();

        switch (ssPos1) {
            case 1:
                getSkystoneFromPos1();
                getSkystoneFromPos4();
                break;
            case 2:
                getSkystoneFromPos2();
                getSkystoneFromPos5();
                break;
            case 3:
                getSkystoneFromPos3();
                getSkystoneFromPos6();
        }

        parkUnderBridge();
        telemetry.addData("Status: ", "Auto Complete");
        telemetry.update();

        this.sleep(30000);

    }


    private void checkDriveMotors()
    {
        while ((opModeIsActive()) && robot.isRobotMoving())
        {
            //robot.logDriveEncoderPosition();
            int[] motorTicks = robot.getDriveMotorEncoderPositions();
            telemetry.addData("Positioning", ":%7d :%7d :%7d :%7d",
                    motorTicks[0], motorTicks[1], motorTicks[2], motorTicks[3]);
            telemetry.addData("Z Angle:", robot.getHeading());
            telemetry.update();

        }
    }

    private int getFirstSkyStonePosition() {
        int pos = 2;

        //move towards Quarry
        Log.i(ElectroBot.LOGTAG, "moving towards quarry");
        telemetry.addData("Status:", "Moving towards quarry");
        telemetry.update();
        //robot.logDriveEncoderPosition();
        robot.setTravelDistance(-27);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);
        Log.i(SoftwareBot.LOGTAG, "Checking for skystone");
        /*if (robot.isYellow()) {*/
            Log.i(ElectroBot.LOGTAG, "Not First block, checking next block");
            telemetry.addData("Status:", "Not First block, checking next block");
            telemetry.update();
        robot.travelLeft(-9);
            robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
            checkDriveMotors();
            robot.setDriveMotorPower(0, 0, 0, 0);

           /* if(robot.isYellow()){
                Log.i(SoftwareBot.LOGTAG, "Position 3");
                telemetry.addData("status:", "Position = 3");
                telemetry.update();
                pos = 3;
            }
            else{
                Log.i(SoftwareBot.LOGTAG, "Position 2");
                telemetry.addData("status:", "Position = 2");
                telemetry.update();
                pos = 2;
            }
        }
        else{
            Log.i(SoftwareBot.LOGTAG, "Position 1");
            telemetry.addData("status:", "Position = 1");
            telemetry.update();
            pos=1;
        }*/
        pos = 3;
        return pos;
    }

    public void getSkystoneFromPos1() {

        Log.i(SoftwareBot.LOGTAG, "strafe right a bit to line up with edge of stone");
        robot.travelRight(-3);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        Log.i(SoftwareBot.LOGTAG, "Turn 30 degrees CCW");
        robot.setDriveMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.turnCounterClockwise(25, .5);
        this.sleep(10);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Log.i(ElectroBot.LOGTAG, "Travel forward while running intake motors");
        //robot.logDriveEncoderPosition();
        /**** ADD CODE TO TURN ON INTAKE MOTORS ****/
        robot.setTravelDistance(-8);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        Log.i(SoftwareBot.LOGTAG, "Turning back to original position");
        robot.setDriveMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.turnClockwise(30, .5);
        this.sleep(10);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Log.i(SoftwareBot.LOGTAG, "Travel back towards alliance wall");
        robot.setTravelDistance(15);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        Log.i(SoftwareBot.LOGTAG, "Strafe under the bridge");
        robot.travelRight(-30);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        Log.i(ElectroBot.LOGTAG, "Turn 90 deg");
        //robot.logDriveEncoderPosition();
        robot.setDriveMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.turnCounterClockwise(90, .5);
        this.sleep(10);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        dropSkyStone();


    }

    public void getSkystoneFromPos2() {

        Log.i(SoftwareBot.LOGTAG, "Travel back to first stone");
        robot.travelRight(-9);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        Log.i(ElectroBot.LOGTAG, "Travelling towards wall before turning");
        //robot.logDriveEncoderPosition();
        robot.setTravelDistance(1);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        Log.i(ElectroBot.LOGTAG, "Turn 90 deg");
        //robot.logDriveEncoderPosition();
        robot.setDriveMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.turnCounterClockwise(90, .5);
        this.sleep(10);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Log.i(ElectroBot.LOGTAG, "Strafe right and push the first block out");
        //robot.logDriveEncoderPosition();
        robot.travelRight(-19);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        Log.i(ElectroBot.LOGTAG, "Travel forward while running intake motors");
        //robot.logDriveEncoderPosition();
        /**** ADD CODE TO TURN ON INTAKE MOTORS ****/
        robot.setTravelDistance(-6);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        Log.i(ElectroBot.LOGTAG, "Strafe left to line up under the bridge");
        //robot.logDriveEncoderPosition();
        robot.travelLeft(-30);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        Log.i(ElectroBot.LOGTAG, "Travel back to cross the bridge and drop the stone");
        //robot.logDriveEncoderPosition();
        robot.setTravelDistance(30);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        dropSkyStone();
    }

    public void getSkystoneFromPos3() {

        Log.i(ElectroBot.LOGTAG, "Travelling towards wall before turning");
        //robot.logDriveEncoderPosition();
        robot.setTravelDistance(1);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        Log.i(ElectroBot.LOGTAG, "Turn 90 deg");
        //robot.logDriveEncoderPosition();
        robot.setDriveMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.turnCounterClockwise(90, .5);
        this.sleep(10);

        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Log.i(ElectroBot.LOGTAG, "Strafe right and push the first block out");
        //robot.logDriveEncoderPosition();
        robot.travelRight(-21);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        Log.i(ElectroBot.LOGTAG, "Travel forward while running intake motors");
        //robot.logDriveEncoderPosition();
        /**** ADD CODE TO TURN ON INTAKE MOTORS ****/
        robot.setTravelDistance(-6);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        Log.i(ElectroBot.LOGTAG, "Strafe left to line up under the bridge");
        //robot.logDriveEncoderPosition();
        robot.travelLeft(-30);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        Log.i(ElectroBot.LOGTAG, "Travel back to cross the bridge and drop the stone");
        //robot.logDriveEncoderPosition();
        robot.setTravelDistance(30);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);

        dropSkyStone();
    }


    public void getSkystoneFromPos4() {
        /***
         * 1. Travel forward
         * 2. Strafe Right to push out two stones
         * 3. Start intake and drive forward
         * 4. Strafe left to line up under bridge
         * 5. Travel back towards building zone
         * 6. drop skystone
         */

    }

    public void getSkystoneFromPos5() {

    }

    public void getSkystoneFromPos6() {

    }

    public void parkUnderBridge() {

        Log.i(ElectroBot.LOGTAG, "Travel to park under the bridge");
        //robot.logDriveEncoderPosition();
        robot.setTravelDistance(-15);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(0.5, 0.5, 0.5, 0.5);
        checkDriveMotors();
        robot.setDriveMotorPower(0, 0, 0, 0);
    }

    public void dropSkyStone() {
        /****
         * 1. Open WG Servo
         * 2. Turn on Intake motors
         * 3. Move forward for few inches.
         */
    }

}

