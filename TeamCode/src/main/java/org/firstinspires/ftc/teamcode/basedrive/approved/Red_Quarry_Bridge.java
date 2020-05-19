package org.firstinspires.ftc.teamcode.basedrive.approved;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.basedrive.hwmap.ElectroBot;

import static java.lang.Math.abs;

/**
 * This is Red side Autonomous for Electro 14614.
 * 1. Robot starts on Quarry Side
 * 2. Align the robot along the wall and line up color sensor to be on the tabs
 * 3. Robot moves and samples the first stone.
 * 4. If it's Yellow, moves left and samples second stone, else assigns skystone positions as 1,4
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
 *
 */

@Autonomous(name = "RED:Quarry:Bridge", group = "1Main_Auto")
//@Disabled
public class Red_Quarry_Bridge extends LinearOpMode {

    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.3;
    static final double START_SPEED = 0.1;
    static final double SPEED_INCR = 0.075;
    static final double END_SPEED = 0.3;
    static final double END_SPEEDf = 0.3;
    static final double END_SPEEDb = -0.3;
    private static final double posPIDF = 5;

    private double F = 0.0;

    int ssPos1;
    int ssPos2;

    private double bridgeHeading;
    private double quarryHeading;
    private double headingToUse;
    private boolean isForward = false;
    private boolean isStrafe = false;
    private boolean isRight = false;

    /* Declare OpMode members. */
    private ElectroBot robot = new ElectroBot();  // Initializing our robot
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        Log.i(ElectroBot.LOGTAG, "RQB: Initializing Robot: ");
        robot.init(hardwareMap);

        Log.i(ElectroBot.LOGTAG, "RQB: Setting velocity PIDF values");
        F = 32767.0 / 2700.0;
        //robot.setMotorPIDF("FL", 0.1 * F, 0.01 * F, 0.0, F);
        //robot.setMotorPIDF("BL", 0.1 * F, 0.01 * F, 0.0, F);
        //robot.setMotorPIDF("FR", 0.1 * F, 0.01 * F, 0.0, F);
        //robot.setMotorPIDF("BR", 0.1 * F, 0.01 * F, 0.0, F);

        Log.i(ElectroBot.LOGTAG, "RQB: Setting Position PIDF");
        robot.setPositionPIDF(posPIDF);

        // Send telemetry message to signify robot waiting;
        Log.i(ElectroBot.LOGTAG, "RQB: Initialized robot");
        telemetry.addData("Status", "Initialized Robot");    //
        telemetry.update();

        Log.i(ElectroBot.LOGTAG, "RQB: Calibrating Gyro");
        telemetry.addData("Status:", "Calbrating Gyro");
        telemetry.update();
        while (!robot.isGyroCalibrated()) {
            this.sleep(50);
            this.idle();
        }

        // Wait for the game to start (driver presses PLAY)
        Log.i(ElectroBot.LOGTAG, "RQB: Wait for start");
        telemetry.update();

        robot.resetDriveMotorEncoders();
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Press Play to begin");
        telemetry.update();
        waitForStart();

        quarryHeading = robot.getYaw();
        headingToUse = quarryHeading;
        Log.i(ElectroBot.LOGTAG, "RQB: Quarry Heading: " + quarryHeading);

        ssPos1 = getFirstSkyStonePosition();

        telemetry.addData("Status: ", "Sampling Skytone");
        telemetry.update();
        switch (ssPos1) {
            case 1:
                getSkystoneFromPos1();

                Log.i(ElectroBot.LOGTAG, "RQB: This moves foundation into Building zone and turns 90 deg");
                moveFoundation(1);

                getSkystoneFromPos4();

                break;
            case 2:
                getSkystoneFromPos2();

                Log.i(ElectroBot.LOGTAG, "RQB: This moves foundation into Building zone and turns 90 deg");
                moveFoundation(2);

                getSkystoneFromPos5();
                break;
            case 3:
                getSkystoneFromPos3();

                Log.i(ElectroBot.LOGTAG, "RQB: This moves foundation into Building zone and turns 90 deg");
                moveFoundation(3);

                getSkystoneFromPos6();
        }

        Log.i(ElectroBot.LOGTAG, "RQB: travelling towards bridge to park");
        telemetry.addData("Status: ", "travelling towards bridge to park");
        telemetry.update();
        parkUnderBridge();

        telemetry.addData("Status: ", "Auto Complete");
        telemetry.update();
        if (isStopRequested()) {
            stop();
        }

        Log.i(ElectroBot.LOGTAG, "RQB: AUTO COMPLETE");
        robot.setDriveMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.setDriveMotorPower(0, 0, 0, 0);

        this.sleep(30000);

    }


    protected int getFirstSkyStonePosition() {
        int pos = 3;

        //move towards Quarry
        Log.i(ElectroBot.LOGTAG, "RQB: moving towards quarry");
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 24,0.475, true);

        Log.i(ElectroBot.LOGTAG, "RQB: Heading after travelling to quarry: " + robot.getYaw());

        this.sleep(250);
        double csRatio = robot.getColorSensorRatio();
        if (csRatio <= 0.7)
            pos = 2;
        else if (csRatio > 1.6)
            pos = 1;
        else
            pos = 3;
        Log.i(ElectroBot.LOGTAG, "RQB: Position of Skystone is: " + pos);
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD,3,false);

        return pos;
    }

    protected void getSkystoneFromPos1() {

        Log.i(ElectroBot.LOGTAG, "RQB: Strafe right 14 in");
        moveRobot(ElectroBot.DIRECTIONS.RIGHT, 18.5, false);

        Log.i(ElectroBot.LOGTAG, "RQB: Turning counter clockwise ");
        robot.turnCounterClockwise(36, TURN_SPEED);

        this.sleep(100);

        Log.i(ElectroBot.LOGTAG, "RQB: Travel forward while running intake motors");
        turnOnIntakeMotors();
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 12.0, false);
        robot.resetGripperServoPosition();
        Log.i(ElectroBot.LOGTAG, "RQB: travel back to line up under the bridge");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 9.5, false);
        stopIntakeMotors();
        this.sleep(100);
        turnOnIntakeMotors();

        Log.i(ElectroBot.LOGTAG, "RQB: Turn 50 deg to line up the bot for travelling back.");
        robot.turnCounterClockwise(48.5, TURN_SPEED);
        stopIntakeMotors();

        this.sleep(200);

        bridgeHeading = robot.getYaw();
        headingToUse = bridgeHeading;
        Log.i(ElectroBot.LOGTAG, "Bridge Heading: " + headingToUse);

        Log.i(ElectroBot.LOGTAG, "RQB: Travel back to building zone cross the bridge");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 58,0.7, true);
    }

    protected void getSkystoneFromPos2() {

        Log.i(ElectroBot.LOGTAG, "RQB: Travel back towards bridge");
        moveRobot(ElectroBot.DIRECTIONS.RIGHT, 11, false);

        Log.i(ElectroBot.LOGTAG, "RQB: Turn 90 deg");
        robot.turnCounterClockwise(83, TURN_SPEED);
        this.sleep(300);

        bridgeHeading = robot.getYaw();
        headingToUse = bridgeHeading;
        Log.i(ElectroBot.LOGTAG, "Bridge Heading: " + headingToUse);


        Log.i(ElectroBot.LOGTAG, "RQB: Strafe right and push the first block out");
        moveRobot(ElectroBot.DIRECTIONS.RIGHT, 18.0, false);

        Log.i(ElectroBot.LOGTAG, "RQB: Travel forward while running intake motors");
        turnOnIntakeMotors();
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 6.0, false);
        robot.resetGripperServoPosition();
        Log.i(ElectroBot.LOGTAG, "RQB: Strafe left to line up under the bridge and stop intake motors");
        moveRobot(ElectroBot.DIRECTIONS.LEFT, 15, false);
        stopIntakeMotors();

        this.sleep(100);

        Log.i(ElectroBot.LOGTAG, "RQB: Bridge Heading after strafe: " + robot.getYaw());
        //robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Log.i(ElectroBot.LOGTAG, "RQB: Travel back to building zone cross the bridge");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 70, 0.7, true);
    }

    protected void getSkystoneFromPos3() {

        Log.i(ElectroBot.LOGTAG, "RQB: Turn 40 deg");
        robot.turnCounterClockwise(34, TURN_SPEED);

        this.sleep(200);

        Log.i(ElectroBot.LOGTAG, "RQB: Travel forward while running intake motors");
        turnOnIntakeMotors();
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 15.0, 0.3, false);
        robot.resetGripperServoPosition();
        Log.i(ElectroBot.LOGTAG, "RQB: Travel back to line up under the bridge");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 12.25, false);
        stopIntakeMotors();
        this.sleep(100);
        turnOnIntakeMotors();

        Log.i(ElectroBot.LOGTAG, "RQB: Turn 50 deg to prepare to travel back");
        robot.turnCounterClockwise(54.5, TURN_SPEED);
        stopIntakeMotors();

        this.sleep(200);

        bridgeHeading = robot.getYaw();
        headingToUse = bridgeHeading;
        Log.i(ElectroBot.LOGTAG, "Bridge Heading: " + headingToUse);

        Log.i(ElectroBot.LOGTAG, "RQB: Travel back to building zone cross the bridge");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 74.5, 0.7,true);
    }

    protected void getSkystoneFromPos4() {
        robot.resetGripperServoPosition();
        Log.i(ElectroBot.LOGTAG, "RQB: travel back to quarry side");
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 75, false);
        //Log.i(ElectroBot.LOGTAG, "Bridge Heading after travel back to quarry: " + robot.getYaw());

        Log.i(ElectroBot.LOGTAG, "Intake second skystone from Pos 4");
        intakeSecondSkystone(1);
        robot.setWGToMidPosition();

        Log.i(ElectroBot.LOGTAG, "RQB: Travel back to cross the bridge and drop the stone");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 79,  true);
        robot.resetGripperServoPosition();
        robot.setFlipServoPosition();
        robot.setIntakeMotorPower(-0.45, -0.45);
        robot.setWheelServos(.9);
        this.sleep(500);
        stopIntakeMotors2();
        robot.resetGripperServoPosition();
    }

    protected void getSkystoneFromPos5() {
        robot.resetGripperServoPosition();
        Log.i(ElectroBot.LOGTAG, "RQB: travel back to quarry side");
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 82, false);

        //Log.i(ElectroBot.LOGTAG, "Bridge Heading after travel back to quarry: " + robot.getYaw());

        Log.i(ElectroBot.LOGTAG, "Intake second skystone from Pos 5");
        intakeSecondSkystone(2);
        robot.setWGToMidPosition();
        Log.i(ElectroBot.LOGTAG, "BQB: Travel back to cross the bridge");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 88,  true);
        robot.resetGripperServoPosition();
        robot.setFlipServoPosition();
        robot.setIntakeMotorPower(-0.45, -0.45);
        robot.setWheelServos(.9);
        this.sleep(700);
        stopIntakeMotors2();
        robot.resetGripperServoPosition();

        //Log.i(ElectroBot.LOGTAG, "Bridge Heading after travel back to building: " + robot.getYaw());
    }

    protected void getSkystoneFromPos6() {
        robot.resetGripperServoPosition();
        Log.i(ElectroBot.LOGTAG, "RQB: travel back to quarry side");
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 92, false);
        //moveRobot(ElectroBot.DIRECTIONS.LEFT,1,false);

        //Log.i(ElectroBot.LOGTAG, "Bridge Heading after travel back to quarry: " + robot.getYaw());

        Log.i(ElectroBot.LOGTAG, "Intake second skystone from Pos 5");
        intakeSecondSkystone(3);
        robot.setWGToMidPosition();

        Log.i(ElectroBot.LOGTAG, "RQB: Travel back to cross the bridge and drop the stone");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 94,  true);
        robot.resetGripperServoPosition();
        robot.setFlipServoPosition();
        robot.setIntakeMotorPower(-0.45, -0.45);
        robot.setWheelServos(.9);
        this.sleep(500);
        stopIntakeMotors2();
        robot.resetGripperServoPosition();

        Log.i(ElectroBot.LOGTAG, "Bridge Heading after travel back to building: " + robot.getYaw());
    }

    private void parkUnderBridge() {
        this.sleep(150);
        Log.i(ElectroBot.LOGTAG, "RQB: Travel towards bridge to park under");
        //moveRobot(ElectroBot.DIRECTIONS.LEFT, 2, .7, false);
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 34.0, 0.85, false);
        robot.setWGToUpPosition();
    }

    /*** NOT USED for MEET3 */
    private void dropSkyStone() {
        /****
         * 1. Open WG Servo
         * 2. Turn on Intake motors
         * 3. Move forward for few inches.
         */
        Log.i(ElectroBot.LOGTAG, "RQB: Open WG Servos");
        robot.setWGToDownPosition();
        this.sleep(400);
        Log.i(ElectroBot.LOGTAG, "RQB: Set flip servos's to close position");
        robot.setFlipServoPosition();
        Log.i(ElectroBot.LOGTAG, "RQB: Turn on Intake motors");
        robot.setIntakeMotorPower(-0.45, -0.45);
        robot.setWheelServos(.9);
        this.sleep(500);
        Log.i(ElectroBot.LOGTAG, "RQB: Pull up WG servos partially to release the ");
        robot.setWGToMidPosition();
        this.sleep(500);

    }
    protected void intakeFirstSkystone(){
        //moveRobot(ElectroBot.DIRECTIONS.BACKWARD,3,false);
        turnOnIntakeMotors();
        robot.turnBiasCounterClockwise(24,-TURN_SPEED/5.5,TURN_SPEED);
        robot.turnBiasCounterClockwise(19,TURN_SPEED,TURN_SPEED);

        moveRobot(ElectroBot.DIRECTIONS.BACKWARD,16,false);

        stopIntakeMotors();

        robot.turnCounterClockwise(40,TURN_SPEED);
    }

    protected void intakeSecondSkystone(double position){
        if(position==2){
            moveRobot(ElectroBot.DIRECTIONS.RIGHT,17.5,false);
        }
        else if(position==3){
            moveRobot(ElectroBot.DIRECTIONS.RIGHT,18,false);
        }
        else{
            moveRobot(ElectroBot.DIRECTIONS.RIGHT,18.5,false);
        }

        turnOnIntakeMotors();
        moveRobot(ElectroBot.DIRECTIONS.FORWARD,6,false);
        if(position==3){
            moveRobot(ElectroBot.DIRECTIONS.LEFT,15,false);

        }
        else if(position==2){
            moveRobot(ElectroBot.DIRECTIONS.LEFT,17.5,false);
        }
        else{
            moveRobot(ElectroBot.DIRECTIONS.LEFT,17,false);

        }
        stopIntakeMotors();
        robot.setGripperServoPosition();

        stopIntakeMotors();
        robot.setGripperServoPosition();

        /*Log.i(ElectroBot.LOGTAG, "RQB: Turn 42 deg");
        robot.turnClockwise(44, TURN_SPEED);

        this.sleep(200);

        Log.i(ElectroBot.LOGTAG, "RQB: Travel forward while running intake motors");
        turnOnIntakeMotors();
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 14.5, 0.3, false);

        Log.i(ElectroBot.LOGTAG, "RQB: travel back to be under the bridge");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 13.5, false);
        stopIntakeMotors();
        this.sleep(100);
        //turnOnIntakeMotors();

        Log.i(ElectroBot.LOGTAG, "RQB: Turn 45 deg");
        robot.turnCounterClockwise(43, TURN_SPEED);
        //stopIntakeMotors();
        this.sleep(200);*/
    }

    /*
     *  Runs all the steps needed to intake a stone
     * 1. Sets WG servos to UP position (These act as a backstop for the stone)
     * 2. Sets Flip servos to CLOSED postion
     * 3. Turns on Intake motors
     * 4. Turns on the servos to pull stone to the back so that it's ready to be placed
     */
    private void turnOnIntakeMotors() {
        robot.setWGToUpPosition();
        robot.setFlipServoPosition();
        robot.setIntakeMotorPower(-0.45, -0.45);
        robot.setWheelServos(.9);
    }

    /*
     * Runs all steps to stop intake motors
     * Sets WG servos to UP position
     * Stops intake motors
     * stops back wheel servos
     */
    private void stopIntakeMotors() {
        robot.setWGToUpPosition();
        robot.setIntakeMotorPower(0, 0);
        robot.setWheelServos(0.0);
    }

    private void stopIntakeMotors2() {
        robot.setIntakeMotorPower(0, 0);
        robot.setWheelServos(0.0);
    }
    /*
     * Moves the foundation and Places the first stone on the foundation
     */
    private void moveFoundation(int position) {
        //this.sleep(5000);
        telemetry.addData("Status: ", "At WM, turning");
        telemetry.update();
        Log.i(ElectroBot.LOGTAG, "RQB: Bridge Heading after travel: " + robot.getYaw());
        turnOnIntakeMotors();
        Log.i(ElectroBot.LOGTAG, "RQB: At Waffle maker. turn Counter Clockwise so that back of robot is  perpendicular to foundation");
        robot.turnCounterClockwise(88, TURN_SPEED);
        stopIntakeMotors();
        Log.i(ElectroBot.LOGTAG, "RQB: Grip the skystone");
        robot.setGripperServoPosition();

        //move towards WaffleMaker
        Log.i(ElectroBot.LOGTAG, "RQB: Travel back at slower speed towards foundation");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 10, 0.3, false);

        Log.i(ElectroBot.LOGTAG, "RQB: At the Waffle maker, dropping anchors, open flip servos, start moving the arm out");
        robot.setWGToDownPosition();  //putting servo down
        robot.resetFlipServoPosition(); //Opening up flip servos
        this.sleep(250);
        robot.setArmServos(-0.9); //extending ARM out

        //pulling WaffleMaker
        Log.i(ElectroBot.LOGTAG, "RQB: Travel back at slower speed towards team wall");
        if (position == 3) {
            moveRobot(ElectroBot.DIRECTIONS.FORWARD, 8, 0.3, false);

        } else {
            moveRobot(ElectroBot.DIRECTIONS.FORWARD, 7, 0.3, false);
        }

        Log.i(ElectroBot.LOGTAG, "RQB: Stop moving the ARM");
        robot.setArmServos(0.0);

        Log.i(ElectroBot.LOGTAG, "RQB: Turn waffle grabber 90 deg");
        robot.turnBiasClockwise(81, 0.7, 0.03);
        //this.sleep(10);

        //Pull up anchors from foundation
        Log.i(ElectroBot.LOGTAG, "RQB: Pulling up anchors from Waffle maker");
        robot.setWGToUpPosition();

        Log.i(ElectroBot.LOGTAG, "RQB: moving away from foundation to avoid dragging it");
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 2, false);

        Log.i(ElectroBot.LOGTAG, "RQB: move robot right, drop skystone and then retract arm");
        if(position==3){
            moveRobot(ElectroBot.DIRECTIONS.RIGHT,4.5,false);
        }
        else {
            moveRobot(ElectroBot.DIRECTIONS.RIGHT, 3.5, false);
        }
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 2, false);
        robot.resetGripperServoPosition();
        robot.resetFlipServoPosition();
        robot.setArmServos(0.85);
        this.sleep(800);
        robot.setArmServos(0.0);


        Log.i(ElectroBot.LOGTAG, "RQB: Move right to be in the middle of waffle grabber");
        //if(position==3){
        //  moveRobot(ElectroBot.DIRECTIONS.RIGHT,5,false);
        //}
        //else {
        moveRobot(ElectroBot.DIRECTIONS.RIGHT, 6, false);
        //}

        Log.i(ElectroBot.LOGTAG, "RQB: Pushing Waffle Grabber towards wall");
        if (position == 3)
            moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 11, false);
        else
            moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 11.5, false);

        //if (position == 3)
        //    moveRobot(ElectroBot.DIRECTIONS.LEFT, 2, false);

        robot.setArmServos(0.0);

        Log.i(ElectroBot.LOGTAG, "RQB: Stop and reset encoders before travelling back");
        robot.setDriveMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Log.i(ElectroBot.LOGTAG, "Bridge Heading before travel back to quarry: " + robot.getYaw());
    }

    private void moveRobot(ElectroBot.DIRECTIONS direction, double distance, boolean checkHeading) {
        moveRobot(direction, distance, DRIVE_SPEED, checkHeading);
    }


    private void moveRobot(ElectroBot.DIRECTIONS direction, double distance, double speed, boolean checkHeading) {
        switch (direction) {
            case FORWARD:
                robot.setTravelDistance(distance);
                isForward = true;
                break;
            case BACKWARD:
                robot.setTravelDistance(-1 * distance);
                isForward = false;
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

        robot.setDriveMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setDriveMotorPower(START_SPEED,START_SPEED,START_SPEED,START_SPEED);
        if(isStrafe || distance  <= 15)
            checkDriveMotors(speed);
        else if (checkHeading)
            checkDriveMotorsWithHeading(speed,distance);
        else
            checkDriveMotors(speed,distance);
        robot.setDriveMotorPower(0, 0, 0, 0);

    }
    protected void checkLiftMotors(){
        while(opModeIsActive()&&robot.isLiftMoving()){
            this.idle();
        }
        robot.setLiftMotorPower(0,0);
    }

    private void checkDriveMotors(double speed) {
        double z_Angle = 0.0;
        double local_speed = START_SPEED;

        while ((opModeIsActive()) && robot.isRobotMoving()) {
            //robot.logDriveEncoderPosition();
            //int[] motorTicks = robot.getDriveMotorEncoderPositions();
            //telemetry.addData("Positioning", ":%7d :%7d :%7d :%7d", motorTicks[0], motorTicks[1], motorTicks[2], motorTicks[3]);
            if (local_speed < speed)
                local_speed += SPEED_INCR;

            robot.setDriveMotorPower(local_speed,local_speed,local_speed,local_speed);

            telemetry.addData("Local Speed:", local_speed);
            telemetry.update();
            //Log.i(ElectroBot.LOGTAG, "Current Heading: " + z_Angle);

        }
    }

    private void checkDriveMotorsWithHeading(double speed) {
        double z_Angle = 0.0;
        double local_speed = START_SPEED;
        double error = 0;
        double steer = 0;

        while ((opModeIsActive()) && robot.isRobotMoving()) {
            //robot.logDriveEncoderPosition();
            //int[] motorTicks = robot.getDriveMotorEncoderPositions();
            //telemetry.addData("Positioning", ":%7d :%7d :%7d :%7d", motorTicks[0], motorTicks[1], motorTicks[2], motorTicks[3]);
            error = headingToUse - robot.getYaw();
            if (local_speed < speed)
                local_speed += SPEED_INCR;
            if (abs(error) > 1) {
                steer = robot.getSteer(-1 * error, 0.04);
                //Log.i(ElectroBot.LOGTAG, "Heading Error: " + error);
                //Log.i(ElectroBot.LOGTAG, "Heading Steer: " + steer);
                //Log.i(ElectroBot.LOGTAG, "LocalSpeed:" + local_speed);
                robot.setDriveMotorPower(local_speed - steer, local_speed + steer, local_speed - steer, local_speed + steer);
            } else
                robot.setDriveMotorPower(local_speed,local_speed,local_speed,local_speed);
            telemetry.addData("Local Speed:", local_speed);
            telemetry.update();
            //Log.i(ElectroBot.LOGTAG, "Current Heading: " + z_Angle);

        }
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
            if(encoderticks1/targetpos<=0.75){
                if (local_speed < speed)
                    local_speed += SPEED_INCR;

            }
            else{
                if(local_speed>=START_SPEED){
                    local_speed-=(2*SPEED_INCR);
                }
            }
            robot.setDriveMotorPower(local_speed, local_speed, local_speed, local_speed);
            //z_Angle = robot.getYaw();
            telemetry.addData("Local Speed:", local_speed);
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
            if((encoderticks1/targetpos) <0.75){
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
            telemetry.addData("Local Speed:", local_speed);
            telemetry.update();

        }
    }

    protected void moveLiftUpInAuto(int towerHeight) {
        robot.setLiftMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        switch (towerHeight) {
            case 0:
                robot.setLiftMotorEncoderPosition(-100);
                break;
            case 1:
                robot.setLiftMotorEncoderPosition(-400);
                break;
            default:
                robot.setLiftMotorEncoderPosition((-100 - (towerHeight * 300)));
                break;
        }
        robot.setLiftMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setLiftMotorPower(-0.75, -0.75);
        while (robot.isLiftMoving()) {
            idle();
        }
        robot.setLiftMotorPower(0, 0);
    }

    protected void moveLiftDownInAuto() {
        robot.setLiftMotorEncoderPosition(0);
        robot.setLiftMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.setLiftMotorPower(0.75, 0.75);
        while (robot.isLiftMoving()) {
            idle();
        }
        robot.setLiftMotorPower(0, 0);
    }

}

