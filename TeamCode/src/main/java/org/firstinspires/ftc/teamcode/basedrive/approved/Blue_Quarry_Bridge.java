package org.firstinspires.ftc.teamcode.basedrive.approved;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.basedrive.hwmap.ElectroBot;
import org.firstinspires.ftc.teamcode.basedrive.hwmap.SoftwareBot;

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

@Autonomous(name = "BLUE:Quarry:Bridge", group = "1Main_Auto")
//@Disabled
public class Blue_Quarry_Bridge extends LinearOpMode {

    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.3;
    static final double START_SPEED = 0.1;
    static final double SPEED_INCR = 0.075;
    static final double END_SPEED = 0.3;
    static final double END_SPEEDf = 0.3;
    static final double END_SPEEDb = -0.3;
    private static final double posPIDF = 5;

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
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        Log.i(ElectroBot.LOGTAG, "BQB: Initializing Robot: ");
        robot.init(hardwareMap);

        Log.i(ElectroBot.LOGTAG, "BQB: Setting velocity PIDF values");
        F = 32767.0 / 2700.0;
        //robot.setMotorPIDF("FL", 0.1 * F, 0.01 * F, 0.0, F);
        //robot.setMotorPIDF("BL", 0.1 * F, 0.01 * F, 0.0, F);
        //robot.setMotorPIDF("FR", 0.1 * F, 0.01 * F, 0.0, F);
        //robot.setMotorPIDF("BR", 0.1 * F, 0.01 * F, 0.0, F);

        Log.i(ElectroBot.LOGTAG, "BQB: Setting Position PIDF");
        robot.setPositionPIDF(posPIDF);

        // Send telemetry message to signify robot waiting;
        Log.i(ElectroBot.LOGTAG, "BQB: Initialized robot");
        telemetry.addData("Status", "Initialized Robot");    //
        telemetry.update();

        Log.i(ElectroBot.LOGTAG, "BQB: Calibrating Gyro");
        telemetry.addData("Status:", "Calbrating Gyro");
        telemetry.update();
        while (!robot.isGyroCalibrated()) {
            this.sleep(50);
            this.idle();
        }


        // Wait for the game to start (driver presses PLAY)
        Log.i(ElectroBot.LOGTAG, "BQB: Wait for start");
        telemetry.update();

        robot.resetDriveMotorEncoders();
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Press Play to begin");
        telemetry.update();
        waitForStart();

        //Get Bridge heading
        quarryHeading = robot.getYaw();
        Log.i(SoftwareBot.LOGTAG, "Starting Heading: " + quarryHeading);
        headingToUse = quarryHeading;

        ssPos1 = getFirstSkyStonePosition();

        telemetry.addData("Status: ", "Sampling Skytone");
        telemetry.update();

        switch (ssPos1) {
            case 1:
                getSkystoneFromPos1();

                Log.i(ElectroBot.LOGTAG, "BQB: This moves foundation into Building zone and turns 90 deg");
                telemetry.addData("Status: ", "Moving Waffle maker to building zone and turns 90 deg");
                telemetry.update();
                moveFoundation(1);

                getSkystoneFromPos4();
                break;
            case 2:
                getSkystoneFromPos2();

                Log.i(ElectroBot.LOGTAG, "BQB: This moves foundation into Building zone and turns 90 deg");
                telemetry.addData("Status: ", "Moving Waffle maker to building zone and turns 90 deg");
                telemetry.update();
                moveFoundation(2);

                getSkystoneFromPos5();
                break;
            case 3:
                getSkystoneFromPos3();

                Log.i(ElectroBot.LOGTAG, "BQB: This moves foundation into Building zone and turns 90 deg");
                telemetry.addData("Status: ", "Moving Waffle maker to building zone and turns 90 deg");
                telemetry.update();
                moveFoundation(3);

                getSkystoneFromPos6();
        }

        Log.i(ElectroBot.LOGTAG, "BQB: travelling towards bridge to park");
        telemetry.addData("Status: ", "travelling towards bridge to park");
        telemetry.update();
        parkUnderBridge();

        //robot.resetFlipServoPosition();
        //robot.setGripperServoPosition();

        telemetry.addData("Status: ", "Auto Complete");
        telemetry.update();
        if (isStopRequested()) {
            stop();
        }

        Log.i(ElectroBot.LOGTAG, "BQB: AUTO COMPLETE");
        robot.setDriveMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.setDriveMotorPower(0,0,0,0);

        this.sleep(30000);
    }


    protected int getFirstSkyStonePosition() {
        int pos = 3;

        //move towards Quarry
        Log.i(ElectroBot.LOGTAG, "BQB: moving towards quarry");
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 24.0, 0.5,true);

        this.sleep(250);

        Log.i(ElectroBot.LOGTAG, "BQB: Checking for skystone");
        double csRatio = robot.getColorSensorRatio();
        if (csRatio <= 0.7)
            pos = 1;
        else if (csRatio > 1.6)
            pos = 2;
        else
            pos = 3;
        Log.i(ElectroBot.LOGTAG, "BQB: Position of Skystone is: "+pos);

        moveRobot(ElectroBot.DIRECTIONS.BACKWARD,3,false);
        return pos;
    }

    protected void getSkystoneFromPos1() {

        Log.i(ElectroBot.LOGTAG, "BQB: Strafe right 14 in");
        moveRobot(ElectroBot.DIRECTIONS.LEFT, 18.5, 0.5, false);

        Log.i(ElectroBot.LOGTAG, "BQB: Turning counter clockwise ");
        robot.turnClockwise(36, TURN_SPEED);

        this.sleep(100);

        Log.i(ElectroBot.LOGTAG, "BQB: Travel forward while running intake motors");
        turnOnIntakeMotors();
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 11.0, 0.3, false);

        Log.i(ElectroBot.LOGTAG, "BQB: travel back to line up under the bridge");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 8.5, false);
        stopIntakeMotors();
        this.sleep(100);
        turnOnIntakeMotors();

        Log.i(ElectroBot.LOGTAG, "BQB: Turn 50 deg to line up the bot for travelling back.");
        robot.turnClockwise(48.5, TURN_SPEED);
        stopIntakeMotors();

        this.sleep(200);

        bridgeHeading = robot.getYaw();
        headingToUse = bridgeHeading;
        Log.i(ElectroBot.LOGTAG, "Bridge Heading: " + headingToUse);

        Log.i(ElectroBot.LOGTAG, "BQB: Travel back under bridge for the second skystone");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 58, true);
    }

    protected void getSkystoneFromPos2() {
        Log.i(ElectroBot.LOGTAG, "BQB: Travel back towards bridge");
        moveRobot(ElectroBot.DIRECTIONS.LEFT, 11, false);

        Log.i(ElectroBot.LOGTAG, "BQB: Turn 90 deg");
        robot.turnClockwise(85.5, TURN_SPEED);
        this.sleep(300);

        bridgeHeading = robot.getYaw();
        headingToUse = bridgeHeading;
        Log.i(ElectroBot.LOGTAG, "Bridge Heading: " + headingToUse);


        Log.i(ElectroBot.LOGTAG, "BQB: Strafe right and push the first block out");
        moveRobot(ElectroBot.DIRECTIONS.LEFT, 17, false);

        Log.i(ElectroBot.LOGTAG, "BQB: Travel forward while running intake motors");
        turnOnIntakeMotors();
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 6.0, false);

        Log.i(ElectroBot.LOGTAG, "BQB: Strafe left to line up under the bridge and stop intake motors");
        moveRobot(ElectroBot.DIRECTIONS.RIGHT, 16.5, false);
        stopIntakeMotors();

        this.sleep(100);
        //Log.i(ElectroBot.LOGTAG, "Heading after strafing: " + robot.getYaw());
        //Log.i(ElectroBot.LOGTAG, "Turn 2 deg to straighten up");
        //robot.turnClockwise(2, TURN_SPEED);

        Log.i(ElectroBot.LOGTAG, "BQB: Travel back to cross the bridge and drop the stone");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 70, true);
    }

    protected void getSkystoneFromPos3() {

        Log.i(ElectroBot.LOGTAG, "BQB: Turn 40 deg");
        robot.turnClockwise(33, TURN_SPEED);

        this.sleep(200);

        Log.i(ElectroBot.LOGTAG, "BQB: Travel forward while running intake motors");
        turnOnIntakeMotors();
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 15.0,0.3, false);

        Log.i(ElectroBot.LOGTAG, "BQB: Travel back to line up under the bridge");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 13, false);
        stopIntakeMotors();
        this.sleep(100);
        turnOnIntakeMotors();

        Log.i(ElectroBot.LOGTAG, "BQB: Turn 50 deg to prepare to travel back");
        robot.turnClockwise(54.5, TURN_SPEED);
        stopIntakeMotors();

        this.sleep(200);

        bridgeHeading = robot.getYaw();
        headingToUse = bridgeHeading;
        Log.i(ElectroBot.LOGTAG, "Bridge Heading: " + headingToUse);

        Log.i(ElectroBot.LOGTAG, "BQB: Travel back to cross the bridge and drop the stone");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 74.5,0.7, true);
    }

    protected void getSkystoneFromPos4() {
        Log.i(ElectroBot.LOGTAG, "BQB: travel back to quarry side");
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 75, false);
        //Log.i(ElectroBot.LOGTAG, "Bridge Heading after travel back to quarry: " + robot.getYaw());

        Log.i(ElectroBot.LOGTAG, "Intake second skystone from Pos 4");
        intakeSecondSkystone(1);
        //robot.setWGToDownPosition();
        robot.setGripperServoPosition();
        robot.setWGToMidPosition();
        Log.i(ElectroBot.LOGTAG, "BQB: Travel back to cross the bridge");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 79,  true);
        robot.resetGripperServoPosition();
        robot.setFlipServoPosition();
        robot.setIntakeMotorPower(-0.45, -0.45);
        robot.setWheelServos(.9);
        this.sleep(500);
        stopIntakeMotors();
        robot.resetGripperServoPosition();

    }

    protected void getSkystoneFromPos5() {
        Log.i(ElectroBot.LOGTAG, "BQB: travel back to quarry side");
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 84, false);

        //Log.i(ElectroBot.LOGTAG, "Bridge Heading after travel back to quarry: " + robot.getYaw());

        Log.i(ElectroBot.LOGTAG, "Intake second skystone from Pos 5");
        intakeSecondSkystone(2);
        robot.setGripperServoPosition();
        robot.setWGToMidPosition();
        Log.i(ElectroBot.LOGTAG, "BQB: Travel back to cross the bridge");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 88,  true);
        robot.resetGripperServoPosition();
        robot.setFlipServoPosition();
        robot.setIntakeMotorPower(-0.45, -0.45);
        robot.setWheelServos(.9);
        this.sleep(700);
        stopIntakeMotors();
        robot.resetGripperServoPosition();

    }

    protected void getSkystoneFromPos6() {
        Log.i(ElectroBot.LOGTAG, "BQB: travel back to quarry side");
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 91.5, true);

        //Log.i(ElectroBot.LOGTAG, "Bridge Heading after travel back to quarry: " + robot.getYaw());
        moveRobot(ElectroBot.DIRECTIONS.LEFT,1,false);
        Log.i(ElectroBot.LOGTAG, "Intake second skystone from Pos 5");
        intakeSecondSkystone(3);
        robot.setGripperServoPosition();
        robot.setWGToMidPosition();
        Log.i(ElectroBot.LOGTAG, "BQB: Travel back to cross the bridge");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 94,  true);
        robot.resetGripperServoPosition();
        robot.setFlipServoPosition();
        robot.setIntakeMotorPower(-0.45, -0.45);
        robot.setWheelServos(.9);
        this.sleep(500);
        stopIntakeMotors();
        robot.resetGripperServoPosition();

    }

    protected void intakeFirstSkystone(){
        //moveRobot(ElectroBot.DIRECTIONS.BACKWARD,3,false);
        turnOnIntakeMotors();
        robot.turnBiasClockwise(43,TURN_SPEED,TURN_SPEED/5);

        stopIntakeMotors();

        robot.turnClockwise(43,TURN_SPEED);
    }

    protected void intakeSecondSkystone(double position){

        if(position==2){
            moveRobot(ElectroBot.DIRECTIONS.LEFT,17.5,false);
        }
        else if(position==3){
            moveRobot(ElectroBot.DIRECTIONS.LEFT,14.5,false);
        }
        else{
            moveRobot(ElectroBot.DIRECTIONS.LEFT,17.5,false);
        }

        turnOnIntakeMotors();
        moveRobot(ElectroBot.DIRECTIONS.FORWARD,6,false);
        if(position==3){
            moveRobot(ElectroBot.DIRECTIONS.RIGHT,13,false);

        }
        else{
            moveRobot(ElectroBot.DIRECTIONS.RIGHT,16.5,false);

        }
        stopIntakeMotors();
        robot.setGripperServoPosition();
    }


    protected void parkUnderBridge() {
        this.sleep(100);
        //Travel towards team wall
        // Log.i(ElectroBot.LOGTAG, "BQB: Strafe left towards team wall");
        //moveRobot(ElectroBot.DIRECTIONS.RIGHT, 2);

        Log.i(ElectroBot.LOGTAG, "BQB: Travel towards bridge to park under");
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 34.0, .99, true);
    }


    /*** NOT USED for MEET3 */
    protected void dropSkyStone() {
        /****
         * 1. Open WG Servo
         * 2. Turn on Intake motors
         * 3. Move forward for few inches.
         */
        robot.setWGToDownPosition();
        this.sleep(400);
        robot.setFlipServoPosition();
        robot.setIntakeMotorPower(-0.45, -0.45);
        robot.setWheelServos(.9);
        this.sleep(500);
        robot.setWGToMidPosition();
        this.sleep(500);

    }

    /*
     *  Runs all the steps needed to intake a stone
     * 1. Sets WG servos to UP position (These act as a backstop for the stone)
     * 2. Sets Flip servos to CLOSED postion
     * 3. Turns on Intake motors
     * 4. Turns on the servos to pull stone to the back so that it's ready to be placed
     */
    protected void turnOnIntakeMotors() {
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
    protected void stopIntakeMotors() {
        robot.setWGToUpPosition();
        robot.setIntakeMotorPower(0, 0);
        robot.setWheelServos(0.0);
        robot.resetFlipServoPosition();
    }

    /*
     * Moves the foundation and Places the first stone on the foundation
     */
    private void moveFoundation(int position) {
        //this.sleep(5000);
        telemetry.addData("Status: ", "At WM, turning");
        telemetry.update();
        //Log.i(ElectroBot.LOGTAG, "BQB: Bridge Heading after travel: " + robot.getYaw());
        turnOnIntakeMotors();
        Log.i(ElectroBot.LOGTAG, "BQB: At Waffle maker. turn Counter Clockwise so that back of robot is  perpendicular to foundation");
        robot.turnClockwise(87, TURN_SPEED);
        stopIntakeMotors();
        Log.i(ElectroBot.LOGTAG, "BQB: Grip the skystone");
        robot.setGripperServoPosition();

        //move towards WaffleMaker
        Log.i(ElectroBot.LOGTAG, "BQB: Travel back at slower speed towards foundation");
        if (position == 3) {
            moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 10, 0.3, false);

        }
        else if(position==1){
            moveRobot(ElectroBot.DIRECTIONS.BACKWARD,10,0.3,false);
        }
        else {
            moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 8, 0.3, false);}



        Log.i(ElectroBot.LOGTAG, "BQB: At the Waffle maker, dropping anchors, open flip servos, start moving the arm out");
        robot.setWGToDownPosition();  //putting servo down
        robot.resetFlipServoPosition(); //Opening up flip servos
        this.sleep(250);
        robot.setArmServos(-0.67); //extending ARM out

        //pulling WaffleMaker
        Log.i(ElectroBot.LOGTAG, "BQB: Travel back at slower speed towards team wall");
        if (position == 3) {
            moveRobot(ElectroBot.DIRECTIONS.FORWARD, 8, 0.3, false);

        } else {
            moveRobot(ElectroBot.DIRECTIONS.FORWARD, 7, 0.3, false);
        }

        Log.i(ElectroBot.LOGTAG, "BQB: Stop moving the ARM");
        robot.setArmServos(0.0);

        Log.i(ElectroBot.LOGTAG, "BQB: Turn waffle grabber 90 deg");
        robot.turnBiasCounterClockwise(81, 0.03, 0.7);
        //this.sleep(10);

        //Pull up anchors from foundation
        Log.i(ElectroBot.LOGTAG, "BQB: Pulling up anchors from Waffle maker");
        robot.setWGToUpPosition();

        Log.i(ElectroBot.LOGTAG, "BQB: moving away from foundation to avoid dragging it");
        moveRobot(ElectroBot.DIRECTIONS.FORWARD, 2, false);

        Log.i(ElectroBot.LOGTAG, "BQB: move robot right, drop skystone and then retract arm");
        moveRobot(ElectroBot.DIRECTIONS.LEFT, 3.5, false);
        Log.i(ElectroBot.LOGTAG, "Strafe Left 3.5");
        moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 2, false);
        Log.i(ElectroBot.LOGTAG, "Travelling back 2");
        robot.resetGripperServoPosition();
        Log.i(ElectroBot.LOGTAG, "Dropped skystone");
        robot.resetFlipServoPosition();
        Log.i(ElectroBot.LOGTAG, "Start pulling ARM back in");
        robot.holdCapstone();

        robot.setArmServos(0.65);
        Log.i(ElectroBot.LOGTAG, "Sleeping while pulling arm in");
        this.sleep(800);
        Log.i(ElectroBot.LOGTAG, "Stopping pulling arm in");
        robot.setArmServos(0.0);


        Log.i(ElectroBot.LOGTAG, "BQB: Move right to be in the middle of waffle grabber");
        if(position==1){
            moveRobot(ElectroBot.DIRECTIONS.LEFT,10,false);
        }
        else if(position==2){
            moveRobot(ElectroBot.DIRECTIONS.LEFT,10,false);
        }
        else {
            moveRobot(ElectroBot.DIRECTIONS.LEFT, 10, false);
        }

        Log.i(ElectroBot.LOGTAG, "BQB: Pushing Waffle Grabber towards wall");
        if (position == 3)
            moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 12.5, false);
        else
            moveRobot(ElectroBot.DIRECTIONS.BACKWARD, 12, false);
        //bridgeHeading = robot.getYaw();
        //headingToUse = bridgeHeading;
        //Log.i(ElectroBot.LOGTAG, "Bridge Heading: " + headingToUse);
        //if (position == 3)
        //    moveRobot(ElectroBot.DIRECTIONS.LEFT, 2, false);

        robot.setArmServos(0.0);

        Log.i(ElectroBot.LOGTAG, "BQB: Stop and reset encoders before travelling back");
        robot.setDriveMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDriveMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //Log.i(ElectroBot.LOGTAG, "Bridge Heading before travel back to quarry: " + robot.getYaw());
    }

    protected void moveLiftUp(){
        robot.moveLift(5.5,.85);
    }
    protected void moveLiftDown(){
        robot.moveLift(-5.5,-0.85);
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

            telemetry.addLine("Local Speedf:" +local_speedf);
            telemetry.addLine("Local Speedb:" +local_speedb);
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
            telemetry.addData("Local Speed:", local_speed);
            telemetry.update();
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
    protected void checkLiftMotors(){
        while(opModeIsActive()&&robot.isLiftMoving()){
            this.idle();
        }
        robot.setLiftMotorPower(0,0);
    }

}

