package org.firstinspires.ftc.teamcode.basedrive.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.basedrive.hwmap.ElectroBot;

import java.util.concurrent.TimeUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Electro TeleOp_DELIVERY", group = "Electrobot")
//@Disabled
public class Electro_TeleOp_DELIVERY extends LinearOpMode {

    // Declare OpMode members.
    private ElectroBot robot = new ElectroBot();   // Initializing our robot
    private ElapsedTime runtime = new ElapsedTime();

    private int[] liftmotorTicks;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
        // set boolean logic for block grabbber
        boolean isGripperSet = false;
        boolean isTurnerSet = false;
        ElapsedTime turnerTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime gripperTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        // set button debounce variables
        boolean rtBmpr1Lock = false;
        boolean ltBmpr1Lock = false;
        ElapsedTime rtBmpr1Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime ltBmpr1Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime btnA1Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime btnX1Timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        boolean dPadLftLock = false;
        boolean dPadRghtLock = false;
        ElapsedTime dPadLftTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime dPadRghtTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        double lmPowerCoeff = 0;
        double rmPowerCoeff = 0;

        double right2y;
        // Send telemetry message to signify robot waiting;
        Log.i(ElectroBot.LOGTAG, "Initialized robot");

        telemetry.addData("Status", "Initialized Robot");    //
        telemetry.update();

        //Log Servo positions
        double[] sPos = robot.getServoPositions();

        robot.setWGToUpPosition();
        robot.resetFlipServoPosition();
        robot.holdCapstone();
        this.sleep(100);

        robot.resetLiftMotorEncoders();
        liftmotorTicks = robot.getLiftMotorEncoderPositions();
        //(ElectroBot.LOGTAG, "Starting lift positions L:R : " + liftmotorTicks[0] + " ; " + liftmotorTicks[1]);

        telemetry.addData("Status", "Press Play to start:");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        gripperTimer.reset();
        turnerTimer.reset();
        rtBmpr1Timer.reset();
        ltBmpr1Timer.reset();


        //robot.getLiftMotorEncoderPositions();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double flPower;
            double frPower;
            double blPower;
            double brPower;
            double llmPower = 0;
            double rlmPower = 0;


            double maxPower = 0.6;
            double minPower = -0.6;


            /** DRIVE MOTOR SECTION **/
            // passing from controller to motor
            double leftsticky1 = -1 * gamepad1.left_stick_y;
            double r = Math.hypot(gamepad1.left_stick_x, leftsticky1);
            double robotAngle = Math.atan2(leftsticky1, gamepad1.left_stick_x) - (Math.PI / 4);
            double rightX = gamepad1.right_stick_x;

            double v1 = r * Math.cos(robotAngle) + (rightX);
            double v2 = r * Math.sin(robotAngle) - (rightX);
            double v3 = r * Math.sin(robotAngle) + (rightX);
            double v4 = r * Math.cos(robotAngle) - (rightX);

            if (gamepad1.left_stick_button) {
                maxPower = 0.95;
                minPower = -0.95;
            } else if (gamepad1.left_trigger != 0) {
                maxPower = 0.25;
                minPower = -0.25;
            } else if (gamepad1.right_trigger != 0) {
                maxPower = 0.35;
                minPower = -0.35;
            } else {
                maxPower = 0.65;
                minPower = -0.65;
            }


            flPower = Range.clip(v1 * 1.4, minPower, maxPower);
            frPower = Range.clip(v2 *  1.4, minPower, maxPower);
            blPower = Range.clip(v3 * 1.4, minPower, maxPower);
            brPower = Range.clip(v4 * 1.4, minPower, maxPower);

            // Send calculated power to wheels
            robot.setDriveMotorPower(flPower, frPower, blPower, brPower);

            /**     END DRIVE MOTOR SECTION **/

            /**         LIFT MOTOR SECTION  **/


            //Log gamepad2 values for lift motors
            /*Log.i(ElectroBot.LOGTAG, "Gamepad 2 Y Value: " + right2y);
            Log.i(ElectroBot.LOGTAG, "LLM Power: " + llmPower);
            Log.i(ElectroBot.LOGTAG, "RLM Power: " + rlmPower);

            Log.i(ElectroBot.LOGTAG, "current lift positions: " + liftmotorTicks[0] + " ; " + liftmotorTicks[1]);
            */
            //Calculate Power for lift motors. Need to adjust these for friction.
            liftmotorTicks = robot.getLiftMotorEncoderPositions();
            if (liftmotorTicks[0] == 0)
                liftmotorTicks[0] = 1;
            if (liftmotorTicks[1] == 0)
                liftmotorTicks[1] = 1;




            lmPowerCoeff = Math.abs((double) liftmotorTicks[1] / (double) liftmotorTicks[0]);
            rmPowerCoeff = Math.abs((double) liftmotorTicks[0] / (double) liftmotorTicks[1]);

            if (gamepad2.right_stick_y != 0) {
                right2y = (Math.round(gamepad2.right_stick_y * 100)) * 0.01;
                if (right2y < 0) {

                    llmPower = Range.clip(right2y + (lmPowerCoeff / -2), -0.975, 0.85);
                    rlmPower = Range.clip(right2y + (rmPowerCoeff / -2), -0.975, 0.85);
                } else if (right2y > 0) {
                    llmPower = Range.clip(right2y + (lmPowerCoeff / 2), -0.975, 0.85);
                    rlmPower = Range.clip(right2y + (rmPowerCoeff / 2), -0.975, 0.85);
                } else {
                    llmPower = 0;
                    rlmPower = 0;
                }
                //Log.i(ElectroBot.LOGTAG, "current lift positions L:R : " + liftmotorTicks[0] + " ; " + liftmotorTicks[1]);
                //Log.i(ElectroBot.LOGTAG, "LLM coef: " + lmPowerCoeff + "  RLM coef: " + rmPowerCoeff);
                //Log.i(ElectroBot.LOGTAG, "Powers LLM: " + llmPower + " RLM: " + rlmPower + "Right2Y: " + right2y);
            } else {
                llmPower = 0;
                rlmPower = 0;
            }

            /*
             * Checks to make sure that the lift motors are not going down too much.
             */
            if (gamepad2.right_stick_y > 0) {
                if (liftmotorTicks[0] >= 0)
                    llmPower = 0;
                if (liftmotorTicks[1] >= 0)
                    rlmPower = 0;
            }

            //Send power to lift motors
            robot.setLiftMotorPower(llmPower, rlmPower);

            /**     END LIFT MOTOR SECTION          **/

            /**     ARM EXTENDER SERVO SECTION      **/
            robot.setArmServos(Range.clip(gamepad2.left_stick_y, -.8, .8));
            //Log.v(ElectroBot.LOGTAG, "GP2 LSY: " + gamepad2.left_stick_y);

            /*set gripper control to GamePad2--Right bumper button.
                This is used for the handle to move stones to foundation and create a tower.
                Pressing "A" once pushes gripper handles inward to hold onto the stone, pushing
                it again will move these handles outward and thus dropping the stone on to surface
                Using a boolean to check current state of servo and a timer to handle
                de-bouncing issues
             */
            if (gamepad2.right_bumper) {
                if (!isGripperSet && gripperTimer.time() > 250) {
                    //Log.i(ElectroBot.LOGTAG, "Setting gripper position: " + gripperTimer.time());
                    robot.setGripperServoPosition();
                    isGripperSet = true;
                    gripperTimer.reset();
                }
                if (isGripperSet && gripperTimer.time() > 250) {
                    //Log.i(ElectroBot.LOGTAG, "Resetting gripper position: " + gripperTimer.time());
                    robot.resetGripperServoPosition();
                    isGripperSet = false;
                    gripperTimer.reset();
                }
            }
            /**     TAPE MEASURE EXTENDER SERVO SECTION      **/

            if(gamepad2.left_trigger>=0.01){
                robot.setTMMPower(0.3);
            }

            else if(gamepad2.right_trigger>=0.01){
                robot.setTMMPower(-0.7);
            }
            else
                robot.setTMMPower(0);

            /*set turner control to GamePad2--Left Bumper. Used when placing stones on foundation.
                Pressing this button turns mechanism 90 degrees. Using a boolean to monitor
                current state and a timer for handling de-bouncing issues
             */

            if (gamepad2.left_bumper) {
                if (!isTurnerSet && turnerTimer.time() > 200) {
                    //Log.i(ElectroBot.LOGTAG, "Setting Turner position: " + turnerTimer.time());
                    robot.setTurnerServoPosition();
                    isTurnerSet = true;
                    turnerTimer.reset();
                }
                if (isTurnerSet && turnerTimer.time() > 200) {
                    //Log.i(ElectroBot.LOGTAG, "Resetting Turner position: " + turnerTimer.time());
                    robot.resetTurnerServoPosition();
                    isTurnerSet = false;
                    turnerTimer.reset();
                }
            }

            /*GamePad1-- Right bumper and left bumpers are used to control different powers and controls intake wheels.
              Using Boolean "LOCK" variables and timers for controlling de-bouncing issues we have seen.
              Also, using the lock variable to control accidental press of both bumpers.
             */
            if (gamepad1.right_bumper && !ltBmpr1Lock) {
                //robot.setWGToUpPosition();

                robot.setIntakeMotorPower(-0.5, -0.5);
                robot.setWheelServos(.8);
                rtBmpr1Lock = true;
                rtBmpr1Timer.reset();
                robot.setFlipServoPosition();
                //Log.i(ElectroBot.LOGTAG, "Turning on intake: " + rtBmpr1Lock);
            } else if (!gamepad1.right_bumper && rtBmpr1Timer.time(TimeUnit.MILLISECONDS) >= 200 && rtBmpr1Lock) {
                rtBmpr1Lock = false;
                rtBmpr1Timer.reset();
                robot.resetFlipServoPosition();
                //Log.i(ElectroBot.LOGTAG, "Released Right Bumper: " + rtBmpr1Lock);
            }

            if (gamepad1.left_bumper && !rtBmpr1Lock) {
                robot.setIntakeMotorPower(0.5, 0.5);
                robot.setWheelServos(-0.9);
                ltBmpr1Lock = true;
                ltBmpr1Timer.reset();
                robot.setFlipServoPosition();
                //Log.i(ElectroBot.LOGTAG, "pushing out motors: " + rtBmpr1Lock);
            } else if (!gamepad1.left_bumper && ltBmpr1Timer.time(TimeUnit.MILLISECONDS) >= 200 && ltBmpr1Lock) {
                ltBmpr1Lock = false;
                ltBmpr1Timer.reset();
                robot.resetFlipServoPosition();
                //Log.i(ElectroBot.LOGTAG, "Released left Bumper: " + rtBmpr1Lock);
            }
            if (!ltBmpr1Lock && !rtBmpr1Lock) {
                robot.setIntakeMotorPower(0.0, 0.0);
                robot.setWheelServos(0.0);
            }

            /* Using Gamepad1--A button to control servos for foundation grabber hooks.
             */
            if (gamepad2.y) {
                //Log.i(ElectroBot.LOGTAG, "Dropping Capstone");
                robot.dropCapstone();


            }
            if (gamepad2.b) {
                //Log.i(ElectroBot.LOGTAG, "Picking Capstone Back Up");
                robot.holdCapstone();

            }
            if (gamepad2.a && btnA1Timer.time() > 200) {
                //Log.i(ElectroBot.LOGTAG, "Setting WG  position: " + btnA1Timer.time());
                robot.setWGToUpPosition();
                btnA1Timer.reset();
            }
            if (gamepad2.x && btnX1Timer.time() > 200) {
                //Log.i(ElectroBot.LOGTAG, "Resetting gripper position: " + btnX1Timer.time());
                robot.setWGToDownPosition();
                btnX1Timer.reset();
            }

            if (gamepad1.dpad_left) {
                if (dPadLftTimer.time(TimeUnit.MILLISECONDS) > 200) {
                    dPadLftTimer.reset();
                    //run motors
                    int[] dmEncoderticks = robot.getDriveMotorEncoderPositions();
                    int frposition = dmEncoderticks[1];
                    int newPosition = frposition + (int) (0.5 * robot.getCountsPerInch());
                    while (frposition <= newPosition) {
                        robot.setDriveMotorPower(-0.3, 0.3, 0.3, -0.3);
                        dmEncoderticks = robot.getDriveMotorEncoderPositions();
                        frposition = dmEncoderticks[1];
                    }
                    robot.setDriveMotorPower(0, 0, 0, 0);

                }
            }

            if (gamepad1.dpad_right) {
                if (dPadRghtTimer.time(TimeUnit.MILLISECONDS) > 200) {
                    dPadRghtTimer.reset();
                    //run motors
                    int[] dmEncoderticks = robot.getDriveMotorEncoderPositions();
                    int flposition = dmEncoderticks[0];
                    int newPosition = flposition + (int) (0.5 * robot.getCountsPerInch());
                    while (flposition <= newPosition) {
                        robot.setDriveMotorPower(0.3, -0.3, -0.3, 0.3);
                        dmEncoderticks = robot.getDriveMotorEncoderPositions();
                        flposition = dmEncoderticks[0];
                    }
                    robot.setDriveMotorPower(0, 0, 0, 0);
                }
            }

            /*if(gamepad2.dpad_down) {
                liftmotorTicks = robot.getLiftMotorEncoderPositions();
                robot.setLiftMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                robot.setLiftMotorEncoderPosition(0);
                robot.setLiftMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                robot.setLiftMotorPower(0.75, 0.75);
                while (robot.isLiftMoving()) {
                    idle();
                }
                robot.setLiftMotorPower(0, 0);
                robot.setLiftMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if(gamepad2.dpad_up) {
                try {
                    robot.setLiftMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.setLiftMotorEncoderPosition(liftmotorTicks[0], liftmotorTicks[1]);
                    robot.setLiftMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    robot.setLiftMotorPower(-0.75, -0.75);
                    while (robot.isLiftMoving()) {
                        idle();
                    }
                    robot.setLiftMotorPower(0, 0);
                    robot.setLiftMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                } catch (Exception x) {
                    Log.i(ElectroBot.LOGTAG, "Exception is moving lift up " + x.getMessage() );
                }
            }*/
        }
    }
}
