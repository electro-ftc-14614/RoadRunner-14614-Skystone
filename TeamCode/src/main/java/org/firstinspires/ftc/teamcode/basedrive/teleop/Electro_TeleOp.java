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

@TeleOp(name = "Electro TeleOp", group = "Electrobot")
//@Disabled
public class Electro_TeleOp extends LinearOpMode
{

    // Declare OpMode members.
    private ElectroBot robot = new ElectroBot();   // Initializing our robot
    private ElapsedTime runtime = new ElapsedTime();

    private int[] liftmotorTicks;

    @Override
    public void runOpMode()
    {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
        // set boolean logic for block grabbber
       /* boolean isGripperSet = false;
        boolean isTurnerSet = false;
        ElapsedTime turnerTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime gripperTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
*/

        // Send telemetry message to signify robot waiting;
        Log.i(ElectroBot.LOGTAG, "Initialized robot");

        telemetry.addData("Status", "Initialized Robot");    //
        telemetry.update();


        telemetry.addData("Status", "Press Play to start:");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //robot.getLiftMotorEncoderPositions();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested())
        {

            // Setup a variable for each drive wheel to save power level for telemetry
            double flPower;
            double frPower;
            double blPower;
            double brPower;


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

            if (gamepad1.left_stick_button)
            {
                maxPower = 0.95;
                minPower = -0.95;
            } else if (gamepad1.left_trigger != 0)
            {
                maxPower = 0.25;
                minPower = -0.25;
            } else if (gamepad1.right_trigger != 0)
            {
                maxPower = 0.35;
                minPower = -0.35;
            } else
            {
                maxPower = 0.65;
                minPower = -0.65;
            }


            flPower = Range.clip(v1 * 1.4, minPower, maxPower);
            frPower = Range.clip(v2 * 1.4, minPower, maxPower);
            blPower = Range.clip(v3 * 1.4, minPower, maxPower);
            brPower = Range.clip(v4 * 1.4, minPower, maxPower);

            // Send calculated power to wheels
            robot.setDriveMotorPower(flPower, frPower, blPower, brPower);

            /**     END DRIVE MOTOR SECTION **/

/*
            if (gamepad2.left_bumper)
            {
                if (!isTurnerSet && turnerTimer.time() > 200)
                {
                    //Log.i(ElectroBot.LOGTAG, "Setting Turner position: " + turnerTimer.time());
                    robot.setTurnerServoPosition();
                    isTurnerSet = true;
                    turnerTimer.reset();
                }
                if (isTurnerSet && turnerTimer.time() > 200)
                {
                    //Log.i(ElectroBot.LOGTAG, "Resetting Turner position: " + turnerTimer.time());
                    robot.resetTurnerServoPosition();
                    isTurnerSet = false;
                    turnerTimer.reset();
                }*/
            }
        }
    }



            /*GamePad1-- Right bumper and left bumpers are used to control different powers and controls intake wheels.
              Using Boolean "LOCK" variables and timers for controlling de-bouncing issues we have seen*/