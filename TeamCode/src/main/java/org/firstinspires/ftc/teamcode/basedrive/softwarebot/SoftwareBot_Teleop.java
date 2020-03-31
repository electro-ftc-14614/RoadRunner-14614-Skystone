package org.firstinspires.ftc.teamcode.basedrive.softwarebot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.basedrive.hwmap.ElectroBot;
import org.firstinspires.ftc.teamcode.basedrive.hwmap.SoftwareBot;


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

@TeleOp(name = "Software_TeleOP", group = "Softwarebot")
@Disabled
public class SoftwareBot_Teleop extends LinearOpMode {

    // Declare OpMode members.
    private ElectroBot robot = new ElectroBot();   // Initializing our robot
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        float angle;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
        float[] hsvValues = {0F, 0F, 0F};

        // Send telemetry message to signify robot waiting;
        Log.i(SoftwareBot.LOGTAG, "Initialized robot");
        telemetry.addData("Status", "Initialized Robot");    //
        telemetry.update();

        telemetry.addData("Status", "Press Play to start:");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Heading", robot.getHeading());
            telemetry.addData("Pitch", robot.getPitch());
            telemetry.addData("Roll", robot.getRoll());
            //telemetry.addData("Steer",robot.getSteer(10,10));
            telemetry.update();
        }
    }


}
