package org.firstinspires.ftc.teamcode.basedrive.testing;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.basedrive.hwmap.ElectroBot;

@TeleOp(name = "MaxVelocityTest", group = "Electrobot")
@Disabled
public class MaxVelocityTest extends LinearOpMode {

    private ElectroBot robot = new ElectroBot();   // Initializing our robot


    @Override
    public void runOpMode() {
        double[] maxVel = new double[]{0.0, 0.0, 0.0, 0.0};
        double[] curVel = new double[4];
        double flPower;
        double frPower;
        double blPower;
        double brPower;

        telemetry.addData("Status", "Initializing");
        telemetry.update();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Press Play to start:");    //
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double leftsticky1 = -1 * gamepad1.left_stick_y;
            double r = Math.hypot(gamepad1.left_stick_x, leftsticky1);
            double robotAngle = Math.atan2(leftsticky1, gamepad1.left_stick_x) - (Math.PI / 4);
            double rightX = -1 * gamepad1.right_stick_x;

            double v1 = r * Math.cos(robotAngle) + (-1 * rightX);
            double v2 = r * Math.sin(robotAngle) - (-1 * rightX);
            double v3 = r * Math.sin(robotAngle) + (-1 * rightX);
            double v4 = r * Math.cos(robotAngle) - (-1 * rightX);

            flPower = Range.clip(v1 * 1.4, -1, 1);
            frPower = Range.clip(v2 * 1.4, -1, 1);
            blPower = Range.clip(v3 * 1.4, -1, 1);
            brPower = Range.clip(v4 * 1.4, -1, 1);

            robot.setDriveMotorPower(flPower, frPower, blPower, brPower);

            curVel = robot.getCurVelocity();
            if (maxVel[0] > curVel[0])
                maxVel[0] = curVel[0];
            if (maxVel[1] > curVel[1])
                maxVel[1] = curVel[1];
            if (maxVel[2] > curVel[2])
                maxVel[2] = curVel[2];
            if (maxVel[3] > curVel[3])
                maxVel[3] = curVel[3];

            Log.i(ElectroBot.LOGTAG, "Current Velocities:" + curVel[0] + ";" + curVel[1] + ":" + curVel[2] + ":" + curVel[3]);
            Log.i(ElectroBot.LOGTAG, "Max Velocities:" + maxVel[0] + ";" + maxVel[1] + ":" + maxVel[2] + ":" + maxVel[3]);


            telemetry.addData("CurVel", curVel[0] + ";" + curVel[1] + ":" + curVel[2] + ":" + curVel[3]);
            telemetry.addData("maxVel", maxVel[0] + ";" + maxVel[1] + ":" + maxVel[2] + ":" + maxVel[3]);
            telemetry.update();


        }
    }
}
