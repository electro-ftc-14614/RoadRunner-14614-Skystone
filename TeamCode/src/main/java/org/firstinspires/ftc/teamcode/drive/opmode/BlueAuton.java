package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ElectroBot;
//import org.firstinspires.ftc.teamcode.drive.ElectroBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class BlueAuton extends LinearOpMode {
    private ElectroBot robot = new ElectroBot();  // Initializing our robot
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //drive.setPoseEstimate(new Pose2d(-54,-32,90));
        waitForStart();

        if (isStopRequested()) return;
        //drive.setPoseEstimate(new Pose2d(-54,-32,90));
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())

                .splineTo(new Pose2d(0,30,Math.toRadians(0))) //sample
                //turn on intake motors
                .addSpatialMarker(new Vector2d(0, 20), () -> {
                    robot.turnOnIntakeMotors();
                })

                .splineTo(new Pose2d(10,39,Math.toRadians(45))) //intake block
                //turn off intake motors
                .addSpatialMarker(new Vector2d(10, 39), () -> {
                    robot.stopIntakeMotors();
                })

                .splineTo(new Pose2d(-35,34,Math.toRadians(90))) //midpoint
                .splineTo(new Pose2d(-85,55,Math.toRadians(180))) //start foundation pull
                //drop wg and turn on intake motors
                .addSpatialMarker(new Vector2d(0, 20), () -> {
                    robot.turnOnIntakeMotors();
                    robot.setWGToDownPosition();
                })
                .splineTo(new Pose2d(-85,15,Math.toRadians(90))) //end foundation pull
                //up wg
                .addSpatialMarker(new Vector2d(0, 20), () -> {
                    robot.setWGToUpPosition();
                })
                //.splineTo(new Pose2d(-35,34,Math.toRadians(90))) //midpoint
                .splineToLinearHeading(new Pose2d(-35,34,Math.toRadians(90)),90.0)
                //turn on intake
                .addSpatialMarker(new Vector2d(0, 20), () -> {
                    robot.turnOnIntakeMotors();
                })
                .splineTo(new Pose2d(26,39,Math.toRadians(45))) //intake block
                //turn off intake
                .addSpatialMarker(new Vector2d(0, 20), () -> {
                    robot.stopIntakeMotors();
                })
                .splineTo(new Pose2d(-85,15,Math.toRadians(90))) //drop stone
                //wg to mid pos, intake on, wait intake off
                .addSpatialMarker(new Vector2d(0, 20), () -> {
                    robot.setWGToMidPosition();
                    robot.turnOnIntakeMotors();
                    this.sleep(200);
                    robot.stopIntakeMotors();
                    robot.setWGToUpPosition();
                })
                .splineTo(new Pose2d(-35,34,Math.toRadians(90))) //midpoint
                //intake on
                .addSpatialMarker(new Vector2d(0, 20), () -> {
                    robot.turnOnIntakeMotors();
                })
                .splineTo(new Pose2d(2,39,Math.toRadians(45))) //intake third stone
                //intake off
                .addSpatialMarker(new Vector2d(0, 20), () -> {
                    robot.stopIntakeMotors();
                })
                .splineTo(new Pose2d(-35,34,Math.toRadians(90))) //midpoint
                //wg to mid pos, intake on, wait intake off, wg up
                .addSpatialMarker(new Vector2d(0, 20), () -> {
                    robot.setWGToMidPosition();
                    robot.turnOnIntakeMotors();
                    this.sleep(200);
                    robot.stopIntakeMotors();
                    robot.setWGToUpPosition();
                })
                .splineTo(new Pose2d(-85,15,Math.toRadians(90))) //drop stone

                .splineTo(new Pose2d(-35,34,Math.toRadians(90))) //midpoint
                //turn on intake motors
                .addSpatialMarker(new Vector2d(0, 20), () -> {
                    robot.turnOnIntakeMotors();
                })
                .splineTo(new Pose2d(-6,39,Math.toRadians(45))) //intake fourth stone
                //turn off intake motors
                .addSpatialMarker(new Vector2d(0, 20), () -> {
                    robot.stopIntakeMotors();
                })
                .splineTo(new Pose2d(-35,34,Math.toRadians(90))) //midpoint
                //wg to mid pos, intake on, wait intake off, wg up
                .addSpatialMarker(new Vector2d(0, 20), () -> {
                    robot.setWGToMidPosition();
                    robot.turnOnIntakeMotors();
                    this.sleep(200);
                    robot.stopIntakeMotors();
                    robot.setWGToUpPosition();
                })
                .splineTo(new Pose2d(-85,15,Math.toRadians(90))) //drop stone
                // wg up
                .addSpatialMarker(new Vector2d(0, 20), () -> {
                    robot.setWGToUpPosition();
                })
                .splineTo(new Pose2d(-35,34,Math.toRadians(90))) //park

                .splineTo(new Pose2d(0,0,Math.toRadians(0))) //cuz im lazy




                .build();
        drive.followTrajectory(traj);

        this.sleep(1000);

//        drive.followTrajectory(
//                drive.trajectoryBuilder(new Pose2d(40, 20, Math.toRadians(180)), true)
//                        .splineTo(new Pose2d(0, 0, Math.toRadians(180)))
//                        .build()
//        );
    }
}
