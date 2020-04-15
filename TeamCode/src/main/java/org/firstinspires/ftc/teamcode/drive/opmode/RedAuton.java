package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ElectroBot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//import org.firstinspires.ftc.teamcode.drive.ElectroBot;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RedAuton extends LinearOpMode {
    private ElectroBot robot = new ElectroBot();  // Initializing our robot
    private DriveConstraints constraints = new DriveConstraints(50.0, 60.0, 0.0, Math.toRadians(270.0), Math.toRadians(270.0), 0.0);
    protected Pose2d startPose = new Pose2d(-36.0, -63.0, Math.toRadians(90.0));

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //drive.setPoseEstimate(new Pose2d(-54,-32,90));

        Trajectory traj1 =
                new TrajectoryBuilder(startPose, false, constraints)
                        .lineTo(new Vector2d(-36.0,-33.0))  //sample
                        .addDisplacementMarker(() -> {

                            robot.turnOnIntakeMotors();

                             })
                        .splineTo(new Pose2d(-36.0,-24.0, Math.toRadians(-45.0)))   //get stone
                        .addDisplacementMarker(() -> {
                            robot.stopIntakeMotors();
                        })
                        .lineTo(new Vector2d(-36.0,-33.0))  //sample

                        .splineTo(new Pose2d(0.0,-36.0,Math.toRadians(0.0)))    //midpoint
                        .splineTo(new Pose2d(48.0,-48.0,Math.toRadians(-90.0)))   //line up with foundation


                        .build();
        //drive.followTrajectory(traj1);
        if (isStopRequested()) return;

        Trajectory traj2 =
                new TrajectoryBuilder(traj1.end(), true, constraints)
                        .lineTo(new Vector2d(48.0,-24.0))   //grab foundation
                        .addDisplacementMarker(() -> {
                            robot.turnOnIntakeMotors();
                        })
                        .build();
        //drive.followTrajectory(traj2);
        if (isStopRequested()) return;

        Trajectory traj3 =
                new TrajectoryBuilder(traj2.end(), false, constraints)
                        .splineTo(new Pose2d(42.0,-48.0,Math.toRadians(180.0)) )  //turn foundation
                        .splineTo(new Pose2d(0.0,-36.0,Math.toRadians(180.0)))    //midpoint
                        .addDisplacementMarker(() -> {
                            robot.turnOnIntakeMotors();
                        })
                        .splineTo(new Pose2d(-66.0,-24.0, Math.toRadians(145.0)))   //get stone
                        .addDisplacementMarker(() -> {
                            robot.stopIntakeMotors();
                        })
                        .build();
        //drive.followTrajectory(traj3);
        if (isStopRequested()) return;


        Trajectory traj4 =
                new TrajectoryBuilder(traj3.end(), true, constraints)
                        .splineTo(new Pose2d(0.0,-36.0,Math.toRadians(180.0)))    //midpoint
                        .splineTo(new Pose2d(42.0,-48.0,Math.toRadians(180.0)))   //drop stone on foundation
                        //drop stone
                        .addDisplacementMarker(() -> {
                            robot.dropStone();
                        })
                        .build();
        //drive.followTrajectory(traj4);
        if (isStopRequested()) return;


        Trajectory traj5 =
                new TrajectoryBuilder(traj4.end(), false, constraints)
                        .splineTo(new Pose2d(0.0,-36.0,Math.toRadians(180.0)))    //midpoint
                        .addDisplacementMarker(() -> {
                            robot.turnOnIntakeMotors();
                        })
                        .splineTo(new Pose2d(-24.0,-24.0, Math.toRadians(145.0)))   //get stone
                        .addDisplacementMarker(() -> {
                            robot.stopIntakeMotors();
                        })
                        .build();
       // drive.followTrajectory(traj5);
        if (isStopRequested()) return;

        Trajectory traj6 =
                new TrajectoryBuilder(traj5.end(), true, constraints)
                        .splineTo(new Pose2d(0.0,-36.0,Math.toRadians(180.0)))    //midpoint
                        .splineTo(new Pose2d(42.0,-48.0,Math.toRadians(180.0)))   //drop stone on foundation
                        //drop stone
                        .addDisplacementMarker(() -> {
                            robot.turnOnIntakeMotors();
                        })
                        .build();
       // drive.followTrajectory(traj6);
        if (isStopRequested()) return;

        Trajectory traj7 =
                new TrajectoryBuilder(traj6.end(), false, constraints)
                        .splineTo(new Pose2d(0.0,-36.0,Math.toRadians(180.0)))    //midpoint
                        .addDisplacementMarker(() -> {
                            robot.turnOnIntakeMotors();
                        })
                        .splineTo(new Pose2d(-30.0,-24.0, Math.toRadians(145.0)))   //get stone
                        .addDisplacementMarker(() -> {
                            robot.stopIntakeMotors();
                        })
                        .build();
        //drive.followTrajectory(traj7);
        if (isStopRequested()) return;

        Trajectory traj8 =
                new TrajectoryBuilder(traj7.end(), true, constraints)
                        .splineTo(new Pose2d(0.0,-36.0,Math.toRadians(180.0)))    //midpoint
                        .splineTo(new Pose2d(42.0,-48.0,Math.toRadians(180.0)))   //drop stone on foundation
                        //drop stone
                        .addDisplacementMarker(() -> {
                            robot.turnOnIntakeMotors();
                        })
                        .build();
        //drive.followTrajectory(traj8);
        if (isStopRequested()) return;

        Trajectory traj9 =
                new TrajectoryBuilder(traj8.end(), false, constraints)
                        .splineTo(new Pose2d(0.0,-36.0,Math.toRadians(180.0)))    //midpoint
                        .addDisplacementMarker(() -> {
                            robot.turnOnIntakeMotors();
                        })
                        .splineTo(new Pose2d(-48.0,-24.0, Math.toRadians(145.0)))   //get stone
                        .addDisplacementMarker(() -> {
                            robot.stopIntakeMotors();
                        })
                        .build();
        //drive.followTrajectory(traj9);
        if (isStopRequested()) return;

        Trajectory traj10 =
                new TrajectoryBuilder(traj9.end(), true, constraints)
                        .splineTo(new Pose2d(0.0,-36.0,Math.toRadians(180.0)))    //midpoint
                        .splineTo(new Pose2d(42.0,-48.0,Math.toRadians(180.0)))   //drop stone on foundation
                        //drop stone
                        .addDisplacementMarker(() -> {
                            robot.turnOnIntakeMotors();
                        })
                        .build();
        //drive.followTrajectory(traj10);
        if (isStopRequested()) return;

        Trajectory traj11 =
                new TrajectoryBuilder(traj10.end(), false, constraints)
                        .splineTo(new Pose2d(0.0,-36.0,Math.toRadians(180.180)))    //midpoint
                        .addDisplacementMarker(() -> {
                            robot.turnOnIntakeMotors();
                        })
                        .splineTo(new Pose2d(-56.0,-24.0, Math.toRadians(145.0)))   //get stone
                        .addDisplacementMarker(() -> {
                            robot.stopIntakeMotors();
                        })
                        .build();
        //drive.followTrajectory(traj11);
        if (isStopRequested()) return;

        Trajectory traj12 =

                new TrajectoryBuilder(traj11.end(), true, constraints)
                        .splineTo(new Pose2d(0.0,-36.0,Math.toRadians(180.0)))    //midpoint
                        .splineTo(new Pose2d(42.0,-48.0,Math.toRadians(180.0)))   //drop stone on foundation
                        //drop stone
                        .addDisplacementMarker(() -> {
                            robot.turnOnIntakeMotors();
                        })

                        .strafeTo(new Vector2d(0.0,-36.0))    //midpoint

                        .build();

        waitForStart();
        //check camera
        //switch trajectory to desired one
        drive.followTrajectory(traj1);
        if (isStopRequested()) return;
        drive.followTrajectory(traj2);
        if (isStopRequested()) return;
        drive.followTrajectory(traj3);
        if (isStopRequested()) return;
        drive.followTrajectory(traj4);
        if (isStopRequested()) return;
        drive.followTrajectory(traj5);
        if (isStopRequested()) return;
        drive.followTrajectory(traj6);
        if (isStopRequested()) return;
        drive.followTrajectory(traj7);
        if (isStopRequested()) return;
        drive.followTrajectory(traj8);
        if (isStopRequested()) return;
        drive.followTrajectory(traj9);
        if (isStopRequested()) return;
        drive.followTrajectory(traj10);
        if (isStopRequested()) return;
        drive.followTrajectory(traj11);
        if (isStopRequested()) return;
        drive.followTrajectory(traj12);
        if (isStopRequested()) return;















        //drive.setPoseEstimate(new Pose2d(-54,-32,90));



//        drive.followTrajectory(
//                drive.trajectoryBuilder(new Pose2d(40, 20, Math.toRadians(180)), true)
//                        .splineTo(new Pose2d(0, 0, Math.toRadians(180)))
//                        .build()
//        );
    }
}



