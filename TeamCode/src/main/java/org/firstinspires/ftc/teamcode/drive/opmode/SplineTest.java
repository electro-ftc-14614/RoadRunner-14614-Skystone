package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-54,-32,90));
        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(-54,-32,90));
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())

                .splineTo(new Pose2d(-32,-32,0))//sampling
                .splineTo(new Pose2d(-32,52,0))//going to foundation
                .splineTo(new Pose2d(-48,52,180))//turning foundation
                .splineTo(new Pose2d(-32,60,90))//getting second stone
                .splineTo(new Pose2d(-48,52,180))//going to foundation
                .splineTo(new Pose2d(-32,66,90))//getting third stone
                .splineTo(new Pose2d(-48,52,180))//going to foundation
                .splineTo(new Pose2d(-32,72,90))//getting fourth stone
                .splineTo(new Pose2d(-48,52,180))//going to foundation
                .splineTo(new Pose2d(-32,78,90))//getting fifth stone
                .splineTo(new Pose2d(-48,52,180))//going to foundation
                .splineTo(new Pose2d(-32,78,90))//getting sixth stone
                .splineTo(new Pose2d(-48,52,180))//going to foundation
                .splineTo(new Pose2d(-36,0,180))//parking


                .build();
        drive.followTrajectory(traj);





        sleep(2000);

//        drive.followTrajectory(
//                drive.trajectoryBuilder(new Pose2d(40, 20, Math.toRadians(180)), true)
//                        .splineTo(new Pose2d(0, 0, Math.toRadians(180)))
//                        .build()
//        );
    }
}
