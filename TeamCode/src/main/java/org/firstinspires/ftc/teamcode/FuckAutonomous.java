package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Auto", group = "At")
public class PedroAuto extends OpMode {
    private Follower follower;
    private PathChain path1;
    private final Pose startPose = new Pose(0, 0, 0);
    private final Pose endPose   = new Pose(36, 0, 0);

    @Override
    public void init() {
       follower = Constants.createFollower(hardwareMap);
       follower.setStartingPose(startPose);
       path1 = follower.pathBuilder()
              .addPath(new BezierLine(startPose, endPose))
              .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
              .build();
       }

    @Override
    public void start() {
        follower.followPath(path1);
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
        }
    }
}
