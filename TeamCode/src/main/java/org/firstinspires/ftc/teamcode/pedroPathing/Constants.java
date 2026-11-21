package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing setup for this robot. Hardware names, motor directions and Pinpoint
 * offsets are copied from the old hand-written Drive class (commit 2daad9c), which drove correctly.
 */
public class Constants {
    // TODO tune: robot mass in kg (Pedro uses it for path following, not for teleop)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("LF")
            .rightFrontMotorName("RF")
            .leftRearMotorName("LR")
            .rightRearMotorName("RR")
            // Same directions as the old Drive class. Pedro's defaults are the opposite
            // (left REVERSE, right FORWARD), so leaving them out makes the robot drive backwards.
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .useBrakeModeInTeleOp(true);   // Pedro defaults to coasting in teleop; old code used BRAKE
            // TODO tune for autonomous: .xVelocity(...) .yVelocity(...) from Pedro's velocity tuners

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("DE")
            .distanceUnit(DistanceUnit.MM)
            // Old code: odo.setOffsets(-84, -168, MM) -> (forward pod sideways offset, strafe pod forward offset)
            .forwardPodY(-84)
            .strafePodX(-168)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
