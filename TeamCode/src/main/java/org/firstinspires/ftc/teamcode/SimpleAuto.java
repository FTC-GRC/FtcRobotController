package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.BooleanSupplier;

/**
 * BIOBUZZ auto: shoot the 4 pre-loads into our upward-facing CELL, LEAVE, PARK in the LOADING ZONE.
 * Uses the Pinpoint + odometry pods (no Pedro, no tuning). Robot is 45 x 45 cm (17.7 in).
 *
 * START (same for RED and BLUE, the field is rotationally symmetric):
 *   - Back of the robot against the perimeter wall on the AUDIENCE side (red) / FAR side (blue),
 *     i.e. the wall the upward-facing CELL of our HIVE points at.
 *   - Facing straight at our HIVE's upward-facing CELL.
 *   - Robot's LEFT side 2.4 in (6 cm) from the tile seam on its left
 *     (red: seam between columns B and C; blue: seam between columns D and E).
 *     That puts the robot centered on our HIVE (25.5 in HIVE spacing -> center 11.25 in from that seam).
 *
 * PATH (robot coordinates, inches: X = forward from start, Y = left from start):
 *   1. shoot from the start
 *   2. forward  to (21, 0)      off the wall = LEAVE; front stays ~13 in short of the HIVE frame
 *   3. left     to (21, 29)     clear of the HIVE frame by ~8 in
 *   4. forward  to (99, 29)     past the FLOWER on the alliance wall by ~13 in
 *   5. left     to (99, 46)     robot ends ~4 in off the alliance wall, ~7 in inside the LOADING ZONE
 */
@Autonomous(name = "Simple Shoot + Park", group = "At", preselectTeleOp = "BWLL")
public class SimpleAuto extends LinearOpMode {

    // ---------------- TUNE ----------------
    static final double START_DELAY_S    = 0.0;   // wait before starting (stay out of your partner's way)

    // Shooting: shooter on -> wait for speed -> transfer + intake together
    static final double SHOOTER_RPM      = 3300;  // same as teleop
    static final double SPINUP_MIN_S     = 0.5;   // always wait at least this long
    static final double SPINUP_TIMEOUT_S = 2.5;   // max wait for the shooter to reach speed
    static final double FEED_TIME_S      = 5.0;  // transfer + intake on this long to push all 4 POLLEN through

    static final boolean SKIP_PARK       = false; // true = only LEAVE (step 2), stay away from the LOADING ZONE

    // Waypoints in inches (robot coordinates, see PATH above)
    static final double LEAVE_X = 21, SIDE_Y = 29, ZONE_X = 99, PARK_Y = 46;

    static final double MAX_POWER        = 0.5;   // drive speed cap
    static final boolean HOLD_HEADING    = true;  // false if the Pinpoint heading is unreliable
    // --------------------------------------

    // Simple position controller (no tuning should be needed at MAX_POWER 0.5)
    static final double KP_DRIVE = 0.06;          // power per inch of error
    static final double KP_TURN  = 1.0;           // power per radian of heading error
    static final double MIN_POWER = 0.12;         // enough to overcome friction near the target
    static final double POS_TOLERANCE_IN = 1.0;
    static final double HEADING_TOLERANCE_RAD = Math.toRadians(3);

    // Hardware constants copied from BetterWithoutLL
    static final double SHOOTER_CPR = 28;
    static final double SHOOTER_TARGET_TPS = SHOOTER_RPM * SHOOTER_CPR / 60.0;
    static final double KF = 0.00051, KP = 0.0005;
    static final double HOLD_POS = 0.0;

    private DcMotor lf, rf, lr, rr, intake, transfer;
    private DcMotorEx shooter;
    private GoBildaPinpointDriver odo;

    private boolean shooterOn = false;
    private double shooterPower = 0;
    private String step = "init";

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, "LF");
        rf = hardwareMap.get(DcMotor.class, "RF");
        lr = hardwareMap.get(DcMotor.class, "LR");
        rr = hardwareMap.get(DcMotor.class, "RR");
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor m : new DcMotor[]{lf, rf, lr, rr}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intake   = hardwareMap.get(DcMotor.class, "IN");
        transfer = hardwareMap.get(DcMotor.class, "RT");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);    // same as teleop
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotorEx.class, "PS");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Same as teleop INIT, so nothing twitches when teleop is initialised (G403)
        hardwareMap.get(Servo.class, "PH").setPosition(HOLD_POS);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "DE");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();   // keep the robot still for a moment after INIT

        while (opModeInInit()) {
            odo.update();
            telemetry.addLine("START: back on the wall the upward CELL points at,");
            telemetry.addLine("  facing that CELL, robot's left side 6 cm from the tile seam on its left");
            telemetry.addData("Park", SKIP_PARK ? "OFF (leave only)" : "LOADING ZONE");
            telemetry.addLine("Check: push fwd -> X up, push left -> Y up, turn left -> heading up");
            telemetry.addData("Pinpoint", odo.getDeviceStatus());
            telemetry.addData("X / Y (in)", "%.1f / %.1f", odo.getPosition().getX(DistanceUnit.INCH),
                    odo.getPosition().getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.1f", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
        if (isStopRequested()) return;

        // Start position = (0, 0, 0). X = forward, Y = left, heading counter-clockwise.
        odo.resetPosAndIMU();
        runFor(0.3, () -> false);   // the reset takes ~0.25 s; then wait (max 1 s more) for READY
        runFor(1.0, () -> odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY);

        step = "start delay";
        runFor(START_DELAY_S, () -> false);

        // 1. Shoot: shooter on, wait for speed, then transfer + intake together
        step = "spin up";
        shooterOn = true;
        runFor(SPINUP_MIN_S, () -> false);
        runFor(SPINUP_TIMEOUT_S - SPINUP_MIN_S, () -> shooter.getVelocity() > 0.95 * SHOOTER_TARGET_TPS);

        step = "shooting";
        transfer.setPower(1.0);
        intake.setPower(1.0);
        runFor(FEED_TIME_S, () -> false);
        transfer.setPower(0);
        intake.setPower(0);
        shooterOn = false;

        // 2-5. LEAVE, then go around the HIVE side and into the LOADING ZONE
        step = "leave";
        boolean ok = driveTo(LEAVE_X, 0);
        if (ok && !SKIP_PARK) { step = "to side";    ok = driveTo(LEAVE_X, SIDE_Y); }
        if (ok && !SKIP_PARK) { step = "along wall"; ok = driveTo(ZONE_X, SIDE_Y); }
        if (ok && !SKIP_PARK) { step = "park";       driveTo(ZONE_X, PARK_Y); }
        drive(0, 0, 0);

        if (!step.startsWith("ABORTED")) step = "done";
        runFor(30, () -> false);   // hold still (G403) and keep telemetry alive until AUTO ends
    }

    /**
     * Drives to (x, y) inches from the start, keeping heading 0, using the Pinpoint.
     * Returns false (and stops) if the odometry looks wrong, so a bad sensor can't send the
     * robot into the HIVE or across the field (G402: no crossing into the opponent's side in AUTO).
     */
    private boolean driveTo(double targetX, double targetY) {
        odo.update();
        double startX = odo.getPosition().getX(DistanceUnit.INCH);
        double startY = odo.getPosition().getY(DistanceUnit.INCH);
        double startDist = Math.hypot(targetX - startX, targetY - startY);
        double timeout = 1.5 + startDist / 12.0;   // generous: ~12 in/s average including slowing down
        ElapsedTime t = new ElapsedTime();

        while (opModeIsActive() && t.seconds() < timeout) {
            odo.update();
            double x = odo.getPosition().getX(DistanceUnit.INCH);
            double y = odo.getPosition().getY(DistanceUnit.INCH);
            double h = HOLD_HEADING ? odo.getPosition().getHeading(AngleUnit.RADIANS) : 0;

            double ex = targetX - x;
            double ey = targetY - y;
            double eh = AngleUnit.normalizeRadians(0 - h);
            double dist = Math.hypot(ex, ey);
            if (dist < POS_TOLERANCE_IN && Math.abs(eh) < HEADING_TOLERANCE_RAD) break;

            // Safety checks
            String abort = null;
            if (t.seconds() > 0.75 && Math.hypot(x - startX, y - startY) < 0.5) abort = "odometry not moving (pods/Pinpoint?)";
            else if (dist > startDist + 4) abort = "moving AWAY from target (pod direction?)";
            else if (Math.abs(eh) > Math.toRadians(30)) abort = "heading off by >30 deg (heading sign?)";
            if (abort != null) {
                drive(0, 0, 0);
                step = "ABORTED: " + abort;
                telemetry.addData("Step", step);
                telemetry.update();
                return false;
            }

            // Field error -> robot error (forward / left)
            double fwd  =  ex * Math.cos(h) + ey * Math.sin(h);
            double left = -ex * Math.sin(h) + ey * Math.cos(h);

            double power = Range.clip(KP_DRIVE * dist, dist < POS_TOLERANCE_IN ? 0 : MIN_POWER, MAX_POWER);
            double scale = dist > 1e-6 ? power / dist : 0;
            double turn = HOLD_HEADING ? Range.clip(-KP_TURN * eh, -MAX_POWER, MAX_POWER) : 0;   // +rotate = clockwise

            drive(fwd * scale, -left * scale, turn);
            telemetry.addData("Step", step);
            telemetry.addData("Target X / Y", "%.1f / %.1f", targetX, targetY);
            telemetry.addData("Now X / Y / H", "%.1f / %.1f / %.1f", x, y, Math.toDegrees(h));
            telemetry.update();
        }
        drive(0, 0, 0);
        return true;   // reached the target, or ran out of time without anything looking wrong
    }

    /** Waits up to `seconds` (or until `done` is true) while keeping the shooter speed loop running. */
    private void runFor(double seconds, BooleanSupplier done) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < seconds && !done.getAsBoolean()) {
            odo.update();
            updateShooter();
            telemetry.addData("Step", step);
            telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getVelocity() * 60.0 / SHOOTER_CPR, SHOOTER_RPM);
            telemetry.addData("Shooter power", "%.2f", shooterPower);
            telemetry.update();
        }
        updateShooter();
    }

    /** Same feedforward + P control as the teleop shooter. */
    private void updateShooter() {
        if (shooterOn) {
            double error = SHOOTER_TARGET_TPS - shooter.getVelocity();
            shooterPower = Range.clip(KF * SHOOTER_TARGET_TPS + KP * error, 0.0, 1.0);
        } else {
            shooterPower = 0;
        }
        shooter.setPower(shooterPower);
    }

    /** Robot-relative mecanum drive, same mixing as the teleop Drive class. */
    private void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
        lf.setPower(fl / max);
        rf.setPower(fr / max);
        lr.setPower(bl / max);
        rr.setPower(br / max);
    }
}
