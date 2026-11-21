package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Simple Shoot + Park", group = "At", preselectTeleOp = "BWLL")
public class Auto21_11_26 extends LinearOpMode {
    // ---- Placeholders: measure on the field. The start pose is (0, 0) facing heading 0. ----
    // Pinpoint axes: +X = robot forward at start, +Y = robot left at start.
    static final double MOVE_X_MM = -300;      // phase 2: leave the shooting spot
    static final double MOVE_Y_MM = 0;
    static final double MOVE_HEADING_DEG = 0;
    static final double PARK_X_MM = -300;      // phase 3: final parking spot
    static final double PARK_Y_MM = 900;
    static final double PARK_HEADING_DEG = 0;

    @Override
    public void runOpMode() {
        AutoDrive drive = new AutoDrive(this, hardwareMap);
        AutoShooter shooter = new AutoShooter(this, hardwareMap);

        ShootOnStart shootOnStart     = new ShootOnStart(shooter);
        MoveAfterShoot moveAfterShoot = new MoveAfterShoot(drive);
        ParkAfterMove parkAfterMove   = new ParkAfterMove(drive);

        telemetry.addLine("Ready. Keep the robot still until start.");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) shootOnStart.run();
        if (opModeIsActive()) moveAfterShoot.run();
        if (opModeIsActive()) parkAfterMove.run();

        drive.stop();
        shooter.stop();
    }
}

/** Phase 1: spin up the shooter, then feed the pollen. */
class ShootOnStart {
    static final double SPINUP_TIMEOUT_S = 4.0;
    static final double FEED_TIME_S = 3.0;

    private final AutoShooter shooter;

    ShootOnStart(AutoShooter shooter) { this.shooter = shooter; }

    void run() {
        shooter.spinUpAndWait(SPINUP_TIMEOUT_S);
        shooter.feed(FEED_TIME_S);
        shooter.stop();
    }
}

/** Phase 2: drive away from the start position. */
class MoveAfterShoot {
    private final AutoDrive drive;

    MoveAfterShoot(AutoDrive drive) { this.drive = drive; }

    void run() {
        drive.driveTo(Auto21_11_26.MOVE_X_MM, Auto21_11_26.MOVE_Y_MM,
                Auto21_11_26.MOVE_HEADING_DEG, 4.0);
    }
}

/** Phase 3: drive to the parking spot. */
class ParkAfterMove {
    private final AutoDrive drive;

    ParkAfterMove(AutoDrive drive) { this.drive = drive; }

    void run() {
        drive.driveTo(Auto21_11_26.PARK_X_MM, Auto21_11_26.PARK_Y_MM,
                Auto21_11_26.PARK_HEADING_DEG, 5.0);
    }
}

/** Drives to field positions using the Pinpoint. Zero is wherever the robot starts. */
class AutoDrive {
    static final double MAX_POWER = 0.6;
    static final double KP_POS = 1.0 / 400.0;   // full power when 400 mm away
    static final double KP_HEADING = 1.0;
    static final double POS_TOLERANCE_MM = 25;
    static final double HEADING_TOLERANCE_RAD = Math.toRadians(3);

    private final LinearOpMode op;
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final GoBildaPinpointDriver odo;

    AutoDrive(LinearOpMode op, HardwareMap hw) {
        this.op = op;
        frontLeft  = hw.get(DcMotor.class, "LF");
        frontRight = hw.get(DcMotor.class, "RF");
        backLeft   = hw.get(DcMotor.class, "LR");
        backRight  = hw.get(DcMotor.class, "RR");
        odo        = hw.get(GoBildaPinpointDriver.class, "DE");

        // Same directions as the TeleOp
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);   // same as the TeleOp
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();   // the robot must be still here
    }

    void driveTo(double xMm, double yMm, double headingDeg, double timeoutS) {
        double targetHeading = Math.toRadians(headingDeg);
        ElapsedTime timer = new ElapsedTime();

        while (op.opModeIsActive() && timer.seconds() < timeoutS) {
            odo.update();
            double x = odo.getPosition().getX(DistanceUnit.MM);
            double y = odo.getPosition().getY(DistanceUnit.MM);
            double h = odo.getPosition().getHeading(AngleUnit.RADIANS);

            double ex = xMm - x;
            double ey = yMm - y;
            double headingError = AngleUnit.normalizeRadians(targetHeading - h);

            if (Math.hypot(ex, ey) < POS_TOLERANCE_MM
                    && Math.abs(headingError) < HEADING_TOLERANCE_RAD) break;

            // Field error to robot frame (Pinpoint: heading is counter-clockwise positive)
            double forwardErr = ex * Math.cos(h) + ey * Math.sin(h);
            double leftErr = -ex * Math.sin(h) + ey * Math.cos(h);

            double forward = Range.clip(forwardErr * KP_POS, -MAX_POWER, MAX_POWER);
            double right   = Range.clip(-leftErr * KP_POS, -MAX_POWER, MAX_POWER);
            double rotate  = Range.clip(-headingError * KP_HEADING, -MAX_POWER, MAX_POWER);  // + = clockwise

            setMecanum(forward, right, rotate);

            op.telemetry.addData("Pose", "%.0f, %.0f mm  %.1f deg", x, y, Math.toDegrees(h));
            op.telemetry.addData("Target", "%.0f, %.0f mm  %.1f deg", xMm, yMm, headingDeg);
            op.telemetry.update();
        }
        stop();
    }

    void stop() { setMecanum(0, 0, 0); }

    private void setMecanum(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double br = forward + right - rotate;
        double bl = forward - right + rotate;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(br), Math.abs(bl))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
        backLeft.setPower(bl / max);
    }
}

/** Pollen shooter at a target RPM (same feedforward + P loop as the TeleOp), plus the holder and transfer. */
class AutoShooter {
    static final double TARGET_RPM = 3000;
    static final double CPR = 28;
    static final double TARGET_TICKS_PER_SEC = TARGET_RPM * CPR / 60.0;
    static final double RPM_TOLERANCE = 100;
    static final double KF = 0.00051, KP = 0.0005;   // paste your tuned values here

    static final double HOLD_POS = 0.0;       // same placeholders as the TeleOp holder
    static final double RELEASE_POS = 0.5;

    private final LinearOpMode op;
    private final DcMotorEx shooter;
    private final DcMotor transfer;
    private final Servo holder;

    AutoShooter(LinearOpMode op, HardwareMap hw) {
        this.op = op;
        shooter  = hw.get(DcMotorEx.class, "PS");
        transfer = hw.get(DcMotor.class, "RT");
        holder   = hw.get(Servo.class, "PH");

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        holder.setPosition(HOLD_POS);
    }

    /** Must be called every loop while the shooter should stay at speed. */
    private void holdSpeed() {
        double error = TARGET_TICKS_PER_SEC - shooter.getVelocity();
        shooter.setPower(Range.clip(KF * TARGET_TICKS_PER_SEC + KP * error, 0, 1));
    }

    private double rpm() { return shooter.getVelocity() * 60.0 / CPR; }

    void spinUpAndWait(double timeoutS) {
        ElapsedTime timer = new ElapsedTime();
        while (op.opModeIsActive() && timer.seconds() < timeoutS
                && Math.abs(rpm() - TARGET_RPM) > RPM_TOLERANCE) {
            holdSpeed();
            op.telemetry.addData("Shooter RPM", "%.0f", rpm());
            op.telemetry.update();
        }
    }

    void feed(double seconds) {
        holder.setPosition(RELEASE_POS);
        transfer.setPower(1.0);
        ElapsedTime timer = new ElapsedTime();
        while (op.opModeIsActive() && timer.seconds() < seconds) {
            holdSpeed();
            op.telemetry.addData("Shooter RPM", "%.0f", rpm());
            op.telemetry.update();
        }
        transfer.setPower(0);
        holder.setPosition(HOLD_POS);
    }

    void stop() {
        shooter.setPower(0);
        transfer.setPower(0);
    }
}