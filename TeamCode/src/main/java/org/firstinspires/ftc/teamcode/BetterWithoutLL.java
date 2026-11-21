package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "BWLL", group = "At")
public class BetterWithoutLL extends OpMode {
    private Control control;
    private Drive drive;
    private Intake intake;
    private PollenShooter pollenShooter;
    private PollenHolder pollenHolder;
    private Telemetry display;

    @Override
    public void init() {
        control       = new Control();
        drive         = new Drive(hardwareMap);
        intake        = new Intake(hardwareMap);
        pollenShooter = new PollenShooter(hardwareMap);
        pollenHolder  = new PollenHolder(hardwareMap);
        display       = new Telemetry(telemetry);
    }

    @Override
    public void loop() {
        control.update(gamepad1, gamepad2);
        drive.update(control);
        intake.update(control);
        pollenShooter.update(control);
        pollenHolder.update(control);
        display.update(drive, pollenShooter);
    }
    @Override
    public void start() {
        drive.start();
    }
}

class Control {
    static final double DEADBAND = 0.02;
    static final double TURN_SCALE = 0.80;
    double forward, right, rotate;
    boolean resetYaw;        // A
    boolean intakeToggle;    // B
    boolean pollenToggle;    // X
    boolean transferToggle;  // R1
    boolean ringForward;     // D-pad right (hold)
    boolean ringReverse;     // D-pad left (hold)
    boolean tuneNext;        // gamepad2 right bumper: select next gain
    boolean tuneUp;          // gamepad2 D-pad up
    boolean tuneDown;        // gamepad2 D-pad down
    boolean holderToggle;    // L1
    boolean driveToTarget;   // Y (hold), only if Drive.ENABLE_DRIVE_TO_TARGET
    boolean driveModeToggle; // Share / Back: field-relative <-> robot-relative
    boolean reverseIntake;   // G2 A
    boolean reverseTransfer; // G2 B

    void update(Gamepad g, Gamepad g2) {
        forward = deadband(-g.left_stick_y);
        right   = deadband(g.left_stick_x);
        rotate  = deadband(g.right_stick_x) * TURN_SCALE;

        resetYaw        = g.a;
        intakeToggle    = g.b;
        pollenToggle    = g.x;
        transferToggle  = g.right_bumper;
        ringForward     = g.dpad_right;
        ringReverse     = g.dpad_left;
        tuneNext        = g2.right_bumper;
        tuneUp          = g2.dpad_up;
        tuneDown        = g2.dpad_down;
        holderToggle    = g.left_bumper;
        driveToTarget   = g.y;
        driveModeToggle = g.back;
        reverseIntake   = g2.a;
        reverseTransfer = g2.b;
    }

    private double deadband(double value) {
        if (Math.abs(value) < DEADBAND) return 0;
        double scaled = (Math.abs(value) - DEADBAND) / (1.0 - DEADBAND);
        return Math.signum(value) * scaled * scaled;
    }
}

/**
 * Mecanum drive. Two implementations behind one switch:
 *  - USE_PEDRO = false: the hand-written drive that already ran on the robot (default, safest).
 *  - USE_PEDRO = true:  Pedro Pathing teleop (not yet tested on the robot).
 * Both read heading from the Pinpoint "DE". Share/Back toggles field-relative <-> robot-relative.
 */
class Drive {
    static final boolean USE_PEDRO = false;
    static final boolean ENABLE_DRIVE_TO_TARGET = false;   // Y button, Pedro only. Keep off until Pedro is tuned.
    static final double HEADING_SIGN = 1.0;                // -1.0 if field-relative feels mirrored

    // Pedro
    private Follower follower;
    private boolean autoActive = false;
    private static final Pose TARGET = new Pose(36, 0, Math.toRadians(0));   // example, change before enabling

    // Hand-written
    private DcMotor frontLeft, frontRight, rearLeft, rearRight;
    private GoBildaPinpointDriver odo;
    private double headingOffsetRad = 0;

    private boolean fieldRelative = true;
    private boolean lastToggle = false;

    Drive(HardwareMap hardwareMap) {
        if (USE_PEDRO) {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(0, 0, 0));
            return;
        }
        frontLeft  = hardwareMap.get(DcMotor.class, "LF");
        frontRight = hardwareMap.get(DcMotor.class, "RF");
        rearLeft   = hardwareMap.get(DcMotor.class, "LR");
        rearRight  = hardwareMap.get(DcMotor.class, "RR");
        odo        = hardwareMap.get(GoBildaPinpointDriver.class, "DE");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, rearLeft, rearRight}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();   // robot must be still for a moment after INIT
    }

    void start() {
        if (USE_PEDRO) follower.startTeleopDrive();
    }

    void update(Control c) {
        if (c.driveModeToggle && !lastToggle) fieldRelative = !fieldRelative;
        lastToggle = c.driveModeToggle;

        if (USE_PEDRO) updatePedro(c);
        else updateHandWritten(c);
    }

    private void updatePedro(Control c) {
        follower.update();   // must run every loop

        if (c.resetYaw) {    // reset field heading, keep position
            Pose p = follower.getPose();
            follower.setPose(new Pose(p.getX(), p.getY(), 0));
        }

        boolean wantAuto = ENABLE_DRIVE_TO_TARGET && c.driveToTarget;
        if (wantAuto && !autoActive) {
            PathChain path = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), TARGET))
                    .setLinearHeadingInterpolation(follower.getHeading(), TARGET.getHeading())
                    .build();
            follower.followPath(path);
            autoActive = true;
        } else if (!wantAuto && autoActive) {
            follower.startTeleopDrive();   // button released: back to manual
            autoActive = false;
        }

        if (!autoActive) {
            // Pedro: strafe + = left, turn + = counter-clockwise, so both flip. 4th arg = isRobotCentric.
            follower.setTeleOpDrive(c.forward, -c.right, -c.rotate, !fieldRelative);
        }
    }

    private void updateHandWritten(Control c) {
        odo.update();   // every loop

        if (c.resetYaw) {
            headingOffsetRad = odo.getPosition().getHeading(AngleUnit.RADIANS);
        }

        if (fieldRelative) {
            double theta = Math.atan2(c.forward, c.right);
            double r = Math.hypot(c.right, c.forward);
            theta = AngleUnit.normalizeRadians(theta - getFieldHeadingRad());
            mecanum(r * Math.sin(theta), r * Math.cos(theta), c.rotate);
        } else {
            mecanum(c.forward, c.right, c.rotate);
        }
    }

    private double getFieldHeadingRad() {
        return AngleUnit.normalizeRadians(
                HEADING_SIGN * (odo.getPosition().getHeading(AngleUnit.RADIANS) - headingOffsetRad));
    }

    private void mecanum(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        rearLeft.setPower(bl / max);
        rearRight.setPower(br / max);
    }

    String getMode() { return (USE_PEDRO ? "Pedro, " : "") + (fieldRelative ? "FIELD-relative" : "ROBOT-relative"); }

    double getHeadingDegrees() {
        return USE_PEDRO ? Math.toDegrees(follower.getHeading()) : Math.toDegrees(getFieldHeadingRad());
    }
    double getRawHeadingDegrees() {
        return USE_PEDRO ? Math.toDegrees(follower.getHeading()) : odo.getPosition().getHeading(AngleUnit.DEGREES);
    }
    double getXmm() { return USE_PEDRO ? follower.getPose().getX() * 25.4 : odo.getPosition().getX(DistanceUnit.MM); }
    double getYmm() { return USE_PEDRO ? follower.getPose().getY() * 25.4 : odo.getPosition().getY(DistanceUnit.MM); }
    String getStatus() {
        if (USE_PEDRO) return autoActive ? "Pedro: following" : "Pedro: teleop";
        return odo.getDeviceStatus().toString();
    }
}

class  Intake {
    private final DcMotor intake;
    private final DcMotor transfer;
    private boolean lastIntakeButton = false;
    private boolean lastTransferButton = false;
    private boolean intakeOn = false;
    private boolean transferOn = false;

    Intake(HardwareMap hardwareMap) {
        intake   = hardwareMap.get(DcMotor.class, "IN");
        transfer = hardwareMap.get(DcMotor.class, "RT");
    }

    void update(Control c) {
        if (c.intakeToggle && !lastIntakeButton) {
            intakeOn = !intakeOn;
            intake.setPower(intakeOn ? 1.0 : 0.0);
        }
        if (c.transferToggle && !lastTransferButton) {
            transferOn = !transferOn;
            transfer.setPower(transferOn ? 1.0 : 0.0);
        }
        lastIntakeButton = c.intakeToggle;
        lastTransferButton = c.transferToggle;
        if (c.reverseIntake = true)
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setPower(1.0);
        if (c.reverseIntake = false)
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setPower(0.0);
        if (c.reverseTransfer = true)
            transfer.setDirection(DcMotorSimple.Direction.FORWARD);
            transfer.setPower(1.0);
        if (c.reverseTransfer = false)
            transfer.setDirection(DcMotorSimple.Direction.REVERSE);
            transfer.setPower(0.0);
    }
}

class PollenShooter {
    static final double SHOOTER_TARGET_RPM = 3300;
    static final double SHOOTER_CPR = 28;
    static final double SHOOTER_TARGET_TICKS_PER_SEC = SHOOTER_TARGET_RPM * SHOOTER_CPR / 60.0;
    static final double ENCODER_CPR = 8192.0;
    static final double GEAR_RATIO = 2.0;
    static final double COUNTS_PER_RING_REV = ENCODER_CPR * GEAR_RATIO;
    static final double TARGET_COUNTS = 2 * COUNTS_PER_RING_REV;

    // PIDF gains, in motor power per tick/sec. Tune live with gamepad2, then paste the printed values here.
    // Starting kF = 1 / (free speed in ticks/sec). 6000 RPM motor at 28 CPR: 1 / 2800 = 0.000357
    static final int F = 0, P = 1, I = 2, D = 3;
    static final String[] NAMES = {"kF", "kP", "kI", "kD"};
    static final double[] STEPS = {0.00001, 0.0001, 0.000001, 0.00001};
    private final double[] gains = {0.00051, 0.0005, 0.0, 0.0};   // kF, kP, kI, kD
    private int selected = F;
    private boolean lastNext = false, lastUp = false, lastDown = false;

    private double integral = 0;
    private double lastMeasured = 0;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();

    private final DcMotorEx shooter;
    private final CRServo rotator;
    private final DcMotor encoder;

    private boolean lastShooterButton = false;
    private boolean shooterOn = false;
    private boolean autoRotating = false;
    private int startCounts = 0;

    PollenShooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "PS");
        rotator = hardwareMap.get(CRServo.class, "PR");
        encoder = hardwareMap.get(DcMotor.class, "PE");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Must be RUN_WITHOUT_ENCODER: the loop below sets the power itself,
        // and RUN_USING_ENCODER would add the hub's own velocity PID on top of it.
        // The encoder can still be read in this mode.
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotator.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timer.reset();
    }

    void update(Control c) {
        tune(c);

        if (c.pollenToggle && !lastShooterButton) {
            shooterOn = !shooterOn;
        }
        lastShooterButton = c.pollenToggle;

        // PIDF velocity loop, runs every cycle
        double dt = Math.max(timer.seconds(), 1e-3);
        timer.reset();
        double measured = shooter.getVelocity();

        if (shooterOn) {
            double target = SHOOTER_TARGET_TICKS_PER_SEC;
            double error = target - measured;

            integral += error * dt;
            if (gains[I] > 0) {   // keep the I term from contributing more than 30% power
                double limit = 0.3 / gains[I];
                integral = Range.clip(integral, -limit, limit);
            }
            double derivative = -(measured - lastMeasured) / dt;   // on measurement, no kick when toggled

            power = gains[F] * target + gains[P] * error + gains[I] * integral + gains[D] * derivative;
            power = Range.clip(power, 0.0, 1.0);
        } else {
            power = 0;
            integral = 0;
        }
        shooter.setPower(power);
        lastMeasured = measured;

        // Ring rotator
        if (autoRotating && Math.abs(encoder.getCurrentPosition() - startCounts) >= TARGET_COUNTS) {
            rotator.setPower(0.0);
            autoRotating = false;
        }
        if (!autoRotating) {
            if (c.ringForward) {
                rotator.setPower(1.0);
            } else if (c.ringReverse) {
                rotator.setPower(-1.0);
            } else {
                rotator.setPower(0.0);
            }
        }
    }

    private void tune(Control c) {
        if (c.tuneNext && !lastNext) selected = (selected + 1) % gains.length;
        if (c.tuneUp && !lastUp)     gains[selected] += STEPS[selected];
        if (c.tuneDown && !lastDown) gains[selected] = Math.max(0, gains[selected] - STEPS[selected]);
        lastNext = c.tuneNext;
        lastUp = c.tuneUp;
        lastDown = c.tuneDown;
    }

    double getAngleDegrees() {
        return encoder.getCurrentPosition() * 360.0 / COUNTS_PER_RING_REV;
    }

    double getShooterRpm() {
        return shooter.getVelocity() * 60.0 / SHOOTER_CPR;
    }

    double getPower() { return power; }
    String getSelectedName() { return NAMES[selected]; }
    double getGain(int i) { return gains[i]; }
}

class PollenHolder {
    static final double HOLD_POS = 0.0;      // tune these on the robot
    static final double RELEASE_POS = 0.5;

    private final Servo holder;
    private boolean lastButton = false;
    private boolean released = false;

    PollenHolder(HardwareMap hardwareMap) {
        holder = hardwareMap.get(Servo.class, "PH");
        holder.setPosition(HOLD_POS);
    }

    void update(Control c) {
        if (c.holderToggle && !lastButton) {
            released = !released;
            holder.setPosition(released ? RELEASE_POS : HOLD_POS);
        }
        lastButton = c.holderToggle;
    }

    boolean isReleased() { return released; }
}
class Telemetry {
    private final org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    Telemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    void update(Drive drive, PollenShooter pollenShooter) {
        telemetry.addLine("1. Left joystick = robot direction");
        telemetry.addLine("1. Right joystick = turn robot");
        telemetry.addLine("1. D-pad right/left = rotate servo (release to stop)");
        telemetry.addLine("1. X = reset field heading");
        telemetry.addLine("1. Share/Back = toggle field / robot relative");
        if (Drive.ENABLE_DRIVE_TO_TARGET) telemetry.addLine("1. △ hold = drive to target");
        telemetry.addData("Drive mode", drive.getMode());
        telemetry.addLine("1. ○ = intake");
        telemetry.addLine("1. □ = shooter");
        telemetry.addLine("1. R1 = transfer");
        telemetry.addLine("1. L1 = pollen holder (hold / release)");
        telemetry.addLine("2. R1 = select, D-pad up/down = change selected");
        telemetry.addData("Pinpoint status", drive.getStatus());
        telemetry.addData("Raw heading (deg)", "%.1f", drive.getRawHeadingDegrees());
        telemetry.addData("Field heading (deg)", "%.1f", drive.getHeadingDegrees());
        telemetry.addData("Odo X / Y (mm)", "%.0f / %.0f", drive.getXmm(), drive.getYmm());
        telemetry.addData("Rotator angle (deg)", "%.1f", pollenShooter.getAngleDegrees());
        telemetry.addData("Shooter RPM", "%.0f", pollenShooter.getShooterRpm());
        telemetry.addData("Shooter target RPM", "%.0f", PollenShooter.SHOOTER_TARGET_RPM);
        telemetry.addData("Shooter power", "%.2f", pollenShooter.getPower());
        telemetry.addData("Tuning: selected", pollenShooter.getSelectedName());
        telemetry.addData("kF / kP / kI / kD", "%.6f / %.6f / %.6f / %.6f",
                pollenShooter.getGain(PollenShooter.F), pollenShooter.getGain(PollenShooter.P),
                pollenShooter.getGain(PollenShooter.I), pollenShooter.getGain(PollenShooter.D));
    }
}