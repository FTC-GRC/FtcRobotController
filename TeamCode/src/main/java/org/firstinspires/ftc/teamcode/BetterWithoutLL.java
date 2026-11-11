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

@TeleOp(name = "BWLL", group = "At")
public class BetterWithoutLL extends OpMode {
    private Control control;
    private Drive drive;
    private Intake intake;
    private PollenShooter pollenShooter;
    private Telemetry display;

    @Override
    public void init() {
        control       = new Control();
        drive         = new Drive(hardwareMap);
        intake        = new Intake(hardwareMap);
        pollenShooter = new PollenShooter(hardwareMap);
        display       = new Telemetry(telemetry);
    }

    @Override
    public void loop() {
        control.update(gamepad1, gamepad2);
        drive.update(control);
        intake.update(control);
        pollenShooter.update(control);
        display.update(drive, pollenShooter);
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
    boolean tuneNext;   // gamepad2 right bumper: select next gain
    boolean tuneUp;     // gamepad2 D-pad up
    boolean tuneDown;   // gamepad2 D-pad down

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
    }

    private double deadband(double value) {
        if (Math.abs(value) < DEADBAND) return 0;
        double scaled = (Math.abs(value) - DEADBAND) / (1.0 - DEADBAND);
        return Math.signum(value) * scaled * scaled;
    }
}

class Drive {
    static final double MAX_SPEED = 1.0;
    static final double HEADING_SIGN = 1.0;   // change to -1.0 if field-relative feels mirrored

    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final GoBildaPinpointDriver odo;

    private double headingOffsetRad = 0;   // set when A is pressed

    Drive(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotor.class, "fld");
        frontRight = hardwareMap.get(DcMotor.class, "frd");
        backLeft   = hardwareMap.get(DcMotor.class, "bld");
        backRight  = hardwareMap.get(DcMotor.class, "brd");
        odo        = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);   // measure on your robot
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    void update(Control c) {
        odo.update();                       // runs every loop, always on

        if (c.resetYaw) {
            headingOffsetRad = odo.getPosition().getHeading(AngleUnit.RADIANS);
        }

        driveFieldRelative(c.forward, c.right, c.rotate);
    }

    private double getFieldHeadingRad() {
        return AngleUnit.normalizeRadians(
                HEADING_SIGN * (odo.getPosition().getHeading(AngleUnit.RADIANS) - headingOffsetRad));
    }

    double getHeadingDegrees() {
        return Math.toDegrees(getFieldHeadingRad());
    }

    String getStatus() {
        return odo.getDeviceStatus().toString();
    }

    double getRawHeadingDegrees() {
        return odo.getPosition().getHeading(AngleUnit.DEGREES);
    }

    double getXmm() {
        return odo.getPosition().getX(DistanceUnit.MM);
    }

    double getYmm() {
        return odo.getPosition().getY(DistanceUnit.MM);
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta - getFieldHeadingRad());

        drive(r * Math.sin(theta), r * Math.cos(theta), rotate);
    }

    private void drive(double forward, double right, double rotate) {
        double frontLeftPower  = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower  = forward + right - rotate;
        double backLeftPower   = forward - right + rotate;

        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeft.setPower(Range.clip(MAX_SPEED * (frontLeftPower / maxPower), -1, 1));
        frontRight.setPower(Range.clip(MAX_SPEED * (frontRightPower / maxPower), -1, 1));
        backLeft.setPower(Range.clip(MAX_SPEED * (backLeftPower / maxPower), -1, 1));
        backRight.setPower(Range.clip(MAX_SPEED * (backRightPower / maxPower), -1, 1));
    }
}

class Intake {
    private final DcMotor intake;
    private final DcMotor transfer;
    private boolean lastIntakeButton = false;
    private boolean lastTransferButton = false;
    private boolean intakeOn = false;
    private boolean transferOn = false;

    Intake(HardwareMap hardwareMap) {
        intake   = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "ts");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
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
    }
}

class PollenShooter {
    static final double SHOOTER_TARGET_RPM = 3000;
    static final double SHOOTER_CPR = 28;
    static final double SHOOTER_TARGET_TICKS_PER_SEC = SHOOTER_TARGET_RPM * SHOOTER_CPR / 60.0;
    static final double ENCODER_CPR = 8192.0;
    static final double GEAR_RATIO = 59.0 / 18.0;
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
        shooter = hardwareMap.get(DcMotorEx.class, "ps");
        rotator = hardwareMap.get(CRServo.class, "pr");
        encoder = hardwareMap.get(DcMotor.class, "pre");

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
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

class Telemetry {
    private final org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    Telemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    void update(Drive drive, PollenShooter pollenShooter) {
        telemetry.addLine("Left joystick = robot direction");
        telemetry.addLine("Right joystick = turn robot");
        telemetry.addLine("D-pad right/left = rotate servo (release to stop)");
        telemetry.addLine("x = reset field heading");
        telemetry.addLine("△ = PS 2 revolutions");
        telemetry.addLine("○ = intake");
        telemetry.addLine("□ = shooter");
        telemetry.addLine("R1 = transfer");
        telemetry.addLine("G2: R1 = select, D-pad up/down = change selected");
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