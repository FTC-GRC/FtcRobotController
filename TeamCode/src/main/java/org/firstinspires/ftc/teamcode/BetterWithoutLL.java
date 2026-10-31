package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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
        control.update(gamepad1);
        drive.update(control);
        intake.update(control);
        pollenShooter.update(control);
        display.update(drive, pollenShooter);
    }
}

/** Reads the gamepad once per loop and turns it into named values. All button assignments live here. */
class Control {
    static final double DEADBAND = 0.02;
    static final double TURN_SCALE = 0.80;

    // Drive
    double forward, right, rotate;
    boolean robotRelative;   // left bumper held
    boolean resetYaw;        // A

    // Mechanisms
    boolean intakeToggle;    // B
    boolean pollenToggle;    // X
    boolean ringTwoTurns;    // Y
    boolean ringForward;     // D-pad right (hold)
    boolean ringReverse;     // D-pad left (hold)

    void update(Gamepad g) {
        forward = deadband(-g.left_stick_y);
        right   = deadband(g.left_stick_x);
        rotate  = deadband(g.right_stick_x) * TURN_SCALE;
        robotRelative = g.left_bumper;
        resetYaw = g.a;

        intakeToggle = g.b;
        pollenToggle = g.x;
        ringTwoTurns = g.y;
        ringForward  = g.dpad_right;
        ringReverse  = g.dpad_left;
    }

    private double deadband(double value) {
        if (Math.abs(value) < DEADBAND) return 0;
        double scaled = (Math.abs(value) - DEADBAND) / (1.0 - DEADBAND);
        return Math.signum(value) * scaled * scaled;
    }
}

/** Mecanum drive with field-relative control using the goBILDA Pinpoint for heading. */
class Drive {
    static final double MAX_SPEED = 1.0;

    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final GoBildaPinpointDriver odo;

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
        odo.update();

        if (c.resetYaw) {
            resetYaw();
        }

        if (c.robotRelative) {
            drive(c.forward, c.right, c.rotate);
        } else {
            driveFieldRelative(c.forward, c.right, c.rotate);
        }
    }

    double getHeadingDegrees() {
        return odo.getPosition().getHeading(AngleUnit.DEGREES);
    }

    private void resetYaw() {
        Pose2D current = odo.getPosition();
        odo.setPosition(new Pose2D(DistanceUnit.MM,
                current.getX(DistanceUnit.MM),
                current.getY(DistanceUnit.MM),
                AngleUnit.RADIANS, 0));
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta - odo.getPosition().getHeading(AngleUnit.RADIANS));

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

/** The intake motor. Toggles on/off each time its button is pressed. */
class Intake {
    private final DcMotor motor;
    private boolean lastButton = false;
    private boolean on = false;

    Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "Intake");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    void update(Control c) {
        if (c.intakeToggle && !lastButton) {
            on = !on;
            motor.setPower(on ? 1.0 : 0.0);
        }
        lastButton = c.intakeToggle;
    }
}

/** The shooter motor, plus the CRServo ring and its Through Bore Encoder. */
class PollenShooter {
    // Through Bore Encoder: 8192 counts per turn. The encoder gear (18T) meshes with the ring (59T).
    static final double ENCODER_CPR = 8192.0;
    static final double GEAR_RATIO = 59.0 / 18.0;
    static final double COUNTS_PER_RING_REV = ENCODER_CPR * GEAR_RATIO;
    static final double TARGET_COUNTS = 2 * COUNTS_PER_RING_REV;

    private final DcMotor shooter;
    private final CRServo rotator;
    private final DcMotor encoder;

    private boolean lastShooterButton = false;
    private boolean shooterOn = false;
    private boolean lastRingButton = false;
    private boolean autoRotating = false;
    private int startCounts = 0;

    PollenShooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotor.class, "ps");
        rotator = hardwareMap.get(CRServo.class, "pr");
        encoder = hardwareMap.get(DcMotor.class, "pre");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rotator.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void update(Control c) {
        // Shooter toggle
        if (c.pollenToggle && !lastShooterButton) {
            shooterOn = !shooterOn;
            shooter.setPower(shooterOn ? 1.0 : 0.0);
        }
        lastShooterButton = c.pollenToggle;

        // Rotate the ring exactly 2 revolutions
        if (c.ringTwoTurns && !lastRingButton) {
            autoRotating = true;
            startCounts = encoder.getCurrentPosition();
            rotator.setPower(1.0);   // change to -1.0 if it spins the wrong way
        }
        lastRingButton = c.ringTwoTurns;

        if (autoRotating && Math.abs(encoder.getCurrentPosition() - startCounts) >= TARGET_COUNTS) {
            rotator.setPower(0.0);
            autoRotating = false;
        }

        // Hold to rotate, release to stop
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

    double getAngleDegrees() {
        return encoder.getCurrentPosition() * 360.0 / COUNTS_PER_RING_REV;
    }
}

/** Puts the control reminders and live readings on the Driver Station. */
class Telemetry {
    // Fully qualified because this class has the same name as the SDK's Telemetry type
    private final org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    Telemetry(org.firstinspires.ftc.robotcore.external.Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    void update(Drive drive, PollenShooter pollenShooter) {
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addLine("Press B to toggle intake");
        telemetry.addLine("Press X to toggle shooter");
        telemetry.addLine("Hold D-pad right/left: rotate servo (release to stop)");
        telemetry.addLine("Press Y: rotate ring 2 revolutions");
        telemetry.addData("Heading (deg)", "%.1f", drive.getHeadingDegrees());
        telemetry.addData("Rotator angle (deg)", "%.1f", pollenShooter.getAngleDegrees());
    }
}