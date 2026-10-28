package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "BWLL", group = "At")
public class BetterWithoutLL extends OpMode {
    static final double DEADBAND = 0.02;
    static final double MAX_SPEED = 1.0;
    static final double TURN_SCALE = 0.80;

    // Through Bore Encoder: 8192 counts per turn. The encoder gear (18T) meshes with the ring (59T).
    static final double ENCODER_CPR = 8192.0;
    static final double GEAR_RATIO = 59.0 / 18.0;
    static final double COUNTS_PER_BIG_GEAR_REV = ENCODER_CPR * GEAR_RATIO;
    static final double TARGET_COUNTS = 2 * COUNTS_PER_BIG_GEAR_REV;

    private boolean lastButtonState = false;
    private boolean motorOn = false;
    private boolean lastShooterButtonState = false;
    private boolean shooterOn = false;
    private boolean lastYState = false;
    private boolean autoRotating = false;
    private int startCounts = 0;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor intakeMotor;
    DcMotor pollenShooter;
    DcMotor rotatorEncoder;
    CRServo pollenRotator;
    GoBildaPinpointDriver odo;

    @Override
    public void init() {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frd");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "bld");
        backRightDrive  = hardwareMap.get(DcMotor.class, "brd");
        intakeMotor     = hardwareMap.get(DcMotor.class, "Intake");
        pollenShooter   = hardwareMap.get(DcMotor.class, "ps");
        pollenRotator   = hardwareMap.get(CRServo.class, "pr");
        rotatorEncoder  = hardwareMap.get(DcMotor.class, "pre");

        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pollenShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        pollenRotator.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotatorEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotatorEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);   // measure on your robot
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    @Override
    public void loop() {
        odo.update();
        odo.getPosition().getHeading(AngleUnit.DEGREES);

        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addLine("Press B to toggle intake");
        telemetry.addLine("Press X to toggle shooter");
        telemetry.addLine("D-pad right/left: rotate servo, D-pad down: stop servo");
        telemetry.addLine("Press Y: rotate ring 2 revolutions");
        telemetry.addData("Heading (deg)", "%.1f", odo.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("Rotator angle (deg)", "%.1f",
                rotatorEncoder.getCurrentPosition() * 360.0 / COUNTS_PER_BIG_GEAR_REV);

        if (gamepad1.a) {
            resetYaw();
        }

        // Intake toggle (B)
        boolean currentButtonState = gamepad1.b;
        if (currentButtonState && !lastButtonState) {
            motorOn = !motorOn;
            intakeMotor.setPower(motorOn ? 1.0 : 0.0);
        }
        lastButtonState = currentButtonState;

        // Shooter toggle (X)
        boolean currentShooterButtonState = gamepad1.x;
        if (currentShooterButtonState && !lastShooterButtonState) {
            shooterOn = !shooterOn;
            pollenShooter.setPower(shooterOn ? 1.0 : 0.0);
        }
        lastShooterButtonState = currentShooterButtonState;

        // Y: rotate the ring 2 revolutions
        if (gamepad1.y && !lastYState) {
            autoRotating = true;
            startCounts = rotatorEncoder.getCurrentPosition();
            pollenRotator.setPower(1.0);   // change to -1.0 if it spins the wrong way
        }
        lastYState = gamepad1.y;

        if (autoRotating && Math.abs(rotatorEncoder.getCurrentPosition() - startCounts) >= TARGET_COUNTS) {
            pollenRotator.setPower(0.0);
            autoRotating = false;
        }

        double forward = deadband(-gamepad1.left_stick_y);
        double right   = deadband(gamepad1.left_stick_x);
        double rotate  = deadband(gamepad1.right_stick_x) * TURN_SCALE;

        if (gamepad1.left_bumper) {
            drive(forward, right, rotate);
        } else {
            driveFieldRelative(forward, right, rotate);
        }

        // CRServo rotator: D-pad right = forward, D-pad left = reverse, D-pad down = stop
        if (gamepad1.dpad_right) {
            pollenRotator.setPower(1.0);
        } else if (gamepad1.dpad_left) {
            pollenRotator.setPower(-1.0);
        } else if (gamepad1.dpad_down) {
            pollenRotator.setPower(0.0);
        }
    }

    private double deadband(double value) {
        if (Math.abs(value) < DEADBAND) return 0;
        double scaled = (Math.abs(value) - DEADBAND) / (1.0 - DEADBAND);
        return Math.signum(value) * scaled * scaled;
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

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        double frontLeftPower  = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower  = forward + right - rotate;
        double backLeftPower   = forward - right + rotate;

        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setPower(Range.clip(MAX_SPEED * (frontLeftPower / maxPower), -1, 1));
        frontRightDrive.setPower(Range.clip(MAX_SPEED * (frontRightPower / maxPower), -1, 1));
        backLeftDrive.setPower(Range.clip(MAX_SPEED * (backLeftPower / maxPower), -1, 1));
        backRightDrive.setPower(Range.clip(MAX_SPEED * (backRightPower / maxPower), -1, 1));
    }
}