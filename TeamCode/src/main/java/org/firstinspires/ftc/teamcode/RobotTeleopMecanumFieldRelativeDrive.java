package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
public class RobotTeleopMecanumFieldRelativeDrive extends OpMode {

    static final double DEADBAND = 0.05;
    static final double MAX_SPEED = 1.0;
    static final double TURN_SCALE = 0.80;

    private boolean lastButtonState = false;
    private boolean motorOn = false;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor intakeMotor;

    GoBildaPinpointDriver odo;

    @Override
    public void init() {
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "fld");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frd");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "bld");
        backRightDrive  = hardwareMap.get(DcMotor.class, "brd");
        intakeMotor     = hardwareMap.get(DcMotor.class, "Intake");

        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        telemetry.addLine("Pinpoint initialized - keep the robot still");
        telemetry.update();
    }

    @Override
    public void loop() {
        odo.update();

        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction and speed");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addData("Heading (deg)", "%.1f",
                odo.getPosition().getHeading(AngleUnit.DEGREES));

        if (gamepad1.a) {
            resetYaw();
        }

        boolean currentButtonState = gamepad1.b;
        if (currentButtonState && !lastButtonState) {
            motorOn = !motorOn;
            intakeMotor.setPower(motorOn ? 1.0 : 0.0);
        }
        lastButtonState = currentButtonState;

        double forward = deadband(-gamepad1.left_stick_y);
        double right   = deadband(gamepad1.left_stick_x);
        double rotate  = deadband(gamepad1.right_stick_x) * TURN_SCALE;

        if (gamepad1.left_bumper) {
            drive(forward, right, rotate);
        } else {
            driveFieldRelative(forward, right, rotate);
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