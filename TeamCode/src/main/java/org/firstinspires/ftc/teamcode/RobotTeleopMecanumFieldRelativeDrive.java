package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
public class RobotTeleopMecanumFieldRelativeDrive extends OpMode {

    private boolean lastButtonState = false;
    private boolean motorOn = false;

    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor intakeMotor;

    // The Pinpoint replaces the IMU as the heading source
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

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Offset of each pod from the robot's center of rotation.
        // X = forward/back offset of the X (forward) pod, Y = left/right offset of the Y (strafe) pod.
        // Measure these on your robot. Left and forward are positive.
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);

        // Pick the pod you actually have
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Flip these if a pod counts the wrong way
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Calibrates the internal IMU - the robot must be completely still here
        odo.resetPosAndIMU();

        telemetry.addLine("Pinpoint initialized - keep the robot still");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Must be called once per loop or the heading never changes
        odo.update();

        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addData("Heading (deg)", odo.getPosition().getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pinpoint status", odo.getDeviceStatus());

        if (gamepad1.a) {
            resetYaw();
        }

        boolean currentButtonState = gamepad1.b;
        if (currentButtonState && !lastButtonState) {
            motorOn = !motorOn;
            intakeMotor.setPower(motorOn ? 1.0 : 0.0);
        }
        lastButtonState = currentButtonState;

        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }

    // Zeroes the heading without recalibrating the IMU, so it is instant and safe mid-match
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

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        double frontLeftPower  = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower  = forward + right - rotate;
        double backLeftPower   = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
