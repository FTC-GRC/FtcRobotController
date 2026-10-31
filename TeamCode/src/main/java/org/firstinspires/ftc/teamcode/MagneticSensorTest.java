package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "MST", group = "Sensor")
public class MagneticSensorTest extends LinearOpMode {
    private DigitalChannel MS;

    @Override
    public void runOpMode() {
        MS = hardwareMap.get(DigitalChannel.class, "MS");
        MS.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean isPressed = MS.getState();
            telemetry.addData("Magnet Detected", !isPressed);
            telemetry.addData("Raw Sensor State", isPressed);
            telemetry.update();
            boolean rawState = MS.getState();
            int rawNumeric = rawState ? 1 : 0;
            telemetry.addData("1. Raw Boolean State", rawState);
            telemetry.addData("2. Raw Numeric State", rawNumeric);
            telemetry.addData("3. Magnet Target Detected", !rawState);
        }
    }
}
