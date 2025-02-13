package org.riverdell.robotics.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Servo Test", group = "Linear OpMode")
public class ServoTest extends LinearOpMode {
    private Servo pivotLeft = null;
    private Servo pivotRight = null;
    private Servo wrist = null;
    private Servo claw = null;

    @Override
    public void runOpMode() {
        // Initialize servos
        pivotLeft = hardwareMap.get(Servo.class, "pivotLeft");
        if (pivotLeft == null) {
            telemetry.addData("Error", "pivotLeft servo not found in hardware map");
            telemetry.update();
            return;
        }
        pivotRight = hardwareMap.get(Servo.class, "pivotRight");
        if (pivotRight == null) {
            telemetry.addData("Error", "pivotRight servo not found in hardware map");
            telemetry.update();
            return;
        }
        claw = hardwareMap.get(Servo.class, "claw");
        if (claw == null) {
            telemetry.addData("Error", "claw servo not found in hardware map");
            telemetry.update();
            return;
        }
        wrist = hardwareMap.get(Servo.class, "wrist");
        if (wrist == null) {
            telemetry.addData("Error", "wrist servo not found in hardware map");
            telemetry.update();
            return;
        }

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            // Control pivot servos
            if (gamepad1.dpad_down) {
                pivotRight.setPosition(Range.clip(pivotRight.getPosition() + 0.01, 0.0, 1.0));
                pivotLeft.setPosition(Range.clip(pivotLeft.getPosition() - 0.01, 0.0, 1.0));
            } else if (gamepad1.dpad_up) {
                pivotRight.setPosition(Range.clip(pivotRight.getPosition() - 0.01, 0.0, 1.0));
                pivotLeft.setPosition(Range.clip(pivotLeft.getPosition() + 0.01, 0.0, 1.0));
            }

            // Control claw servo
            if (gamepad1.a || (gamepad2 != null && gamepad2.a)) {
                claw.setPosition(0.5);
                // Rumble both gamepads
                gamepad1.rumble(0.5, 0.5, 100); // Left and right rumbling thing n the controller at full strength for 1 sec cuz why not
                if (gamepad2 != null) {
                    gamepad2.rumble(0.5, 0.5, 100);
                }
            } else {
                claw.setPosition(0.0); // grip of the claw
            }

            // Control wrist servo
            if (gamepad1.left_bumper) {
                wrist.setPosition(Range.clip(wrist.getPosition() - 0.01, 0.0, 1.0));
            } else if (gamepad1.right_bumper) {
                wrist.setPosition(Range.clip(wrist.getPosition() + 0.01, 0.0, 1.0));
            }

            // Telemetry data
            telemetry.addData("\n\nPivot Left", pivotLeft.getPosition());
            telemetry.addData("\n\nPivot Right", pivotRight.getPosition());
            telemetry.addData("\n\nWrist", wrist.getPosition());
            telemetry.update();
        }
    }
}