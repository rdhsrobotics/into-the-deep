package org.riverdell.robotics.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Test", group="Linear OpMode")
public class ServoTest extends LinearOpMode {
    private Servo pivotLeft = null;
    private Servo pivotRight = null;
    private Servo wrist = null;
    private Servo claw = null;


    @Override
    public void runOpMode()
    {
        pivotLeft = hardwareMap.get(Servo.class, "pivotLeft");
        pivotRight = hardwareMap.get(Servo.class, "pivotRight");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");

        while (opModeIsActive()) {
            if (gamepad1.dpad_down) {
                pivotRight.setPosition(pivotRight.getPosition() + 0.01);
                pivotLeft.setPosition(pivotLeft.getPosition() - 0.01);
            } else if (gamepad1.dpad_up) {
                pivotRight.setPosition(pivotRight.getPosition() - 0.01);
                pivotLeft.setPosition(pivotLeft.getPosition() + 0.01);
            }

            if (gamepad1.a || gamepad2.a) {
                claw.setPosition(0.5);
                // Rumble both gamepads
                gamepad1.rumble(0.5, 0.5, 100); // Left and right rumbling thing n the controller at full strength for 1 sec cuz why not
                gamepad2.rumble(0.5, 0.5, 100);
            } else {
                claw.setPosition(0.0);//grip of the claw
            }


            if (gamepad1.left_bumper) {
                wrist.setPosition(wrist.getPosition() - 0.01);
            } else if (gamepad1.right_bumper) {
                wrist.setPosition(wrist.getPosition() + 0.01);
            }




            telemetry.addData("Pivot Left", pivotLeft.getPosition());
            telemetry.addData("Pivot Right", pivotRight.getPosition());
            telemetry.addData("Wrist", wrist.getPosition());
            telemetry.update();
        }
    }
}