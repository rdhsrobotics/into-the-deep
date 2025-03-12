package org.riverdell.robotics.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.riverdell.robotics.autonomous.movement.geometry.Pose;


@TeleOp(name="BadWolf OpMode", group="Linear OpMode")
public class BadWolfOpMode extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Servo pivotRight = null;
    private Servo pivotLeft = null;
    private Servo claw = null;

    private final Pose robotPose = new Pose(0, 0, 0); // Initialize the robot's pose

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Sigma skibidi");
        telemetry.update();

        // Initialize hardware variables
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor liftRight = hardwareMap.get(DcMotor.class, "liftRight");
        DcMotor liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        pivotRight = hardwareMap.get(Servo.class, "pivotRight");
        pivotLeft = hardwareMap.get(Servo.class, "pivotLeft");
        claw = hardwareMap.get(Servo.class, "claw");
        // New servo variable
        Servo wrist = hardwareMap.get(Servo.class, "wrist"); // this init the new servo

        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftRight.setDirection(DcMotor.Direction.FORWARD);
        liftLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set initial servo positions
        pivotRight.setPosition(0.90); // Initial position for right elevator servo
        pivotLeft.setPosition(0.10);  // Initial position for left elevator servo
        claw.setPosition(0);      // Initial position for master claw
        wrist.setPosition(0.47);       // Initial position for claw rotation

        telemetry.addData("Status", "Init");
        telemetry.speak("Ready for Launch");

        telemetry.update();

        waitForStart();
        runtime.reset();

        // Set servo positions after game starts
        pivotRight.setPosition(0.7);
        pivotLeft.setPosition(0.3);
        wrist.setPosition(0.47);//for vertical samples and rest state

        while (opModeIsActive()) {

            robotPose.set(new Pose(1, 2, 3)); // Replace with actual pose constantly

            // Change speed multiplier based on right trigger
            // Speed multiplier with default value
            double speedMultiplier = (gamepad1.left_trigger > 0.1 ? 1 : 0.4);

            // Mecanum wheel drive calculations
            double drive = -gamepad1.left_stick_y; // Forward/Backward
            double strafe = gamepad1.left_stick_x; // Left/Right
            double turn = gamepad1.right_stick_x; // Turning

            if (gamepad2.left_stick_y != 0 || gamepad2.left_stick_x != 0) {
                drive = -gamepad2.left_stick_y * 0.4; // 0.4 power for all drive motors
                strafe = gamepad2.left_stick_x * 0.4;
                turn = 0; // No turning action for gamepad
                //this is so that there can be more fine action from player 2 than player 1 controlling the entire thing
            }

            // Calculate power for each wheel
            double frontLeftPower = Range.clip((drive + strafe + turn) * speedMultiplier, -1.0, 1.0);
            double frontRightPower = Range.clip((drive - strafe - turn) * speedMultiplier, -1.0, 1.0);
            double backLeftPower = Range.clip((drive - strafe + turn) * speedMultiplier, -1.0, 1.0);
            double backRightPower = Range.clip((drive + strafe - turn) * speedMultiplier, -1.0, 1.0);

            // Send calculated power to wheels
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Elevator control code
            int liftRightPosition = liftRight.getCurrentPosition();
            int liftLeftPosition = liftLeft.getCurrentPosition();

            if (gamepad1.right_bumper && liftRightPosition < 2500 && liftLeftPosition < 2500) {
                // Raise elevator and also tune for new Misumi and new ultra planetary gears.
                liftRight.setPower(1.0);
                liftLeft.setPower(1.0);
            } else if (gamepad1.left_bumper && liftRightPosition > 60 && liftLeftPosition > 60) {
                // Lower elevator
                liftRight.setPower(-1);
                liftLeft.setPower(-1);
            } else {
                liftRight.setPower(0);
                liftLeft.setPower(0);
            }

            // Claw rotation control with gamepad2 right joystick
            double clawIncrement = 0.01; // this how much the increment increases by.
            double rightStickX = gamepad2.right_stick_x;

            if (rightStickX > 0.1) {
                // rotate right1
                wrist.setPosition(Range.clip(wrist.getPosition() - clawIncrement, 0.0, 1.0));
            } else if (rightStickX < -0.1) {
                // rotates left
                wrist.setPosition(Range.clip(wrist.getPosition() + clawIncrement, 0.0, 1.0));
            }

            if (gamepad1.right_trigger > 0.2) { //scoring
                pivotRight.setPosition(0.75);
                pivotLeft.setPosition(0.25);
            } else if (gamepad2.right_trigger > 0.2) { //scoring
                pivotRight.setPosition(0.33);
                pivotLeft.setPosition(0.67);
            }


            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                wrist.setPosition(0.64);//diagonal left
            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                wrist.setPosition(0.27); // diagonal right
            } else if ((gamepad1.dpad_down || gamepad2.dpad_down)) {
                wrist.setPosition(0.47); //reset to legal point
            } else if ((gamepad1.dpad_up || gamepad2.dpad_up)) {
                wrist.setPosition(0.8); //Horizontal pickup point
            }

            if (gamepad1.a || gamepad2.a) {
                claw.setPosition(0.8);
                // Rumble both gamepads
                gamepad1.rumble(0.5, 0.5, 100); // Left and right rumbling thing n the controller at full strength for 1 sec cuz why not
                gamepad2.rumble(0.5, 0.5, 100);
            } else {
                claw.setPosition(0.0);//grip of the claw
            }

            if (gamepad1.b || gamepad2.b) {
                // reset everything and go to default position
                pivotRight.setPosition(0.7);
                pivotLeft.setPosition(0.3);
                wrist.setPosition(0.47);
                claw.setPosition(0.0);
            }

            if (gamepad1.y || gamepad2.y) {
                // Move servos to specific positions. This is the hover point
                pivotRight.setPosition(0.4);//real low to hover. Make higher to hover higher and make lower to hover lower
                pivotLeft.setPosition(0.6);//these two numbers should always add up to hundred. otherwise u are breaking the servos
            }

            if (gamepad1.x || gamepad2.x) {
                // Check if servos are in the correct positions for to perform a grab
                //so if y is pressed and then x is pressed it performs a grab.
                if (pivotRight.getPosition() == 0.4 && pivotLeft.getPosition() == 0.6) {
                    performGrab();
                }
            }

            // Telemetry data
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Motors", "frontLeft (%.2f), frontRight (%.2f)", frontLeftPower, frontRightPower);
//            telemetry.addData("Motors", "backLeft (%.2f), backRight (%.2f)", backLeftPower, backRightPower);
            telemetry.addData("/n/nAccuracy Mode Speed", speedMultiplier);
            telemetry.addData("/n/nElevator Position", "Right: %d, Left: %d", liftRightPosition, liftLeftPosition);
            telemetry.addData("/nRotate Position", wrist.getPosition());
            telemetry.addData("/nPose", robotPose.toString());
            telemetry.update();
        }
    }

    private void performGrab() {
        ElapsedTime timer = new ElapsedTime();

        // Open claw to position 0.45
        claw.setPosition(0.9);
        timer.reset();
        while (timer.seconds() < 0.05 && opModeIsActive()) {
            // Wait for 0.1 seconds
            telemetry.addData("Grab Step", "Opening Claw: %.2f", timer.seconds());
            telemetry.update();
        }

        // Move servos to new positions
        pivotRight.setPosition(0.33);
        pivotLeft.setPosition(0.67);
        timer.reset();
        while (timer.seconds() < 0.1 && opModeIsActive()) {
            // Wait for 0.5 second
            telemetry.addData("Grab Step", "Moving Servos: %.2f", timer.seconds());
            telemetry.update();
        }

        // Close claw to position 0
        claw.setPosition(0);

        // Wait until the claw is closed
        while (Math.abs(claw.getPosition() - 0) > 0.2 && opModeIsActive()) {
            telemetry.addData("Grab Step", "Closing Claw");
            telemetry.update();
        }

        // Wait for 0.3 seconds before setting servos
        timer.reset();
        while (timer.seconds() < 0.15     && opModeIsActive()) {
            telemetry.addData("Grab Step", "Waiting before setting servos: %.2f", timer.seconds());
            telemetry.update();
        }

        // Set right and left servo positions a little higher than hover to move back from submersible
        pivotRight.setPosition(0.45);
        pivotLeft.setPosition(0.55);
        claw.setPosition(0);
    }
}