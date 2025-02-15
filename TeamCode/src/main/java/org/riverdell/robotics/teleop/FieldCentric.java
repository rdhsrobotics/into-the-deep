package org.riverdell.robotics.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Field Centric OpMode", group = "Linear OpMode")
public class FieldCentric extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private Servo pivotRight, pivotLeft, claw, wrist;
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Field-Centric Mode...");
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
        wrist = hardwareMap.get(Servo.class, "wrist");

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setDirection(DcMotor.Direction.FORWARD);
        liftLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set initial servo positions
        pivotRight.setPosition(0.63); // Initial position for right elevator servo
        pivotLeft.setPosition(0.37);  // Initial position for left elevator servo
        claw.setPosition(0);          // Initial position for master claw
        wrist.setPosition(0.47);      // Initial position for claw rotation

        telemetry.addData("Status", "Field-Centric Mode Ready!");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Get robot heading from IMU
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = angles.firstAngle;

            // Joystick inputs
            double drive = -gamepad1.left_stick_y; // Forward/Backward
            double strafe = gamepad1.left_stick_x; // Left/Right
            double turn = gamepad1.right_stick_x;  // Turning

            // Adjust for field-centric driving
            double angle = Math.toDegrees(Math.atan2(drive, strafe)) - heading;
            double magnitude = Math.sqrt(Math.pow(drive, 2) + Math.pow(strafe, 2));

            // Calculate motor powers
            double v1 = magnitude * Math.cos(Math.toRadians(angle + 45)) + turn;
            double v2 = magnitude * Math.sin(Math.toRadians(angle + 45)) + turn;
            double v3 = magnitude * Math.sin(Math.toRadians(angle - 45)) - turn;
            double v4 = magnitude * Math.cos(Math.toRadians(angle - 45)) - turn;

            // Scale powers to ensure they are within [-1, 1]
            double[] powers = {v1, v2, v3, v4};
            scale(powers);

            // Apply powers to motors
            frontLeft.setPower(powers[0]);
            frontRight.setPower(powers[1]);
            backLeft.setPower(powers[2]);
            backRight.setPower(powers[3]);

            // Elevator control code
            int liftRightPosition = liftRight.getCurrentPosition();
            int liftLeftPosition = liftLeft.getCurrentPosition();
            if (gamepad1.right_bumper && liftRightPosition < 2700 && liftLeftPosition < 2700) {
                liftRight.setPower(1.0);
                liftLeft.setPower(1.0);
            } else if (gamepad1.left_bumper && liftRightPosition > 70 && liftLeftPosition > 70) {
                liftRight.setPower(-0.9);
                liftLeft.setPower(-0.9);
            } else {
                liftRight.setPower(0);
                liftLeft.setPower(0);
            }

            // Claw and wrist controls remain unchanged
            double clawIncrement = 0.01;
            double rightStickX = gamepad2.right_stick_x;
            if (rightStickX > 0.1) {
                wrist.setPosition(Range.clip(wrist.getPosition() - clawIncrement, 0.0, 1.0));
            } else if (rightStickX < -0.1) {
                wrist.setPosition(Range.clip(wrist.getPosition() + clawIncrement, 0.0, 1.0));
            }

            if (gamepad1.right_trigger > 0.2) {
                pivotRight.setPosition(0.5);
                pivotLeft.setPosition(0.5);
            } else if (gamepad2.right_trigger > 0.2) {
                pivotRight.setPosition(0);
                pivotLeft.setPosition(1);
            }

            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                wrist.setPosition(0.64);
            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                wrist.setPosition(0.27);
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                wrist.setPosition(0.47);
            } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
                wrist.setPosition(0.8);
            }

            if (gamepad1.a || gamepad2.a) {
                claw.setPosition(1);
                gamepad1.rumble(0.5, 0.5, 100);
                gamepad2.rumble(0.5, 0.5, 100);
            } else {
                claw.setPosition(0.0);
            }

            if (gamepad1.b || gamepad2.b) {
                pivotRight.setPosition(0.45);
                pivotLeft.setPosition(0.55);
                wrist.setPosition(0.47);
                claw.setPosition(0.0);
            }

            if (gamepad1.y || gamepad2.y) {
                pivotRight.setPosition(0.16);
                pivotLeft.setPosition(0.84);
            }

            if (gamepad1.x || gamepad2.x) {
                if (pivotRight.getPosition() == 0.16 && pivotLeft.getPosition() == 0.84) {
                    performGrab();
                }
            }

            // Telemetry data
            telemetry.addData("Heading", heading);
            telemetry.addData("Elevator Position", "Right: %d, Left: %d", liftRightPosition, liftLeftPosition);
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.update();
        }
    }

    private void scale(double[] powers) {
        double maxPower = 0;
        for (double power : powers) {
            maxPower = Math.max(maxPower, Math.abs(power));
        }
        if (maxPower > 1) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= maxPower;
            }
        }
    }

    private void performGrab() {
        ElapsedTime timer = new ElapsedTime();
        claw.setPosition(0.9);
        timer.reset();
        while (timer.seconds() < 0.05 && opModeIsActive()) {
            telemetry.addData("Grab Step", "Opening Claw: %.2f", timer.seconds());
            telemetry.update();
        }

        pivotRight.setPosition(0.0);
        pivotLeft.setPosition(1.0);
        timer.reset();
        while (timer.seconds() < 0.2 && opModeIsActive()) {
            telemetry.addData("Grab Step", "Moving Servos: %.2f", timer.seconds());
            telemetry.update();
        }

        claw.setPosition(0);
        while (Math.abs(claw.getPosition() - 0) > 0.01 && opModeIsActive()) {
            telemetry.addData("Grab Step", "Closing Claw");
            telemetry.update();
        }

        timer.reset();
        while (timer.seconds() < 0.1 && opModeIsActive()) {
            telemetry.addData("Grab Step", "Waiting before setting servos: %.2f", timer.seconds());
            telemetry.update();
        }

        pivotRight.setPosition(0.20);
        pivotLeft.setPosition(0.80);
        claw.setPosition(0);
    }
}