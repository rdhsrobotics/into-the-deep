package org.riverdell.robotics.autonomous.movement;

import static org.riverdell.robotics.autonomous.movement.PositionChangeAction.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.riverdell.robotics.autonomous.HypnoticAuto;
import org.riverdell.robotics.autonomous.movement.geometry.Pose;

@Config
public class MecanumTranslations {

    public static FrictionCompensationFunctionParameters STRAIGHT = new FrictionCompensationFunctionParameters(1.33, 0.068, 0.2, 3.45);
    public static FrictionCompensationFunctionParameters STRAFE = new FrictionCompensationFunctionParameters(1.33, 0.068, 0.15, 3.45);
    public static FrictionCompensationFunctionParameters TURN = new FrictionCompensationFunctionParameters(0.83, 0.01, 0.3, 100.0);

    private static double frictionCompensationFunction(FrictionCompensationFunctionParameters parameters, double v) {
        return -parameters.m * Math.signum(v) * (1 / (parameters.a * parameters.sharpness * Math.pow(v, 4) + 1) + (parameters.F_d * parameters.sharpness * Math.pow(v, 2) - 1) / (parameters.sharpness * Math.pow(v, 2) + 1));
    }

    public static DrivetrainUpdates getPowers(double strafePower, double straightPower,
                                              double turnPower, double straightVel, double strafeVel, double turnVel) {


        Vector2d input = new Vector2d(strafePower, straightPower);//.rotateBy(-gyroAngle);

        strafePower = Range.clip(input.getX(), -1, 1);
        straightPower = Range.clip(input.getY(), -1, 1);
        turnPower = Range.clip(turnPower, -1, 1);

        straightPower += MecanumTranslations.frictionCompensationFunction(STRAIGHT, straightVel) * STRAIGHT_POWER_BOOST;
        strafePower += MecanumTranslations.frictionCompensationFunction(STRAFE, strafeVel) * STRAFE_POWER_BOOST;
        turnPower += MecanumTranslations.frictionCompensationFunction(TURN, turnVel) * TURN_POWER_BOOST;

        double[] wheelSpeeds = new double[4];

        wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] = straightPower + strafePower + turnPower;
        wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] = straightPower - strafePower - turnPower;
        wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] = (straightPower - strafePower + turnPower);
        wheelSpeeds[RobotDrive.MotorType.kBackRight.value] = (straightPower + strafePower - turnPower);
        // 1.06, 1.04

        // feedforward & voltage comp
        double correction = 13 / HypnoticAuto.getInstance().getRobot().getDrivetrain().voltage();
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = wheelSpeeds[i] * correction;
        }

        // Normalize values
        double max = 1;
        for (double wheelSpeed : wheelSpeeds) max = Math.max(max, Math.abs(wheelSpeed));

        if (max > 1) {
            wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] /= max;
            wheelSpeeds[RobotDrive.MotorType.kBackRight.value] /= max;
        }

        return new DrivetrainUpdates(
                wheelSpeeds[0],
                wheelSpeeds[1],
                wheelSpeeds[2],
                wheelSpeeds[3]
        );
    }

    public static DrivetrainUpdates getPowers(Pose pose, double straightVel, double strafeVel, double turnVel) {
        return getPowers(pose.x, pose.y, pose.getHeading(), straightVel, strafeVel, turnVel);
    }
}
