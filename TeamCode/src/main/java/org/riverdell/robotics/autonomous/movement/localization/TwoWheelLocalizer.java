package org.riverdell.robotics.autonomous.movement.localization;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.riverdell.robotics.HypnoticRobot;
import org.riverdell.robotics.autonomous.movement.geometry.Pose;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer {

    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.94488189; // 48
    public static double GEAR_RATIO = 1;

    /**
     * The System.nanoTime() of the latest wheel position
     */
    public static double translateAcquisitionTime = 0.0;

    private final HypnoticRobot hypnoticRobot;

    private final Supplier<Integer> lateral;
    private final Supplier<Integer> perpendicular;

    public TwoWheelLocalizer(HypnoticRobot hypnoticRobot) {
        super(Arrays.asList(
                new Pose2d(0.0, 0, 0), // left + right TODO TUNE X VALUE
                new Pose2d(0.0, 0, Math.toRadians(90)) // front TODO TUNE X VALUE
        ));

        this.hypnoticRobot = hypnoticRobot;
        lateral = hypnoticRobot.getHardware().getFrontLeft()::getCurrentPosition;
        perpendicular = hypnoticRobot.getHardware().getBackLeft()::getCurrentPosition;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        double lateralPos = lateral.get();
        double perpPos = perpendicular.get();
        translateAcquisitionTime = System.nanoTime();
        return Arrays.asList(
                encoderTicksToInches(lateralPos),
                encoderTicksToInches(perpPos)
        );
    }

    @Override
    public double getHeading() {
        return hypnoticRobot.getDrivetrain().imu().getYaw(AngleUnit.RADIANS);
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(0.0, 0.0);
    }

    public Pose getPose() {
        Pose2d pose = getPoseEstimate();
        return new Pose(pose.getY(), pose.getX(), pose.getHeading());
    }
}