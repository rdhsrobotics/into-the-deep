package org.riverdell.robotics.autonomous.movement;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.riverdell.robotics.autonomous.HypnoticAuto;
import org.riverdell.robotics.autonomous.movement.geometry.Point;
import org.riverdell.robotics.autonomous.movement.geometry.Pose;
import org.riverdell.robotics.autonomous.movement.localization.TwoWheelLocalizer;

import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicInteger;

import io.liftgate.robotics.mono.Mono;
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup;
import kotlin.Unit;
import kotlin.jvm.functions.Function1;

@Config
public class PositionChangeAction {
    public static final DrivetrainUpdates ZERO = new DrivetrainUpdates(0.0, 0.0, 0.0, 0.0);

    private final HypnoticAuto instance;
    private final RootExecutionGroup executionGroup;

    public static PIDCoefficients strafePID = new PIDCoefficients(0.18, 0.0, 0.024);
    public static PIDCoefficients straightPID = new PIDCoefficients(0.077, 0.0, 0.013);
    public static PIDCoefficients headingPID = new PIDCoefficients(1.1, 0.0, -0.125);

    public static PIDCoefficients extendoOutHeadingPID = new PIDCoefficients(1.08, 0.0, -0.19);

    public static double TURN_POWER_BOOST = -0.05;
    public static double STRAFE_POWER_BOOST = 0.045;
    public static double STRAIGHT_POWER_BOOST = 0.03;

    public static double TURN_PREDICTION_ACCELERATION = 0;
    public static double TURN_PREDICTION_VELOCITY = 0;
    public static double TURN_MAX_VELOCITY = 5;

    public double maxTranslationalError = 1.1;
    public double maxHeadingErrorRad = 1.4 * Math.PI / 180;
    public double maxTranslationalVelocity = 2.5;
    public double maxHeadingVelocity = 30;
    private final double atTargetMillis = 50;

    public PIDFController strafeController;
    public PIDFController straightController;
    public PIDFController hController;

    private final ElapsedTime activeTimer = new ElapsedTime();
    private final ElapsedTime stuckTime = new ElapsedTime();
    private final ElapsedTime atTargetTimer = new ElapsedTime();

    private final LinkedList<double[]> prevHPowers = new LinkedList<>();
    private double prevHPower = 0.0;

    private final @Nullable Pose targetPose;

    /**
     * Preferred to use stuck protection in this case...
     */
    private @Nullable Double automaticDeathMillis = 3500.0;

    private double maxTranslationalPower = 1.0;
    private double maxRotationalPower = 1.0;

    private double periodNanos = 0.0;
    private double prevTime = 0.0;
    private @Nullable Pose previousPose = null;
    private Pose velocity = new Pose();
    private double translatePosAcquisitionTime = System.nanoTime();
    private double headingAcquisitionTime = System.nanoTime();
    private double currentTimeNanos = System.nanoTime();

    public PositionChangeAction(@Nullable Pose targetPose, @NotNull RootExecutionGroup executionGroup) {
        this.instance = HypnoticAuto.getInstance();
        this.targetPose = targetPose;
        this.executionGroup = executionGroup;

        strafeController = new PIDFController(strafePID, (current, target, vel) -> 0.0);
        straightController = new PIDFController(straightPID, (current, target, vel) -> 0.0);
        hController = new PIDFController(headingPID, (current, target, vel) -> 0.0);

        strafeController.setTolerance(maxTranslationalError, maxTranslationalVelocity);
        straightController.setTolerance(maxTranslationalError, maxTranslationalVelocity);
        hController.setTolerance(maxHeadingErrorRad, maxHeadingVelocity);
    }

    private @Nullable PathAlgorithm pathAlgorithm = null;
    private @Nullable Function1<PositionChangeActionEndResult, Unit> endSubscription = null;

    public void executeBlocking() {
        executeBlocking(false);
    }

    public void executeBlocking(boolean test) {
        while (true) {
            currentTimeNanos = System.nanoTime();

            if (prevTime == 0) { prevTime = System.nanoTime(); }
            periodNanos = currentTimeNanos - prevTime;
            prevTime = currentTimeNanos;

            if (instance.isStopRequested()) {
                executionGroup.terminateMidExecution();
                finish(PositionChangeActionEndResult.ForcefulTermination);
                return;
            }

            Pose robotPose = instance.getRobot().getDrivetrain().getLocalizer().getPose();
            if (previousPose == null) { previousPose = robotPose; }

            if (Math.abs(translatePosAcquisitionTime - TwoWheelLocalizer.translateAcquisitionTime) > 10) {
                double translatePosPeriod = Math.max( 1.0 / 1E3, (TwoWheelLocalizer.translateAcquisitionTime - translatePosAcquisitionTime) / 1E9);
                double deltaX = robotPose.x - previousPose.x;
                double deltaY = robotPose.y - previousPose.y;
                velocity.x = deltaX / translatePosPeriod;
                velocity.y = deltaY / translatePosPeriod;
                previousPose.x = robotPose.x;
                previousPose.y = robotPose.y;
                translatePosAcquisitionTime = TwoWheelLocalizer.translateAcquisitionTime;
            }

            double newHeadingAcquisitionTime = instance.getRobot().getDrivetrain().imu().getAcquisitionTime();
            if (Math.abs(headingAcquisitionTime - newHeadingAcquisitionTime) > 10) { // I2C register only updates at 100 Hz max
                double headingPeriod = Math.max(1.0 / 100, (newHeadingAcquisitionTime - headingAcquisitionTime) / 1E9);
                double deltaHeading = robotPose.getHeading() - previousPose.getHeading();
                if (deltaHeading > Math.PI) deltaHeading -= 2 * Math.PI;
                if (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;
                velocity.setHeading(deltaHeading / headingPeriod);
                previousPose.setHeading(robotPose.getHeading());
                headingAcquisitionTime = newHeadingAcquisitionTime;
            }

            Pose targetPose = this.pathAlgorithm == null ? this.targetPose :
                    pathAlgorithm.getTargetCompute().invoke(robotPose);

            assert targetPose != null;
            Pose powers = getPowers(robotPose, targetPose, velocity);
            if (!test) {
                HypnoticAuto.setNextUpdates(MecanumTranslations.getPowers(powers,
                        straightController.getVelocityError(),
                        strafeController.getVelocityError(),
                        hController.getVelocityError()
                ));
            }

            PositionChangeActionEndResult result = getState(robotPose, targetPose, velocity);
            if (result != null && !test) {
                finish(result);
                return;
            }
        }
    }

    public double predictHeading(double lastPeriodMillis, double imuLatencyNanos, double headingMeasurement, double initialHeadingVelocity) {
        double estimatedVelocity = initialHeadingVelocity + TURN_PREDICTION_ACCELERATION * (instance.robot.getDrivetrain().voltage() * prevHPower - (13.0 / TURN_MAX_VELOCITY) * initialHeadingVelocity) * lastPeriodMillis / 2000;
        return headingMeasurement + Range.clip(TURN_PREDICTION_VELOCITY * estimatedVelocity * imuLatencyNanos / 1E9, -TURN_MAX_VELOCITY, TURN_MAX_VELOCITY);
    }

    public @NotNull Pose getPowers(@NotNull Pose robotPose, @NotNull Pose targetPose, Pose velocity) {
        double targetHeading = targetPose.getHeading();
        double imuLatencyNanos = currentTimeNanos - headingAcquisitionTime;
        double robotHeading = robotPose.getHeading();
        double headingError = targetHeading - robotHeading;

        if (headingError > Math.PI) headingError -= 2 * Math.PI;
        if (headingError < -Math.PI) headingError += 2 * Math.PI;

        Point robotCentricTranslationalError = targetPose.subtract(robotPose).rotate(-robotHeading - Math.PI / 2);
        Point robotCentricTranslationalVelocity = velocity.rotate(-robotHeading - Math.PI / 2);

        double xPower = strafeController.calculate(
                robotCentricTranslationalError.x, 0, robotCentricTranslationalVelocity.x);
        double yPower = straightController.calculate(
                robotCentricTranslationalError.y, 0, robotCentricTranslationalVelocity.y);
        double hPower = hController.calculate(-headingError, 0, velocity.getHeading());

        hPower = Range.clip(hPower, -maxRotationalPower, maxRotationalPower);
        prevHPower = hPower;
        xPower = Range.clip(xPower, -maxTranslationalPower, maxTranslationalPower);
        yPower = Range.clip(yPower, -maxTranslationalPower, maxTranslationalPower);

        return new Pose(xPower, yPower, hPower);
    }

    public void updateTelemetry(double imuLatencyNanos, double robotHeading) {
        this.instance.getRobot().getMultipleTelemetry().addData(
                "X Position Error",
                strafeController.getPositionError()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "X Velocity Error",
                strafeController.getVelocityError()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "Y Position Error",
                straightController.getPositionError()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "Y Velocity Error",
                straightController.getVelocityError()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "Heading",
                Math.toDegrees(hController.getPositionError())
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "Heading Velocity Error",
                Math.toDegrees(hController.getVelocityError())
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "Period Milliseconds",
                periodNanos / 1E6
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "IMU Latency ms",
                imuLatencyNanos / 1E6
        );
        this.instance.getRobot().getMultipleTelemetry().update();
    }

    private @Nullable RobotStuckProtection robotStuckProtection = null;

    public void withStuckProtection(@NotNull RobotStuckProtection stuckProtection) {
        disableAutomaticDeath();
        this.robotStuckProtection = stuckProtection;
    }

    private @Nullable PositionChangeActionEndResult getState(Pose currentPose, Pose targetPose, Pose velocity) {
        if (currentPose == null || targetPose == null) {
            return PositionChangeActionEndResult.LocalizationFailure;
        }
        if (Double.isNaN(currentPose.getHeading()) || Double.isNaN(targetPose.getHeading())) {
            return PositionChangeActionEndResult.LocalizationFailure;
        }

        if (automaticDeathMillis != null) {
            if (activeTimer.milliseconds() > automaticDeathMillis) {
                return PositionChangeActionEndResult.ExceededTimeout;
            }
        }

        if (robotStuckProtection != null) {
            if (velocity != null) {
                if (velocity.radius() > robotStuckProtection.getMinimumRequiredTranslationalDifference() ||
                        Math.abs(velocity.getHeading()) > robotStuckProtection.getMinimumRequiredRotationalDifference()) {
                    stuckTime.reset();
                }

                if (stuckTime.milliseconds() > robotStuckProtection.getMinimumMillisUntilDeemedStuck()) {
                    return PositionChangeActionEndResult.StuckDetected;
                }
            } else {
                stuckTime.reset();
            }
        }

        if (pathAlgorithm != null && !pathAlgorithm.getEuclideanCompletionCheck()) {
            if (pathAlgorithm.getPathComplete().invoke(currentPose, targetPose)) {
                return PositionChangeActionEndResult.PathAlgorithmSuccessful;
            } else if (pathAlgorithm.getStrict()) {
                return null;
            }
        }

        if (!(strafeController.atSetPoint() && straightController.atSetPoint() && hController.atSetPoint())) {
            atTargetTimer.reset();
        }

        if (atTargetTimer.milliseconds() > atTargetMillis) {
            return PositionChangeActionEndResult.Successful;
        }

        return null;
    }

    protected void finish(@NotNull PositionChangeActionEndResult result) {
        if (result != PositionChangeActionEndResult.ForcefulTermination) {
            HypnoticAuto.sendZeroCommand();

            if (endSubscription != null) {
                endSubscription.invoke(result);
            }
        }

        Mono.INSTANCE.getLogSink().invoke("[position] ended with result of " + result.name());
    }

    public void withEndSubscription(@NotNull Function1<PositionChangeActionEndResult, Unit> subscription) {
        this.endSubscription = subscription;
    }

    public void whenStuck(@NotNull RobotStuckProtection stuckProtection, @NotNull Runnable stuck) {
        withStuckProtection(stuckProtection);
        withEndSubscription(positionChangeActionEndResult -> {
            if (positionChangeActionEndResult == PositionChangeActionEndResult.StuckDetected) {
                stuck.run();
            }
            return null;
        });
    }

    public void whenStuckWithMaxAttempts(@NotNull RobotStuckProtection stuckProtection, AtomicInteger accumulator, int maxAttempts, @NotNull Runnable stuck) {
        disableAutomaticDeath();
        withStuckProtection(stuckProtection);
        withEndSubscription(positionChangeActionEndResult -> {
            if (positionChangeActionEndResult == PositionChangeActionEndResult.StuckDetected) {
                if (accumulator.getAndIncrement() < maxAttempts) {
                    stuck.run();
                } else {
                    executionGroup.terminateMidExecution();
                    return null;
                }
            }
            return null;
        });
    }

    public void whenTimeOutExceeded(@NotNull Runnable timeoutExceed) {
        withEndSubscription(positionChangeActionEndResult -> {
            if (positionChangeActionEndResult == PositionChangeActionEndResult.ExceededTimeout) {
                timeoutExceed.run();
            }
            return null;
        });
    }

    public void whenStuck(@NotNull Runnable stuck) {
        withEndSubscription(positionChangeActionEndResult -> {
            if (positionChangeActionEndResult == PositionChangeActionEndResult.StuckDetected) {
                stuck.run();
            }
            return null;
        });
    }

    public void withCustomMaxRotationalSpeed(double maxRotationalSpeed) {
        this.maxRotationalPower = maxRotationalSpeed;
    }

    public void withCustomHeadingTolerance(double headingToleranceDeg) {
        this.maxHeadingErrorRad = headingToleranceDeg * Math.PI / 180;
    }

    public void withCustomTranslationalTolerance(double translationalTolerance) {
        this.maxTranslationalError = translationalTolerance;
    }

    public void withCustomMaxTranslationalSpeed(double maxTranslationalSpeed) {
        this.maxTranslationalPower = maxTranslationalSpeed;
    }

    public void withCustomPathAlgorithm(@NotNull PathAlgorithm pathAlgorithm) {
        this.pathAlgorithm = pathAlgorithm;
    }

    public void withAutomaticDeath(double automaticDeathMillis) {
        this.automaticDeathMillis = automaticDeathMillis;
    }

    public void withCustomHeadingPID(double p, double i, double d) {
        hController = new PIDFController(new PIDCoefficients(p, i, d), (current, target, vel) -> 0.0);
    }

    public void withCustomHeadingPID(PIDCoefficients PID) {
        hController = new PIDFController(PID, (current, target, vel) -> 0.0);
    }

    public void withExtendoOut(boolean use) {
        if (use) {
            withCustomHeadingPID(extendoOutHeadingPID);
        }
    }

    public void disableAutomaticDeath() {
        this.automaticDeathMillis = null;
    }
}
