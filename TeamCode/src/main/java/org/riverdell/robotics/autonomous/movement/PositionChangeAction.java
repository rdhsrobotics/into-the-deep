package org.riverdell.robotics.autonomous.movement;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.riverdell.robotics.autonomous.HypnoticAuto;
import org.riverdell.robotics.autonomous.movement.geometry.Point;
import org.riverdell.robotics.autonomous.movement.geometry.Pose;

import java.util.LinkedList;
import java.util.concurrent.atomic.AtomicInteger;

import io.liftgate.robotics.mono.Mono;
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup;
import kotlin.Unit;
import kotlin.jvm.functions.Function1;

@Config
public class PositionChangeAction {
    private static final DrivetrainUpdates ZERO = new DrivetrainUpdates(0.0, 0.0, 0.0, 0.0);

    private final HypnoticAuto instance;
    private final RootExecutionGroup executionGroup;

    public static double strafeP = 0.12;
    public static double strafeD = 0.012;

    public static double straightP = 0.075;
    public static double straightD = 0.01;

    public static double hP = 0.95; // 1.25 for extendo in
    public static double hI = 0.0;
    public static double hD = 0.19; // 0.16 for extendo in

    public static double TURN_POWER_BOOST = 0.06;
    public static double STRAFE_POWER_BOOST = 0.18;
    public static double STRAIGHT_POWER_BOOST = 0.03;

    public static double TURN_PREDICTION_ACCELERATION = 0.00025;
    public static double TURN_PREDICTION_VELOCITY = 6.0;
    public static double TURN_MAX_VELOCITY = 5;

    public double maxTranslationalError = 0.7;
    public double maxHeadingError = 1.5;

    public static PIDFController strafeController;
    public static PIDFController straightController;
    public static PIDFController hController;

    private final ElapsedTime activeTimer = new ElapsedTime();
    private final ElapsedTime stuckTime = new ElapsedTime();

    private final LinkedList<double[]> prevHPowers = new LinkedList<>();
    private double prevHPower = 0.0;

    private final @Nullable Pose targetPose;

    /**
     * Preferred to use stuck protection in this case...
     */
    private @Nullable Double automaticDeathMillis = 2500.0;

    private double maxTranslationalSpeed = 1.0;
    private double maxRotationalSpeed = 1.0;

    private double period = 0.0;
    private double prevTime = 0.0;

    public PositionChangeAction(@Nullable Pose targetPose, @NotNull RootExecutionGroup executionGroup) {
        this.instance = HypnoticAuto.getInstance();
        this.targetPose = targetPose;
        this.executionGroup = executionGroup;

        strafeController = new PIDFController(strafeP, 0.0, strafeD, (target, vel) -> 0.0);
        straightController = new PIDFController(straightP, 0.0, straightD, (target, vel) -> 0.0);
        hController = new PIDFController(hP, hI, hD, (target, vel) -> 0.0);

        strafeController.setTolerance(maxTranslationalError, maxTranslationalSpeed);
    }

    private @Nullable PathAlgorithm pathAlgorithm = null;
    private @Nullable Function1<PositionChangeActionEndResult, Unit> endSubscription = null;

    private @Nullable Pose previousPose = null;
    private @Nullable Pose velocity = null;

    public void executeBlocking() {
        while (true) {
            if (prevTime == 0) { prevTime = System.nanoTime(); }
            period = System.nanoTime() - prevTime;
            prevTime = System.nanoTime();


            if (instance.isStopRequested()) {
                executionGroup.terminateMidExecution();
                finish(PositionChangeActionEndResult.ForcefulTermination);
                return;
            }

            instance.getRobot().getImuProxy().allPeriodic();
            instance.getRobot().getDrivetrain().getLocalizer().update();

            Pose robotPose = instance.getRobot().getDrivetrain().getLocalizer().getPose();
            velocity = (previousPose != null) ? robotPose.subtract(previousPose).divide(new Pose(period, period, period)) : new Pose(0, 0, 0);
            previousPose = robotPose;

            Pose targetPose = this.pathAlgorithm == null ? this.targetPose :
                    pathAlgorithm.getTargetCompute().invoke(robotPose);

            PositionChangeActionEndResult result = getState(robotPose, targetPose, velocity);
            if (result != null) {
                finish(result);
                return;
            }

            assert targetPose != null;
            Pose powers = getPowers(robotPose, targetPose, velocity);
            MecanumTranslations.getPowers(powers,
                    straightController.getVelocityError(),
                    strafeController.getVelocityError(),
                    hController.getVelocityError()
            ).propagate(instance);
        }
    }

    public double predictHeading(double lastPeriodMillis, double imuLatencyNanos, double headingMeasurement, double initialHeadingVelocity) {
        double estimatedVelocity = initialHeadingVelocity + TURN_PREDICTION_ACCELERATION * (instance.robot.getDrivetrain().voltage() * prevHPower - (13.0 / TURN_MAX_VELOCITY) * initialHeadingVelocity) * lastPeriodMillis / 2000;
        return headingMeasurement + Range.clip(TURN_PREDICTION_VELOCITY * estimatedVelocity * imuLatencyNanos / 1E9, -TURN_MAX_VELOCITY, TURN_MAX_VELOCITY);
    }

    public @NotNull Pose getPowers(@NotNull Pose robotPose, @NotNull Pose targetPose, Pose velocity) {
        double currentTime = System.nanoTime();

        double imuLatencyNanos = currentTime - instance.getRobot().getImuProxy().alternativeImu().getAcquisitionTime();
        double targetHeading = targetPose.getHeading();
        double robotHeading = predictHeading(
                period / 1E6,
                imuLatencyNanos,
                robotPose.getHeading(),
                velocity.getHeading());
        double headingError = targetHeading - robotHeading;

        if (headingError > Math.PI) headingError -= 2 * Math.PI;
        if (headingError < -Math.PI) headingError += 2 * Math.PI;

        Point robotCentricTranslationalError = targetPose.subtract(robotPose).rotate(robotHeading);
        Point robotCentricTranslationalVelocity = velocity.rotate(robotHeading);

        double xPower = strafeController.calculate(
                -robotCentricTranslationalError.x,
                0,
                robotCentricTranslationalVelocity.x);
        double yPower = straightController.calculate(
                -robotCentricTranslationalError.y,
                0,
                robotCentricTranslationalVelocity.y
        );
        double hPower = hController.calculate(-headingError, 0);

        hPower = Range.clip(hPower, -maxRotationalSpeed, maxRotationalSpeed);
        prevHPower = hPower;
        xPower = Range.clip(xPower, -maxRotationalSpeed, maxTranslationalSpeed);
        yPower = Range.clip(yPower, -maxRotationalSpeed, maxTranslationalSpeed);

        updateTelemetry(imuLatencyNanos);

        return new Pose(xPower, yPower, hPower);
    }

    public void updateTelemetry(double imuLatencyNanos) {
        this.instance.getRobot().getMultipleTelemetry().addData(
                "X Position Error",
                PositionChangeAction.strafeController.getPositionError()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "X Velocity Error",
                PositionChangeAction.strafeController.getVelocityError()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "Y Position Error",
                PositionChangeAction.straightController.getPositionError()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "Y Velocity Error",
                PositionChangeAction.straightController.getVelocityError()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "Heading Error",
                Math.toDegrees(hController.getPositionError())
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "Heading Velocity Error",
                PositionChangeAction.hController.getVelocityError()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "Period Milliseconds",
                period / 1E6
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

        if (strafeController.atSetPoint() && strafeController.atSetPoint() && hController.atSetPoint()) {
            return PositionChangeActionEndResult.Successful;
        }

        return null;
    }

    protected void finish(@NotNull PositionChangeActionEndResult result) {
        if (result != PositionChangeActionEndResult.ForcefulTermination) {
            ZERO.propagate(instance);
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
        this.maxRotationalSpeed = maxRotationalSpeed;
    }

    public void withCustomMaxTranslationalSpeed(double maxTranslationalSpeed) {
        this.maxTranslationalSpeed = maxTranslationalSpeed;
    }

    public void withCustomPathAlgorithm(@NotNull PathAlgorithm pathAlgorithm) {
        this.pathAlgorithm = pathAlgorithm;
    }

    public void withAutomaticDeath(double automaticDeathMillis) {
        this.automaticDeathMillis = automaticDeathMillis;
    }

    public void disableAutomaticDeath() {
        this.automaticDeathMillis = null;
    }
}
