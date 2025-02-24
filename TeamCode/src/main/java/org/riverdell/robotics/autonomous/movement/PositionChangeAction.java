package org.riverdell.robotics.autonomous.movement;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.riverdell.robotics.autonomous.HypnoticAuto;
import org.riverdell.robotics.autonomous.movement.geometry.Pose;

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

    public double K_STATIC = 1.85;

    public static double xP = 0.05;
    public static double xD = 0.007;

    public static double yP = 0.125;
    public static double yD = 0.02;

    public static double hP = 0.82;
    public static double hD = 0.07;

    public double MINIMUM_TRANSLATIONAL_DIFF_FROM_TARGET = 0.75;
    public double MINIMUM_ROTATIONAL_DIFF_FROM_TARGET = 0.02;
    public double AT_TARGET_AUTOMATIC_DEATH = 200;

    public static PIDFController xController;
    public static PIDFController yController;
    public static PIDFController hController;

    private final ElapsedTime activeTimer = new ElapsedTime();
    private final ElapsedTime atTargetTimer = new ElapsedTime();
    private final ElapsedTime stuckProtection = new ElapsedTime();

    private final @Nullable Pose targetPose;

    /**
     * Preferred to use stuck protection in this case...
     */
    private @Nullable Double automaticDeathMillis = 2500.0;

    private double maxTranslationalSpeed = 1.0;
    private double maxRotationalSpeed = 1.0;

    public PositionChangeAction(@Nullable Pose targetPose, @NotNull RootExecutionGroup executionGroup) {
        this.instance = HypnoticAuto.getInstance();
        this.targetPose = targetPose;
        this.executionGroup = executionGroup;

//        populateDefaults();

        double head = 0.0;
        double pog = 0.0;

        xController = new PIDFController(xP, 0.0, xD, (ass, penis) -> head - pog);
        yController = new PIDFController(yP, 0.0, yD, (ass, penis) -> head - pog);
        hController = new PIDFController(hP, 0.0, hD, (ass, penis) -> head - pog);
    }

    private @Nullable PathAlgorithm pathAlgorithm = null;
    private @Nullable Function1<PositionChangeActionEndResult, Unit> endSubscription = null;

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

    private @Nullable Pose previousPose = null;

    public void executeBlocking() {
        while (true) {
            if (instance.isStopRequested()) {
                executionGroup.terminateMidExecution();
                finish(PositionChangeActionEndResult.ForcefulTermination);
                return;
            }

            Pose previousPose = this.previousPose;
            Pose robotPose = instance.getRobot().getDrivetrain().getLocalizer().getPose();
            this.previousPose = robotPose;

            Pose targetPose = this.pathAlgorithm == null ? this.targetPose :
                    pathAlgorithm.getTargetCompute().invoke(robotPose);

            if (robotPose == null || targetPose == null) {
                finish(PositionChangeActionEndResult.LocalizationFailure);
                return;
            }

            PositionChangeActionEndResult result = getState(robotPose, previousPose, targetPose);
            if (result != null) {
                finish(result);
                return;
            }

            Pose powers = getPower(robotPose, targetPose);
            MecanumTranslations.getPowers(powers, 0.0, 0.0, 0.0).propagate(instance);
        }
    }

    private @Nullable RobotStuckProtection robotStuckProtection = null;

    public void withStuckProtection(@NotNull RobotStuckProtection stuckProtection) {
        disableAutomaticDeath();
        this.robotStuckProtection = stuckProtection;
    }

    private @Nullable PositionChangeActionEndResult getState(
            @NotNull Pose currentPose,
            @Nullable Pose previousPose,
            @NotNull Pose targetPose
    ) {
        if (automaticDeathMillis != null) {
            if (activeTimer.milliseconds() > automaticDeathMillis) {
                return PositionChangeActionEndResult.ExceededTimeout;
            }
        }

        if (robotStuckProtection != null) {
            if (previousPose != null) {
                Pose movementDelta = currentPose.subtract(previousPose); // chore: do buffer system
                if (movementDelta.toVec2D().magnitude() > robotStuckProtection.getMinimumRequiredTranslationalDifference() ||
                        Math.abs(movementDelta.getHeading()) > robotStuckProtection.getMinimumRequiredRotationalDifference()) {
                    stuckProtection.reset();
                }

                if (stuckProtection.milliseconds() > robotStuckProtection.getMinimumMillisUntilDeemedStuck()) {
                    return PositionChangeActionEndResult.StuckDetected;
                }
            } else {
                stuckProtection.reset();
            }
        }

        if (pathAlgorithm != null && !pathAlgorithm.getEuclideanCompletionCheck()) {
            if (pathAlgorithm.getPathComplete().invoke(currentPose, targetPose)) {
                return PositionChangeActionEndResult.PathAlgorithmSuccessful;
            } else if (pathAlgorithm.getStrict()) {
                return null;
            }
        }

        Pose delta = targetPose.subtract(currentPose);

        if ((delta.toVec2D().magnitude() > MINIMUM_TRANSLATIONAL_DIFF_FROM_TARGET ||
                Math.abs(delta.getHeading()) > MINIMUM_ROTATIONAL_DIFF_FROM_TARGET)) {
            atTargetTimer.reset();
        }

        if (atTargetTimer.milliseconds() > AT_TARGET_AUTOMATIC_DEATH) {
            return PositionChangeActionEndResult.Successful;
        }

        return null;
    }

    public @NotNull Pose getPower(@NotNull Pose robotPose, @NotNull Pose targetPose) {
        double headingError = targetPose.getHeading() - robotPose.getHeading();
        if (headingError > Math.PI) targetPose.setHeading(targetPose.getHeading() - 2 * Math.PI);
        if (headingError < -Math.PI) targetPose.setHeading(targetPose.getHeading() + 2 * Math.PI);

        double xPower = xController.calculate(robotPose.x, targetPose.x);
        double yPower = yController.calculate(robotPose.y, targetPose.y);
        double hPower = hController.calculate(robotPose.getHeading(), targetPose.getHeading());

        double x_rotated = xPower * Math.cos(-robotPose.getHeading()) - yPower * Math.sin(-robotPose.getHeading());
        double y_rotated = xPower * Math.sin(-robotPose.getHeading()) + yPower * Math.cos(-robotPose.getHeading());

        hPower = Range.clip(hPower, -maxRotationalSpeed, maxRotationalSpeed);
        x_rotated = Range.clip(x_rotated, -maxTranslationalSpeed / K_STATIC, maxTranslationalSpeed / K_STATIC);
        y_rotated = Range.clip(y_rotated, -maxTranslationalSpeed, maxTranslationalSpeed);

        return new Pose(x_rotated * K_STATIC, y_rotated, hPower);
    }
}