package org.riverdell.robotics.autonomous.movement;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.riverdell.robotics.autonomous.AutonomousWrapper;
import org.riverdell.robotics.autonomous.geometry.Pose;
import org.riverdell.robotics.autonomous.movement.purepursuit.PathAlgorithm;

import io.liftgate.robotics.mono.Mono;
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup;
import kotlin.Unit;
import kotlin.jvm.functions.Function1;

@Config
public class PositionChangeAction {
    private static final double K_STATIC = 1.85;
    private static final DrivetrainUpdates ZERO = new DrivetrainUpdates(0.0, 0.0, 0.0, 0.0);

    public static double xP = 0.07;
    public static double xD = 0.012;

    public static double yP = 0.07;
    public static double yD = 0.012;

    public static double hP = 1;
    public static double hD = 0.075;

    public static double MINIMUM_TRANSLATIONAL_DIFF_FROM_TARGET = 0.75;
    public static double MINIMUM_ROTATIONAL_DIFF_FROM_TARGET = 0.02;
    public static double AT_TARGET_AUTOMATIC_DEATH = 100;

    public PIDFController xController = new PIDFController(xP, 0.0, xD, 0);
    public PIDFController yController = new PIDFController(yP, 0.0, yD, 0);
    public PIDFController hController = new PIDFController(hP, 0.0, hD, 0);

    private final ElapsedTime activeTimer = new ElapsedTime();
    private final ElapsedTime atTargetTimer = new ElapsedTime();
    private final ElapsedTime stuckProtection = new ElapsedTime();

    private final AutonomousWrapper drivetrain;
    private final @Nullable Pose targetPose;

    /**
     * Preferred to use stuck protection in this case...
     */
    private @Nullable Double automaticDeathMillis = 2500.0;

    private double maxTranslationalSpeed = 1.0;
    private double maxRotationalSpeed = 1.0;

    private final RootExecutionGroup executionGroup;

    public PositionChangeAction(
            @Nullable Pose targetPose,
            @NotNull RootExecutionGroup executionGroup
    ) {
        this.drivetrain = AutonomousWrapper.getInstance();
        this.targetPose = targetPose;
        this.executionGroup = executionGroup;
    }

    private @Nullable PathAlgorithm pathAlgorithm = null;
    private @Nullable Function1<PositionChangeActionEndResult, Unit> endSubscription = null;

    protected void finish(@NotNull PositionChangeActionEndResult result) {
        if (result != PositionChangeActionEndResult.ForcefulTermination) {
            ZERO.propagate(drivetrain);
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
            if (drivetrain.isStopRequested()) {
                executionGroup.terminateMidExecution();
                finish(PositionChangeActionEndResult.ForcefulTermination);
                return;
            }

            Pose previousPose = this.previousPose;
            Pose robotPose = drivetrain.getLocalizer().getPose();
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
            MecanumTranslations.getPowers(powers).propagate(drivetrain);
        }
    }

    private @Nullable RobotStuckProtection robotStuckProtection = null;
    public void withStuckProtection(@NotNull RobotStuckProtection stuckProtection) {
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
                Pose movementDelta = currentPose.subtract(previousPose);

                if (movementDelta.toVec2D().magnitude() > robotStuckProtection.getMinimumRequiredTranslationalDifference() ||
                        Math.abs(movementDelta.heading) > robotStuckProtection.getMinimumRequiredRotationalDifference()) {
                    System.out.println("Not stuck");
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
            }
        }

        Pose delta = targetPose.subtract(currentPose);

        if (delta.toVec2D().magnitude() > MINIMUM_TRANSLATIONAL_DIFF_FROM_TARGET || Math.abs(delta.heading) > MINIMUM_ROTATIONAL_DIFF_FROM_TARGET) {
            atTargetTimer.reset();
        }

        if (atTargetTimer.milliseconds() > AT_TARGET_AUTOMATIC_DEATH) {
            return PositionChangeActionEndResult.Successful;
        }

        return null;
    }

    public @NotNull Pose getPower(@NotNull Pose robotPose, @NotNull Pose targetPose) {
        double headingError = targetPose.heading - robotPose.heading;
        if (headingError > Math.PI) targetPose.heading -= 2 * Math.PI;
        if (headingError < -Math.PI) targetPose.heading += 2 * Math.PI;

        double xPower = xController.calculate(robotPose.x, targetPose.x);
        double yPower = yController.calculate(robotPose.y, targetPose.y);
        double hPower = hController.calculate(robotPose.heading, targetPose.heading);

        double x_rotated = xPower * Math.cos(-robotPose.heading) - yPower * Math.sin(-robotPose.heading);
        double y_rotated = xPower * Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);

        hPower = Range.clip(hPower, -maxRotationalSpeed, maxRotationalSpeed);
        x_rotated = Range.clip(x_rotated, -maxTranslationalSpeed / K_STATIC, maxTranslationalSpeed / K_STATIC);
        y_rotated = Range.clip(y_rotated, -maxTranslationalSpeed, maxTranslationalSpeed);

        return new Pose(x_rotated * K_STATIC, y_rotated, hPower);
    }
}