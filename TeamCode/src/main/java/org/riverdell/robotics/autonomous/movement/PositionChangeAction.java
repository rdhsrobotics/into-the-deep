package org.riverdell.robotics.autonomous.movement;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.riverdell.robotics.autonomous.HypnoticAuto;
import org.riverdell.robotics.autonomous.movement.geometry.Pose;
import org.riverdell.robotics.autonomous.movement.konfig.NavigationConfig;

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

    //public double K_STATIC = 1.85;

    public static double strafeP = 0.11;
    public static double strafeD = 0.012;

    public static double straightP = 0.075;
    public static double straightD = 0.01;

    public static double hP = 2.2;
    public static double hI = 0.0;
    public static double hD = 0.16;

    public static double TURN_POWER_BOOST = 0.06;
    public static double STRAFE_POWER_BOOST = 0.18;
    public static double STRAIGHT_POWER_BOOST = 0.03;

    public static double TURN_PREDICTION_ACCELERATION = 0.000057;
    public static double TURN_MAX_VELOCITY = 5;

    public double MINIMUM_TRANSLATIONAL_DIFF_FROM_TARGET = 0.7;
    public double MINIMUM_ROTATIONAL_DIFF_FROM_TARGET = 2.0;
    public double AT_TARGET_AUTOMATIC_DEATH = 250;

    public static PIDFController strafeController;
    public static PIDFController straightController;
    public static PIDFController hController;

    private final ElapsedTime activeTimer = new ElapsedTime();
    private final ElapsedTime atTargetTimer = new ElapsedTime();
    private final ElapsedTime stuckProtection = new ElapsedTime();

    private final LinkedList<double[]> prevHPowers = new LinkedList<>();
    private double prevHPower = 0.0;

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

        strafeController = new PIDFController(strafeP, 0.0, strafeD, 0.0);
        straightController = new PIDFController(straightP, 0.0, straightD, 0.0);
        hController = new PIDFController(hP, hI, hD, 0.0);
    }

    private void populateDefaults() {
        final NavigationConfig config = ((HypnoticAuto.HypnoticAutoRobot) instance.getRobot()).getNavigationConfig();

        MINIMUM_ROTATIONAL_DIFF_FROM_TARGET = config.getMinimumRotationalDifferenceFromTarget();
        MINIMUM_TRANSLATIONAL_DIFF_FROM_TARGET = config.getMinimumTranslationDifferenceFromTarget();
        AT_TARGET_AUTOMATIC_DEATH = config.getAutomaticDeathMillis();

        strafeP = config.getXP();
        strafeD = config.getXD();

        straightP = config.getYP();
        straightD = config.getYD();

        hP = config.getHP();
        hD = config.getHD();
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
            instance.getRobot().getImuProxy().allPeriodic();
            instance.getRobot().getDrivetrain().getLocalizer().update();
            Pose robotPose = instance.getRobot().getDrivetrain().getLocalizer().getPose();
            this.previousPose = robotPose;

            Pose targetPose = this.pathAlgorithm == null ? this.targetPose :
                    pathAlgorithm.getTargetCompute().invoke(robotPose);

            if (robotPose == null || targetPose == null) {
                finish(PositionChangeActionEndResult.LocalizationFailure);
                return;
            }

            Pose powers = getPower(robotPose, targetPose);
            MecanumTranslations.getPowers(powers,
                    straightController.getVelocityError(),
                    strafeController.getVelocityError(),
                    hController.getVelocityError()
            ).propagate(instance);

            PositionChangeActionEndResult result = getState(robotPose, previousPose, targetPose);
            if (result != null) {
                finish(result);
                return;
            }
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

        if (((delta.toVec2D().magnitude() < MINIMUM_TRANSLATIONAL_DIFF_FROM_TARGET &&
            Math.abs(Math.toDegrees(delta.getHeading())) < MINIMUM_ROTATIONAL_DIFF_FROM_TARGET))
        && ((Math.abs(hController.getVelocityError()) < 0.25)
                && (Math.abs(Math.hypot(straightController.getVelocityError(), strafeController.getVelocityError())) < 0.5))) {
            return PositionChangeActionEndResult.Successful;
        }

        return null;
    }

    public double predictHeading(double lastPeriodMillis, double imuLatencyMillis, double headingMeasurement, double initialHeadingVelocity) {
        double maxPower = TURN_PREDICTION_ACCELERATION * instance.robot.getDrivetrain().voltage();
        double estimatedVelocity = initialHeadingVelocity + (maxPower * prevHPower - (13.0 * TURN_PREDICTION_ACCELERATION / TURN_MAX_VELOCITY)) * lastPeriodMillis;
        return headingMeasurement + Range.clip(estimatedVelocity * imuLatencyMillis / 1000, -TURN_MAX_VELOCITY, TURN_MAX_VELOCITY);
    }

    public @NotNull Pose getPower(@NotNull Pose robotPose, @NotNull Pose targetPose) {
        double targetHeading = targetPose.getHeading();
        double currentTime = System.nanoTime();
        double imuLatencyMillis = (currentTime - instance.getRobot().getImuProxy().alternativeImu().getAcquisitionTime()) / 1E6;

        double robotHeading = Math.toRadians(instance.getRobot().getImuProxy().alternativeImu().getYaw());

        robotHeading = predictHeading((currentTime - hController.getLastTimeStamp()) / 1E6, imuLatencyMillis, robotHeading, hController.getVelocityError());
        //robotHeading += (imuLatencyMillis/1000) * Range.clip(hController.getVelocityError(), -TURN_MAX_PREDICTION, TURN_MAX_PREDICTION); // correct for IMU latency, velocity is in deg/s
        double headingError = robotHeading - targetHeading;

        if (headingError > Math.PI) headingError -= 2 * Math.PI;
        if (headingError < -Math.PI) headingError += 2 * Math.PI;

        Vector2d error = new Vector2d(targetPose.x - robotPose.x, targetPose.y - robotPose.y).rotateBy(-Math.toDegrees(robotPose.getHeading()));

        double xPower = strafeController.calculate(-error.getX(), 0);
        double yPower = straightController.calculate(-error.getY(), 0);
        double hPower = hController.calculate(headingError, 0);

        prevHPower = Range.clip(hPower, -1.0, 1.0);

        this.instance.getRobot().getMultipleTelemetry().addData(
                "X Position Error",
                PositionChangeAction.strafeController.getPositionError()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "Y Position Error",
                PositionChangeAction.straightController.getPositionError()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "X Velocity Error",
                PositionChangeAction.strafeController.getVelocityError()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "Y Velocity Error",
                PositionChangeAction.straightController.getVelocityError()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "H Velocity Error",
                PositionChangeAction.hController.getVelocityError()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "Period",
                strafeController.getPeriod()
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "IMU Latency",
                imuLatencyMillis
        );
        this.instance.getRobot().getMultipleTelemetry().addData(
                "Heading",
                Math.toDegrees(headingError)
        );
        this.instance.getRobot().getMultipleTelemetry().update();


//        double x_rotated = xPower * Math.cos(-robotPose.getHeading()) - yPower * Math.sin(-robotPose.getHeading());
//        double y_rotated = xPower * Math.sin(-robotPose.getHeading()) + yPower * Math.cos(-robotPose.getHeading());

        hPower = Range.clip(hPower, -maxRotationalSpeed, maxRotationalSpeed);
//        x_rotated = Range.clip(x_rotated, -maxTranslationalSpeed / K_STATIC, maxTranslationalSpeed / K_STATIC);
//        y_rotated = Range.clip(y_rotated, -maxTranslationalSpeed, maxTranslationalSpeed);
        xPower = Range.clip(xPower, -maxRotationalSpeed, maxTranslationalSpeed);
        yPower = Range.clip(yPower, -maxRotationalSpeed, maxTranslationalSpeed);

        return new Pose(xPower, yPower, hPower);
    }
}
