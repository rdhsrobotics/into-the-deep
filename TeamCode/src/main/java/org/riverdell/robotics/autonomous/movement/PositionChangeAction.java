package org.riverdell.robotics.autonomous.movement;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.riverdell.robotics.autonomous.HypnoticAuto;
import org.riverdell.robotics.autonomous.movement.geometry.Point;
import org.riverdell.robotics.autonomous.movement.geometry.Pose;
import org.riverdell.robotics.autonomous.movement.geometry.RawPose;
import org.firstinspires.ftc.robotcontroller.internal.localization.GoBildaPinpointDriver;

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

    public static PIDCoefficients strafePID = new PIDCoefficients(0.25, 0.0, 0.02667);
    public static PIDCoefficients straightPID = new PIDCoefficients(0.21, 0.0, 0.033);
    public static PIDCoefficients headingPID = new PIDCoefficients(1.45, 0.0, -0.135);

    public static PIDCoefficients extendoOutHeadingPID = new PIDCoefficients(1.375, 0.0, -0.165);

    public static double TURN_POWER_BOOST = -0.05;
    public static double STRAFE_POWER_BOOST = 0.045;
    public static double STRAIGHT_POWER_BOOST = 0.03;

    private PositionChangeTolerance tolerances = new PositionChangeTolerance();

    public PIDFController strafeController;
    public PIDFController straightController;
    public PIDFController hController;

    private final ElapsedTime activeTimer = new ElapsedTime();
    private final ElapsedTime stuckTime = new ElapsedTime();
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

    private boolean telemetry = true;
    private boolean doNothing = false;
    private boolean noStop = false;

    public PositionChangeAction(@Nullable Pose targetPose, @NotNull RootExecutionGroup executionGroup) {
        this.instance = HypnoticAuto.getInstance();
        this.targetPose = targetPose;
        this.executionGroup = executionGroup;

        strafeController = new PIDFController(strafePID, (current, target, vel) -> 0.0);
        straightController = new PIDFController(straightPID, (current, target, vel) -> 0.0);
        hController = new PIDFController(headingPID, (current, target, vel) -> 0.0);
    }

    private @Nullable PathAlgorithm pathAlgorithm = null;
    private @Nullable Function1<PositionChangeActionEndResult, Unit> endSubscription = null;
    public void executeBlocking() {
        while (true) {
            double currentTimeNanos = System.nanoTime();

            if (prevTime == 0) { prevTime = System.nanoTime(); }
            periodNanos = currentTimeNanos - prevTime;
            prevTime = currentTimeNanos;

            if (instance.isStopRequested()) {
                executionGroup.terminateMidExecution();
                finish(PositionChangeActionEndResult.ForcefulTermination);
                return;
            }


            GoBildaPinpointDriver pinpointDriver = instance.getRobot().getHardware().getPinpoint();
            Pose2D pinpointPose = pinpointDriver.getPosition();

            Pose robotPose = new Pose(
                    pinpointPose.getX(DistanceUnit.INCH),
                    pinpointPose.getY(DistanceUnit.INCH),
                    pinpointPose.getHeading(AngleUnit.RADIANS)
            );

            RawPose velocity = new RawPose(
                    pinpointDriver.getVelocity().getX(DistanceUnit.INCH),
                    pinpointDriver.getVelocity().getY(DistanceUnit.INCH),
                    pinpointDriver.getHeadingVelocity()
            );

            if (previousPose == null) { previousPose = robotPose; }

            Pose targetPose = this.pathAlgorithm == null ? this.targetPose :
                    pathAlgorithm.getTargetCompute().invoke(robotPose);

            if (targetPose == null) {
                return;
            }

            Pose powers = getPowers(robotPose, targetPose, velocity);
            if (!doNothing) {
                HypnoticAuto.setNextUpdates(MecanumTranslations.getPowers(powers,
                        straightController.getVelocityError(),
                        strafeController.getVelocityError(),
                        hController.getVelocityError()
                ));
            }

            if (telemetry) {
                updateTelemetry();
            }

            PositionChangeActionEndResult result = getState(robotPose, targetPose, velocity);
            if (result != null && !doNothing) {
                finish(result);
                return;
            }
        }
    }

    public double wrapAround(double in) {
        if (in > Math.PI) in -= 2 * Math.PI;
        if (in < -Math.PI) in += 2 * Math.PI;
        return in;
    }

    public @NotNull Pose getPowers(@NotNull Pose robotPose, @NotNull Pose targetPose,
                                   RawPose velocity) {
        double targetHeading = targetPose.getHeading();
        double robotHeading = robotPose.getHeading();
        double headingError = targetHeading - robotHeading;

        headingError = wrapAround(headingError);

        Point robotCentricTranslationalError = targetPose.subtract(robotPose).rotate(robotHeading);
        Point robotCentricTranslationalVelocity = velocity.rotate(robotHeading);

        double xPower = strafeController.calculate(
                robotCentricTranslationalError.x, 0, robotCentricTranslationalVelocity.x);
        double yPower = straightController.calculate(
                -robotCentricTranslationalError.y, 0, -robotCentricTranslationalVelocity.y);
        double hPower = hController.calculate(-headingError, 0, velocity.getHeading());

        hPower = Range.clip(hPower, -maxRotationalPower, maxRotationalPower);
        xPower = Range.clip(xPower, -maxTranslationalPower, maxTranslationalPower);
        yPower = Range.clip(yPower, -maxTranslationalPower, maxTranslationalPower);

        return new Pose(xPower, yPower, hPower);
    }

    public void updateTelemetry() {
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
                "Heading Error",
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
        this.instance.getRobot().getMultipleTelemetry().update();
    }

    private @Nullable RobotStuckProtection robotStuckProtection = null;

    public void withStuckProtection(@NotNull RobotStuckProtection stuckProtection) {
        disableAutomaticDeath();
        this.robotStuckProtection = stuckProtection;
    }

    private @Nullable PositionChangeActionEndResult getState(Pose currentPose, Pose targetPose, RawPose velocity) {
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

        if (velocity != null) {
            Pose predictedError = targetPose.subtract(predictPose(currentPose, velocity, tolerances.predictMillis));

            boolean withinPositionTolerance = ((predictedError.radius() < tolerances.translateTolerance)
                    && (Math.abs(predictedError.getHeading()) < tolerances.headingToleranceRad));

            boolean withinVelocityTolerance = (velocity.radius() < tolerances.translateToleranceVel) &&
                    (Math.abs(velocity.getHeading()) < tolerances.headingToleranceVel);

            if (withinPositionTolerance && withinVelocityTolerance) {
                System.out.println("Ending Movement");
                return PositionChangeActionEndResult.Successful;
            }
        }

        return null;
    }

    public Pose predictPose(Pose current, RawPose velocity, double timeMillis) {
        RawPose delta = velocity.divide(1000.0 / timeMillis);
        return current.add(new Pose(delta.x, delta.y, delta.getHeading()));
    }

    protected void finish(@NotNull PositionChangeActionEndResult result) {
        if (result != PositionChangeActionEndResult.ForcefulTermination) {
            if (!noStop) {
                HypnoticAuto.sendZeroCommand();
            }

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

    public void withCustomTolerances(PositionChangeTolerance tolerances) {
        this.tolerances = tolerances;
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

    public void doNothing(boolean use) {
        doNothing = use;
    }

    public void printTelemetry(boolean use) {
        telemetry = use;
    }

    public void disableAutomaticDeath() {
        this.automaticDeathMillis = null;
    }

    public void noStop(boolean use) {
        noStop = use;
    }
}
