package org.riverdell.robotics.subsystems.slides

import com.acmerobotics.roadrunner.control.PIDCoefficients
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import org.riverdell.robotics.utilities.managed.StuckProtection
import org.riverdell.robotics.utilities.managed.pidf.PIDFConfig
import java.util.concurrent.CompletableFuture

class Extension(val robot: HypnoticRobot) : AbstractSubsystem()
{
    val slides = with(PIDFConfig(ExtensionConfig.kP, 0.0, ExtensionConfig.kD)) {
        ManagedMotorGroup(
            this@Extension,
            PIDCoefficients(kP, kI, kD),
            kV, kA,
            .1,
            kF = { position, target, velocity ->
                val error = position - target
                if (error > 10 && error < 70) { // If extendo is close to being fully retracted, pull harder
                    -0.3
                } else
                {
                    0.0
                }
            },
            master = robot.hardware.extensionMotorRight,
            slaves = listOf(robot.hardware.extensionMotorLeft)
        ).withTimeout(1500L)
            .enableStuckProtection(StuckProtection(
                minimumRequiredPositionDifference = 5,
                timeStuckUnderMinimumMillis = 200L
            ))
    }

    fun extendToAndStayAt(position: Int) = slides.goTo(position)
    fun isExtending() = slides.isTravelling()

    override fun start()
    {
        slides.idle()
    }

    fun reconfigureForHang() {
        slides.disableStuckProtection()
        slides.withTimeout(15000L)
        slides.feedForward { static(-1.0) }
    }

    override fun doInitialize()
    {

    }

    fun asyncPeriodic() {
        super.periodic()
    }

    override fun periodic() {

    }

}