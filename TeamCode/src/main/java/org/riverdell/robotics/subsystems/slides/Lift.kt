package org.riverdell.robotics.subsystems.slides

import com.acmerobotics.roadrunner.control.PIDCoefficients
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.movement.cubicBezier
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import org.riverdell.robotics.utilities.managed.pidf.PIDFConfig
import kotlin.math.abs

class Lift(val robot: HypnoticRobot) : AbstractSubsystem()
{
    val slides = with(PIDFConfig(LiftConfig.kP, LiftConfig.kI, LiftConfig.kD)) { //0.01, 0.0, 0.0005
        ManagedMotorGroup(
            this@Lift,
            PIDCoefficients(kP, kI, kD),
            kV, kA, kStatic,
            tolerance = 20,
            kF = { position, targetPosition, velocity ->
                val error = position - targetPosition
                if (targetPosition > 50.0) // If going up, always resist gravity
                {
                    0.17
                } else { // If going near 0, give extra push down
                    if (error > -20 && error < 90) // When elevator is just above the target position
                    {
                        (0.01 * error * abs(error)).coerceIn(-0.2, 0.2)
                    } else {
                        0.0 // Don't pull down when very far from target
                    }
                }
            },
            master = robot.hardware.liftMotorLeft,
            slaves = listOf(robot.hardware.liftMotorRight)
        ).withTimeout(2000)
    }

    fun position() = robot.hardware.liftMotorLeft.currentPosition
    fun extendToAndStayAt(position: Int) = slides.goTo(position)
    fun isExtending() = slides.isTravelling()

    override fun start()
    {
//        extendToAndStayAt(0)
    }

    override fun doInitialize()
    {

    }

}