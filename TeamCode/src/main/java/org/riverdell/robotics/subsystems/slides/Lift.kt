package org.riverdell.robotics.subsystems.slides

import com.acmerobotics.roadrunner.control.PIDCoefficients
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import org.riverdell.robotics.utilities.managed.pidf.PIDFConfig

class Lift(val robot: HypnoticRobot) : AbstractSubsystem()
{
    val slides = with(PIDFConfig(LiftConfig.kP, LiftConfig.kI, LiftConfig.kD)) { //0.01, 0.0, 0.0005
        ManagedMotorGroup(
            this@Lift,
            PIDCoefficients(kP, kI, kD),
            kV, kA, kStatic,
            tolerance = 7,
            kF = { position, targetPosition, velocity ->
                val error = position - targetPosition
                if (targetPosition > 50.0) // If going up, resist gravity
                {
                    if (error > -40 && error < -2) { // If a little below, push up
                        if ((velocity ?: 0.0) > 10.0) { LiftConfig.f_g + 0.4 }
                        else { LiftConfig.f_g }
                    } else {
                        LiftConfig.f_g
                    }
                } else { // If going near 0, give extra push down
                    if (error > 4 && error < 30) // When elevator is just above the target position
                    {
                        /*(0.03 * error * abs(error)).coerceIn(-0.35, 0.35)*/ - 0.3
                    } else {
                        0.0 // Don't pull down when very far or at target
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

    fun asyncPeriodic() {
        super.periodic()
    }

    override fun periodic() {

    }

    override fun doInitialize()
    {

    }

}