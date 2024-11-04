package org.riverdell.robotics

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import io.liftgate.robotics.mono.Mono
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import io.liftgate.robotics.mono.subsystem.Subsystem
import io.liftgate.robotics.mono.subsystem.System
import org.riverdell.robotics.subsystems.Drivetrain
import org.riverdell.robotics.subsystems.hang.Hang
import org.riverdell.robotics.subsystems.slides.Extension
import org.riverdell.robotics.subsystems.slides.Lift
import org.riverdell.robotics.subsystems.intake.Intake
import org.riverdell.robotics.subsystems.intake.composite.CompositeIntake
import org.riverdell.robotics.subsystems.intake.v4b.IntakeV4B
import org.riverdell.robotics.subsystems.outtake.Outtake

abstract class HypnoticRobot(val opMode: HypnoticOpMode) : System
{
    companion object
    {
        @JvmStatic
        lateinit var instance: HypnoticRobot
    }

    override val subsystems: MutableSet<Subsystem> = mutableSetOf()
    lateinit var hardware: HypnoticRobotHardware

    val drivetrain by lazy { Drivetrain(this) }
    val intake by lazy { Intake(this) }
    val intakeV4B by lazy { IntakeV4B(this) }
    val intakeComposite by lazy { CompositeIntake(this) }
    val extension by lazy { Extension(this) }
    val lift by lazy { Lift(this) }
    val hang by lazy { Hang(this) }
    val outtake by lazy { Outtake(this) }

    val multipleTelemetry by lazy {
        MultipleTelemetry(
            opMode.telemetry,
            FtcDashboard.getInstance().telemetry
        )
    }

    abstract fun initialize()
    abstract fun opModeStart()

    fun runPeriodics()
    {
        subsystems
            .map { it as AbstractSubsystem }
            .forEach { it.allPeriodic() }
    }

    open fun additionalSubSystems(): List<AbstractSubsystem>
    {
        return emptyList()
    }

    fun runOpMode()
    {
        instance = this

        hardware = HypnoticRobotHardware(opMode)
        hardware.initializeHardware()

        register(
            drivetrain, intake, intakeV4B, extension, lift,
            *additionalSubSystems().toTypedArray()
        )

        // keep all log entries
        Mono.logSink = {
            multipleTelemetry.addLine("[mono] $it")
            multipleTelemetry.update()
            Log.i("mono", it)
        }

        initializeAll()
        initialize()

        opMode.waitForStart()
        if (opMode.isStopRequested)
        {
            return
        }

        startAll()
        opModeStart()
        disposeOfAll()
    }
}