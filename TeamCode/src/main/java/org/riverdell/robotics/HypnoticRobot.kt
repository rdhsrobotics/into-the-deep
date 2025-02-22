package org.riverdell.robotics

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import io.liftgate.robotics.mono.Mono
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import io.liftgate.robotics.mono.subsystem.Subsystem
import io.liftgate.robotics.mono.subsystem.System
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.subsystems.Drivetrain
import org.riverdell.robotics.subsystems.IMUProxySubsystem
import org.riverdell.robotics.subsystems.hang.Hang
import org.riverdell.robotics.subsystems.slides.Extension
import org.riverdell.robotics.subsystems.slides.Lift
import org.riverdell.robotics.subsystems.intake.Intake
import org.riverdell.robotics.subsystems.intake.composite.CompositeInteraction
import org.riverdell.robotics.subsystems.intake.v4b.IntakeV4B
import org.riverdell.robotics.subsystems.outtake.Outtake
import org.riverdell.robotics.teleop.HypnoticTeleOp

abstract class HypnoticRobot(val opMode: HypnoticOpMode) : System
{
    companion object
    {
        @JvmStatic
        lateinit var instance: HypnoticRobot
        @JvmStatic
        val pwmResolution = 1.0 / 128

        var resetMode = false
    }

    override val subsystems: MutableSet<Subsystem> = mutableSetOf()
    lateinit var hardware: HypnoticRobotHardware

    val drivetrain by lazy { Drivetrain(this) }
    val intake by lazy { Intake(this) }
    val intakeV4B by lazy { IntakeV4B(this) }
    val intakeComposite by lazy { CompositeInteraction(this) }
    val extension by lazy { Extension(this) }
    val lift by lazy { Lift(this) }
    val hang by lazy { Hang(this) }
    val outtake by lazy { Outtake(this) }

    val imuProxy by lazy { IMUProxySubsystem(opMode) }

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

    fun Telemetry.addEssentialLines()
    {
        addLine("Intake State: ${intake.intakeState}")
        addLine("Wrist State: ${intake.wristState}")
        addLine("4BR State: ${intakeV4B.v4bState}")
        addLine("Coaxial State: ${intakeV4B.coaxialState}")
        addLine("Composite State: ${intakeComposite.state}")

        addLine("LIFT Left Position: ${hardware.liftMotorLeft.currentPosition}")
        addLine("LIFT Left Power: ${hardware.liftMotorLeft.power}")
        addLine("LIFT Right Position: ${hardware.liftMotorRight.currentPosition}")
        addLine("LIFT Right Power: ${hardware.liftMotorRight.power}")

        addLine("EXTENDO Left Position: ${hardware.extensionMotorLeft.currentPosition}")
        addLine("EXTENDO Left Power: ${hardware.extensionMotorLeft.power}")
        addLine("EXTENDO Right Position: ${hardware.extensionMotorRight.currentPosition}")
        addLine("EXTENDO Right Power: ${hardware.extensionMotorRight.power}")
    }
    
    open fun additionalSubSystems(): List<AbstractSubsystem>
    {
        return emptyList()
    }

    fun runOpMode()
    {
        instance = this

        resetMode = opMode.gamepad1.a

        hardware = HypnoticRobotHardware(opMode)
        hardware.initializeHardware()

        register(
            drivetrain, intake, intakeV4B, outtake, extension, lift,
            *additionalSubSystems().toTypedArray()
        )

        if (this is HypnoticTeleOp.TeleOpRobot)
        {
            register(imuProxy)
        }

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