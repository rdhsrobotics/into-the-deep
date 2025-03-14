package org.riverdell.robotics.autonomous

import io.liftgate.robotics.mono.Mono
import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.riverdell.robotics.HypnoticOpMode
import org.riverdell.robotics.HypnoticRobot
import org.riverdell.robotics.autonomous.detection.SampleType
import org.riverdell.robotics.autonomous.detection.VisionPipeline
import org.riverdell.robotics.autonomous.movement.DrivetrainUpdates
import org.riverdell.robotics.autonomous.movement.PositionChangeAction
import org.riverdell.robotics.utilities.managed.ManagedMotorGroup
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.thread
import kotlin.concurrent.withLock

abstract class HypnoticAuto(
    val sampleType: SampleType = SampleType.Red,
    internal val blockExecutionGroup: RootExecutionGroup.(HypnoticAuto) -> Unit,
    internal val onInit: (HypnoticAutoRobot) -> Unit = { }
) : HypnoticOpMode() {
    constructor(
        blockExecutionGroup: RootExecutionGroup.(HypnoticAuto) -> Unit,
        onInit: (HypnoticAutoRobot) -> Unit = { }
    ) : this(SampleType.Red, blockExecutionGroup, onInit)

    companion object {
        @JvmStatic
        lateinit var instance: HypnoticAuto

        @JvmStatic
        var nextUpdates: DrivetrainUpdates? = null
        var previousUpdate: DrivetrainUpdates? = null

        @JvmStatic
        val updateLock = ReentrantLock()

        @JvmStatic
        fun sendZeroCommand() {
            updateLock.withLock {
                nextUpdates = null
                previousUpdate = null
                PositionChangeAction.ZERO.propagate(instance)
            }
        }
    }

    inner class HypnoticAutoRobot : HypnoticRobot(this@HypnoticAuto) {
        val visionPipeline by lazy { VisionPipeline(this@HypnoticAuto, sampleType) }

        override fun additionalSubSystems() = listOf<AbstractSubsystem>(visionPipeline)
        override fun initialize() {
            HypnoticAuto.instance = this@HypnoticAuto

            nextUpdates = null
            previousUpdate = null

            while (opModeInInit()) {
                runPeriodics()
                onInit(this)

                hardware.pinpoint.update()

                multipleTelemetry.addLine("--- Initialization ---")
                multipleTelemetry.addData(
                    "X Position Error",
                    0.0
                )
                multipleTelemetry.addData(
                    "X Velocity Error",
                    0.0
                )
                multipleTelemetry.addData(
                    "Y Position Error",
                    0.0
                )
                multipleTelemetry.addData(
                    "Y Velocity Error",
                    0.0
                )
                multipleTelemetry.addData(
                    "Heading Error",
                    0.0
                )

                multipleTelemetry.addData(
                    "Heading Velocity Error",
                    0.0
                )
                multipleTelemetry.addData(
                    "Period Milliseconds",
                    0.0
                )
                multipleTelemetry.update()
            }
        }

        override fun opModeStart() {
            thread {
                while (opModeIsActive()) { // localizer thread
                    hardware.pinpoint.update()
                }
            }

            thread { // motor power setter thread
                while (opModeIsActive()) {
                    updateLock.withLock {
                        if (nextUpdates != null) {
                            if (previousUpdate != null) {
                                if (!previousUpdate!!.equalsUpdate(nextUpdates!!)) {
                                    nextUpdates!!.propagate(this@HypnoticAuto)
                                }
                            } else {
                                nextUpdates!!.propagate(this@HypnoticAuto)
                            }
                            previousUpdate = nextUpdates
                        }
                    }
                }
            }

            thread { // subsystems thread
                while (opModeIsActive()) {
                    kotlin.runCatching {
                        runPeriodics()
                    }.onFailure {
                        it.printStackTrace()
                    }
                }
            }

            val executionGroup = Mono.buildExecutionGroup {
                blockExecutionGroup(
                    this@HypnoticAuto
                )
            }

            var operatingThreadFinishedProperly = false
            val operatingThread = thread {
                runCatching {
                    executionGroup.executeBlocking()
                }.onFailure {
                    it.printStackTrace()
                }

                operatingThreadFinishedProperly = true
            }

            while (opModeIsActive()) {
                if (operatingThreadFinishedProperly) {
                    ManagedMotorGroup.keepEncoderPositions = true
                    break
                }

                Thread.sleep(50L)
            }

            if (!operatingThreadFinishedProperly) {
                operatingThread.interrupt()
            }
        }
    }

    override fun buildRobot() = HypnoticAutoRobot()
}
