package org.riverdell.robotics.autonomous.detection

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.riverdell.robotics.autonomous.detection.SampleDetectionPipelinePNP.AnalyzedSample

/**
 * Manages and configures all [VisionPortal] processors
 * for an op mode.
 */
@Config
class VisionPipeline(
    private val opMode: LinearOpMode,
    val colorType: SampleType = SampleType.Red,
) : AbstractSubsystem() {
    companion object {
        @JvmField
        var CAMERA_WIDTH = 1280.0
        @JvmField
        var CAMERA_HEIGHT = 720.0
        @JvmField
        var STREAM_VIEW = StreamType.Neutral
    }

    lateinit var portal: VisionPortal

    lateinit var yellowPipeline: SampleDetectionPipelinePNP
    lateinit var coloredPipeline: SampleDetectionPipelinePNP

    @Volatile
    var aggregateSampleCache = listOf<AnalyzedSample>()

    var preferredNextPick: AnalyzedSample? = null
    var paused = false

    override fun start() {}

    override fun periodic() {
        if (!paused) {
            aggregateSampleCache = yellowPipeline.allAnalyzedSamples + coloredPipeline.allAnalyzedSamples
        }
    }

    override fun doInitialize() {
        coloredPipeline = SampleDetectionPipelinePNP()
        coloredPipeline.sampleType = colorType

        yellowPipeline = SampleDetectionPipelinePNP()
        yellowPipeline.sampleType = SampleType.Yellow

        portal = VisionPortal.Builder()
            .setCamera(
                opMode.hardwareMap["webcam"] as WebcamName
            )
            .setCameraResolution(Size(
                CAMERA_WIDTH.toInt(),
                CAMERA_HEIGHT.toInt()
            ))
            .enableLiveView(false)
            .setAutoStopLiveView(true)
            .addProcessors(
                yellowPipeline,
                coloredPipeline
            )
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()

        startStreaming()
        pause()
    }

    private fun startStreaming() {
        FtcDashboard.getInstance().stopCameraStream()
        FtcDashboard.getInstance().startCameraStream(
            if (STREAM_VIEW == StreamType.Neutral) yellowPipeline else coloredPipeline,
            30.0
        )
    }

    fun pause() {
        portal.setProcessorEnabled(yellowPipeline, false)
        portal.setProcessorEnabled(coloredPipeline, false)

        portal.stopStreaming()
        portal.stopLiveView()

        clearCache()

        paused = true
    }

    fun clearCache() {
        yellowPipeline.clearCache()
        coloredPipeline.clearCache()
        aggregateSampleCache = listOf()
    }

    fun resume() {
        clearCache()

        portal.setProcessorEnabled(yellowPipeline, true)
        portal.setProcessorEnabled(coloredPipeline, true)

        portal.resumeStreaming()
        portal.resumeLiveView()

        paused = false
    }

    override fun dispose() {
        portal.close()
    }
}
