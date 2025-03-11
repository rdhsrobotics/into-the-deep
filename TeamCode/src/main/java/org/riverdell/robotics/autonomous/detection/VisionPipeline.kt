package org.riverdell.robotics.autonomous.detection

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionPortal.CameraState

/**
 * Manages and configures all [VisionPortal] processors
 * for an op mode.
 */
class VisionPipeline(
    private val opMode: LinearOpMode,
    val coloredType: SampleType = SampleType.Red,
) : AbstractSubsystem()
{
    lateinit var portal: VisionPortal
    lateinit var yellowPipeline: SampleDetectionPipelinePNP
    lateinit var coloredPipeline: SampleDetectionPipelinePNP

    var detectedSample: SampleDetectionPipelinePNP.AnalyzedSample? = null
    var paused = false

    override fun start() {}

    override fun periodic() {
        if (!paused) {
            detectedSample = yellowPipeline.chooseClosestValidSample() ?: coloredPipeline.chooseClosestValidSample()
        }
    }

    override fun doInitialize()
    {
        coloredPipeline = SampleDetectionPipelinePNP()
        coloredPipeline.sampleType = coloredType

        yellowPipeline = SampleDetectionPipelinePNP()
        yellowPipeline.sampleType = SampleType.Yellow

        portal = VisionPortal.Builder()
            .setCamera(
                opMode.hardwareMap["webcam"] as WebcamName
            )
            .setCameraResolution(Size(1280, 960))
            .enableLiveView(false)
            .setAutoStopLiveView(true)
            .addProcessors(
                yellowPipeline,
                coloredPipeline
            )
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()

        FtcDashboard.getInstance().startCameraStream(
            coloredPipeline,
            30.0
        )

        pause()
    }

    fun pause() {
        portal.setProcessorEnabled(yellowPipeline, false)
        portal.setProcessorEnabled(coloredPipeline, false)
        portal.stopStreaming()
        portal.stopLiveView()
        yellowPipeline.clientStoneList.clear()
        coloredPipeline.clientStoneList.clear()
        detectedSample = null
        paused = true
    }

    fun resume() {
        portal.setProcessorEnabled(yellowPipeline, true)
        portal.setProcessorEnabled(coloredPipeline, true)
        if (portal.cameraState != CameraState.ERROR) {
            portal.resumeStreaming()
            portal.resumeLiveView()
        }
        yellowPipeline.clientStoneList.clear()
        coloredPipeline.clientStoneList.clear()
        detectedSample = null
        paused = false
    }

    override fun dispose()
    {
        portal.close()
    }
}
