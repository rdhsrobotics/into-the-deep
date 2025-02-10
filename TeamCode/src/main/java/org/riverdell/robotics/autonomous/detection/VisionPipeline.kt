package org.riverdell.robotics.autonomous.detection

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.liftgate.robotics.mono.subsystem.AbstractSubsystem
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal

/**
 * Manages and configures all [VisionPortal] processors
 * for an op mode.
 */
class VisionPipeline(
    private val opMode: LinearOpMode
) : AbstractSubsystem()
{
    lateinit var portal: VisionPortal
    lateinit var sampleDetection: SampleDetection

    override fun start()
    {

    }

    override fun periodic() {
    }

    override fun doInitialize()
    {
        sampleDetection = SampleDetection()
        portal = VisionPortal.Builder()
            .setCamera(
                opMode.hardwareMap["webcam"] as WebcamName
            )
            .setCameraResolution(Size(1280, 960))
            .enableLiveView(false)
            .setAutoStopLiveView(true)
            .addProcessors(sampleDetection)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()

        FtcDashboard.getInstance().startCameraStream(
            sampleDetection,
            30.0
        )

        pause()
    }

    fun pause()
    {
        portal.setProcessorEnabled(sampleDetection, false)
        portal.stopStreaming()
        portal.stopLiveView()
    }

    override fun dispose()
    {
        portal.close()
    }
}
