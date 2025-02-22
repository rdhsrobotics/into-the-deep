package org.riverdell.robotics.autonomous.impl.intothedeep.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.subsystems.intake.WristState

@Autonomous(name = "Intake Get Pose Test", group = "Test")
class IntakeEnabledGetPoseTest : HypnoticAuto(something@{ opMode ->
    single("hors") {
        opMode.robot.intakeComposite.prepareForPickup(
            WristState.Lateral,
            wideOpen = true,
            submersibleOverride = 300
        ).join()

        Thread.sleep(100000L)
    }
})