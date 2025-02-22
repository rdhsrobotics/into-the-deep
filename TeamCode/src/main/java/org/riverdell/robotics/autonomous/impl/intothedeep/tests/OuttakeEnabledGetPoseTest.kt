package org.riverdell.robotics.autonomous.impl.intothedeep.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.subsystems.outtake.OuttakeLevel

@Autonomous(name = "Outtake Get Pose Test", group = "Test")
class OuttakeEnabledGetPoseTest : HypnoticAuto({ opMode ->
    single("Outtake") {
        opMode.robot.intakeComposite
            .initialOuttakeFromRest(OuttakeLevel.HighBasket)

        Thread.sleep(100000L)
    }
})