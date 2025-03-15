package org.riverdell.robotics.autonomous.impl.intothedeep.posers

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.impl.intothedeep.visionIntake

@Autonomous(name = "\uD83E\uDD16 Submersible Intake", group = "Test")
class SubmersibleAutoIntakeTest : HypnoticAuto(something@{ opMode ->
    visionIntake(opMode, isolated = true)
})