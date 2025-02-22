package org.riverdell.robotics.autonomous.impl.intothedeep.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.impl.intothedeep.visionIntake

@Autonomous(name = "Submersible AutoIntake Test", group = "Test")
class SubmersibleAutoIntakeTest : HypnoticAuto(something@{ opMode ->
    visionIntake(opMode, isolated = true)
})