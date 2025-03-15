package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.navigateTo

@Autonomous(name = "\uD83E\uDD16 Movement", group = "Test")
class TestMovement : HypnoticAuto({ opMode ->
    single("go to position") {
        (opMode.robot as HypnoticAutoRobot).visionPipeline.pause()

        navigateTo(Pose(TestConfig.x, TestConfig.y, TestConfig.turn.degrees)) {
            withAutomaticDeath(
                TestConfig.automaticDeath
            )
            withCustomTolerances(TestConfig.positionChangeTolerance)
            withExtendoOut(TestConfig.extendoOut)
            printTelemetry(true)
        }
    }
})