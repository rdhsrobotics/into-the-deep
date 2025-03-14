package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.PositionChangeAction
import org.riverdell.robotics.autonomous.movement.geometry.Pose

@Autonomous(name = "\uD83E\uDD16 Do Nothing", group = "Test")
class DoNothing : HypnoticAuto({ opMode ->
    single("do nothing") {
        val targetPose = Pose()
        val positionChangeAction = PositionChangeAction(targetPose, this.parent).apply {
            doNothing(true)
            printTelemetry(true)
        }
        if (instance.isStopRequested) {
            this.parent.terminateMidExecution()
        }

        positionChangeAction.executeBlocking()
    }
})