package org.riverdell.robotics.autonomous

import io.liftgate.robotics.mono.pipeline.RootExecutionGroup
import org.riverdell.robotics.autonomous.movement.PathAlgorithm
import org.riverdell.robotics.autonomous.movement.PositionChangeAction
import org.riverdell.robotics.autonomous.movement.geometry.Pose

class VisionPositionAction(
    lockPosition: (Pose) -> Pose,
    unlockConsumer: (Pose, Pose) -> Boolean,
    executionGroup: RootExecutionGroup
) : PositionChangeAction(null, executionGroup) {
    init {
        disableAutomaticDeath()
        withCustomPathAlgorithm(
            PathAlgorithm(
                lockPosition,
                unlockConsumer,
                strict = true
            )
        )
    }
}
