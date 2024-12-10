package org.riverdell.robotics.autonomous.impl.intothedeep

import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.purepursuit.WaypointLike
import org.riverdell.robotics.subsystems.intake.WristState

data class GroundPickupPosition(
    val pose: Pose,
    val purePursuitPoints: List<WaypointLike>? = null,
    val wristState: WristState = WristState.Lateral
)