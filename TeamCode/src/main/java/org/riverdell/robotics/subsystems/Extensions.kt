package org.riverdell.robotics.subsystems

import com.qualcomm.robotcore.hardware.ServoImplEx
import io.liftgate.robotics.mono.states.StateHolder
import org.riverdell.robotics.utilities.managed.ManagedServo
import org.riverdell.robotics.utilities.motionprofile.ProfileConstraints

fun StateHolder.motionProfiledServo(
    id: String, servo: ServoImplEx,
    constraints: ProfileConstraints
) = ManagedServo(
    id,
    servo,
    this,
    constraints
)