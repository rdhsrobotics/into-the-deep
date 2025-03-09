package org.riverdell.robotics.subsystems.outtake

import org.riverdell.robotics.subsystems.slides.LiftConfig

enum class OuttakeLevel(val encoderPercentage: Double)
{
    Bar2(0.32), LowBasket(0.4), HighBasket(0.93);
    val encoderLevel: Int
        get() = (encoderPercentage * LiftConfig.MAX_EXTENSION).toInt()

    fun next() = entries.getOrNull(ordinal + 1)
    fun previous() = entries.getOrNull(ordinal - 1)
}