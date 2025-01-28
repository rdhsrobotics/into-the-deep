package org.riverdell.robotics.subsystems.outtake

import org.riverdell.robotics.subsystems.slides.LiftConfig

enum class OuttakeLevel(val encoderPercentage: Double) {
    rest(0.0),
    lowBar(0.25),
    lowBasket(0.35),
    scoring(0.54),  // Just below Bar2 for safer scoring
    lowBar2(0.56),  // Existing low bar position
    bar2(0.58),     // Existing middle position
    highBar(0.72),
    HighBasket(1.0);

    val encoderLevel: Int
        get() = (encoderPercentage * LiftConfig.MAX_EXTENSION).toInt()

    fun next() = entries.getOrNull(ordinal + 1)
    fun previous() = entries.getOrNull(ordinal - 1)
}