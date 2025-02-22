package org.riverdell.robotics.autonomous.movement.geometry

import kotlinx.serialization.Serializable
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

@Serializable
open class Point @JvmOverloads constructor(
    @JvmField var x: Double = 0.0,
    @JvmField var y: Double = 0.0
)
{
    fun subtract(other: Point): Point
    {
        return Point(x - other.x, y - other.y)
    }

    fun add(other: Point): Point
    {
        return Point(x + other.x, y + other.y)
    }

    fun add(scalar: Double): Point
    {
        return Point(x + scalar, y + scalar)
    }

    fun subt(other: Point): Point
    {
        return Point(x - other.x, y - other.y)
    }

    open fun divide(div: Double): Point
    {
        return Point(x / div, y / div)
    }

    fun distanceTo(other: Point): Double
    {
        return other.subtract(this).radius()
    }

    fun atan(): Double
    {
        return atan2(x, y)
    }

    fun radius(): Double
    {
        return hypot(x, y)
    }

    fun rotate(amount: Double): Point
    {
        return polar(radius(), atan() + amount)
    }

    fun polar(r: Double, a: Double): Point
    {
        return Point(cos(a) * r, sin(a) * r)
    }

    override fun toString(): String
    {
        return "$x, $y"
    }
}