package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.abs

object Drivetrain : Subsystem {

    val frontLeft = MotorEx("frontLeft").reversed()
    val frontRight = MotorEx("frontRight")
    val backLeft = MotorEx("backLeft").reversed()
    val backRight = MotorEx("backRight")

    override fun initialize() {
        listOf(frontLeft, frontRight, backLeft, backRight).forEach {
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    // y = forward/back, x = strafe, rx = rotate
    fun driveMecanum(y: Double, x: Double, rx: Double) {
        val denom = maxOf(abs(y) + abs(x) + abs(rx), 1.0)
        frontLeft.power = (y + x + rx) / denom
        backLeft.power = (y - x + rx) / denom
        frontRight.power = (y - x - rx) / denom
        backRight.power = (y + x - rx) / denom
    }

    fun stop() = driveMecanum(0.0, 0.0, 0.0)
}