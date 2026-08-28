package org.firstinspires.ftc.teamcode.opModes.subsystems

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

object Intake : Subsystem {

    val intake = MotorEx("intake")

    val spinFast = InstantCommand {
        intake.power = 1.0
    }

    val stop = InstantCommand {
        intake.power = 0.0
    }
}