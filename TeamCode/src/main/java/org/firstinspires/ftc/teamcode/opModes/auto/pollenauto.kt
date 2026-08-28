package org.firstinspires.ftc.teamcode.opModes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.opModes.subsystems.Drivetrain
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake
import org.firstinspires.ftc.teamcode.opModes.subsystems.Limelight

@Autonomous(name = "PollenAuto")
class PollenAuto : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(Drivetrain, Limelight, Intake)
        )
    }

    private enum class State { DRIVE_BACK, TURN, DRIVE_TO_BALL, INTAKE, DONE }
    private var state = State.DRIVE_BACK

    // tune these
    private val searchPower = -0.3
    private val turnPower = 0.3
    private val targetArea = 8.0
    private val kP_forward = 0.02
    private val kP_turn = 0.015

    override fun onStartButtonPressed() {
        state = State.DRIVE_BACK
        Limelight.useAprilTagPipeline()
    }

    override fun onUpdate() {
        when (state) {
            State.DRIVE_BACK -> {
                Drivetrain.driveMecanum(searchPower, 0.0, 0.0)
                if (Limelight.seesAprilTag()) {
                    Drivetrain.stop()
                    Limelight.usePollenPipeline()
                    state = State.TURN
                }
            }

            State.TURN -> {
                Drivetrain.driveMecanum(0.0, 0.0, turnPower)
                telemetry.addData("Sees Ball", Limelight.seesPollenBall())
                if (Limelight.seesPollenBall()) {
                    Drivetrain.stop()
                    state = State.DRIVE_TO_BALL
                }
            }

            State.DRIVE_TO_BALL -> {
                val offsets = Limelight.ballOffsets()
                telemetry.addData("Sees Ball", Limelight.seesPollenBall())
                telemetry.addData("Offsets", offsets)
                if (offsets == null) {
                    Drivetrain.stop()
                } else {
                    val (tx, area) = offsets
                    if (area >= targetArea) {
                        Drivetrain.stop()
                        state = State.INTAKE
                    } else {
                        val forward = ((targetArea - area) * kP_forward).coerceIn(0.1, 0.4)
                        val turn = -tx * kP_turn
                        Drivetrain.driveMecanum(forward, 0.0, turn)
                    }
                }
            }

            State.INTAKE -> {
                Intake.spinFast()
                state = State.DONE
            }

            State.DONE -> {
                // sit here, auto finished
            }
        }

        telemetry.addData("State", state)
        telemetry.update()
    }
}