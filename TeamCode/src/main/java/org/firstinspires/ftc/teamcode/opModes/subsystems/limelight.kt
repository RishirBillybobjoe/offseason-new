package org.firstinspires.ftc.teamcode.opModes.subsystems

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode

object Limelight : Subsystem {

    private const val APRILTAG_PIPELINE = 0
    private const val POLLEN_PIPELINE = 1 // set this to whatever pipeline index runs your ball detector

    lateinit var ll: Limelight3A
        private set

    override fun initialize() {
        ll = ActiveOpMode.hardwareMap.get(Limelight3A::class.java, "limelight")
        ll.setPollRateHz(100)
        ll.pipelineSwitch(APRILTAG_PIPELINE)
        ll.start()
    }

    fun useAprilTagPipeline() = ll.pipelineSwitch(APRILTAG_PIPELINE)
    fun usePollenPipeline() = ll.pipelineSwitch(POLLEN_PIPELINE)

    private fun latestResult(): LLResult? {
        val result = ll.latestResult
        return if (result != null && result.isValid) result else null
    }

    fun seesAprilTag(): Boolean =
        latestResult()?.fiducialResults?.isNotEmpty() ?: false

    fun seesPollenBall(): Boolean =
        latestResult()?.colorResults?.isNotEmpty() ?: false

    // horizontal angle offset (deg) and target area (bigger = closer) for the ball
    fun ballOffsets(): Pair<Double, Double>? {
        val result = latestResult() ?: return null
        val ball = result.colorResults.firstOrNull() ?: return null
        return Pair(result.tx, ball.targetArea)
    }
}