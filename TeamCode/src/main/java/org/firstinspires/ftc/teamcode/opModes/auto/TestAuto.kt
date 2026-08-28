package org.firstinspires.ftc.teamcode.opModes.auto

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opModes.subsystems.Intake

import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@Autonomous(name = "TestAuto")
class TestAuto: NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                Intake),
            BulkReadComponent,
            PedroComponent(Constants::createFollower)
        )
    }

    val startPose = Pose(59.2, 7.0, Math.toRadians(90.0))

    val pointTwo = Pose(11.6, 36.0, )
    val pointThree = Pose(61.7, 8.2, )
    val pointFour = Pose(12.4,59.0)
    val pointFive = Pose(60.8, 8.0)
    val pointSix = Pose(11.7, 83.6)
    val pointSeven = Pose(60.6, 7.9)
    val pointEight = Pose(9.0,64.6)
    val endPoint = Pose(42.7, 64.0)

    val controlPointOne = Pose(60.8,38.3)
    val controlPointTwo = Pose(66.1, 65.4)
    val controlPointThree = Pose(69.5,89.0)

    lateinit var firstPath: PathChain
    lateinit var secondPath: PathChain
    lateinit var thirdPath: PathChain
    lateinit var fourthPath: PathChain
    lateinit var fifthPath: PathChain
    lateinit var sixthPath: PathChain
    lateinit var seventhPath: PathChain
    lateinit var eigthPath: PathChain


    fun buildPaths() {
        firstPath= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierCurve(startPose,controlPointOne, pointTwo))
            .setLinearHeadingInterpolation(90.0,180.0)
            .build()
        secondPath= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pointTwo, pointThree))
            .setLinearHeadingInterpolation(180.0,115.0)
            .build()
        thirdPath= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierCurve(pointThree,controlPointTwo, pointFour))
            .setTangentHeadingInterpolation()
            .build()
        fourthPath= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pointFour, pointFive))
            .setLinearHeadingInterpolation(180.0,115.0)
            .build()
        fifthPath= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierCurve(pointFive,controlPointThree, pointSix))
            .setTangentHeadingInterpolation()
            .build()
        sixthPath= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pointSix, pointSeven))
            .setLinearHeadingInterpolation(180.0,115.0)
            .build()
        seventhPath= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pointSeven, pointEight))
            .setConstantHeadingInterpolation(120.0)
            .build()
        eigthPath= PedroComponent.Companion.follower.pathBuilder()
            .addPath(BezierLine(pointEight, endPoint))
            .setTangentHeadingInterpolation()
            .build()


    }



    val autoRoutine: Command
        get() =
            SequentialGroup(
                FollowPath(firstPath),
                FollowPath(secondPath),
                FollowPath(thirdPath),
                FollowPath(fourthPath),
                FollowPath(fifthPath),
                FollowPath(sixthPath),
                FollowPath(seventhPath),
                FollowPath(eigthPath)

            )


    override fun onInit() {
        PedroComponent.Companion.follower.setMaxPower(1.0)


    }

    override fun onStartButtonPressed() {
        PedroComponent.Companion.follower.setStartingPose(startPose)

        buildPaths()
        autoRoutine()

    }

    override fun onStop() {
    }

    override fun onUpdate() {

    }

}