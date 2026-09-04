package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // ================= PINPOINT LOCALIZATION =================
    // Reset to a clean slate for THIS robot. Do NOT trust these
    // until you've run the Localization Test + Offsets Tuner.
    //
    // Tuning order:
    //   1. Run Tuning -> Localization Test
    //   2. Push robot forward by hand -> x should increase.
    //      If not: .forwardEncoderDirection(Encoder.REVERSE)
    //   3. Push robot left by hand -> y should increase.
    //      If not: .strafeEncoderDirection(Encoder.REVERSE)
    //   4. Run Tuning -> Offsets Tuner (or measure manually in inches)
    //      to get real forwardPodY / strafePodX values.
    //   5. Re-run Localization Test: spin in place, x/y should stay
    //      near 0. Drive around and return to start, should read ~0.
    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // ================= DRIVETRAIN =================
    // Motor names/directions are structural (from your actual wiring),
    // kept as-is. xVelocity/yVelocity below are still the COPIED values
    // from the other team's robot - not needed for Pinpoint tuning today,
    // but re-tune with the Velocity Tuner OpModes before relying on them.
    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // TODO: still copied values - re-tune later with Velocity Tuners
            .xVelocity(86.82)
            .yVelocity(71.54);

    // ================= FOLLOWER / BRAKING =================
    // Still copied values - not needed for Pinpoint tuning today.
    // Re-tune later with the Heading Tuner and PredictiveBrakingTuner.
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1, 0, .01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0))
            .useSecondaryHeadingPIDF(true)
            // TODO: still copied values - re-tune later with PredictiveBrakingTuner
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.15, 0.084677871, 0.00157));

    public static PathConstraints pathConstraints = new PathConstraints(0.97, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}