package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;
//fff
@Autonomous(name = "Drive To AprilTag", group = "Test")
public class limelighttest extends LinearOpMode {

    private Limelight3A limelight;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // --- Tune these for your robot/field ---
    private static final int TARGET_TAG_ID = -1;        // -1 = accept any tag; set a specific ID to lock onto one tag
    private static final double TARGET_DISTANCE_IN = 12.0; // stop once this close to the tag, in inches
    private static final double DISTANCE_TOLERANCE_IN = 0.5; // stop-band so it doesn't hunt back and forth
    private static final double MAX_DRIVE_POWER = 0.4;   // cap forward speed
    private static final double MIN_DRIVE_POWER = 0.12;  // minimum power to overcome friction near the target
    private static final double kP_DRIVE = 0.03;         // power per inch of remaining distance
    private static final double kP_STRAFE = 0.02;        // power per inch of lateral (x) offset, to stay centered on tag
    private static final long NO_TAG_TIMEOUT_MS = 3000;  // give up if tag isn't seen for this long
    private static final double SEARCH_POWER = -0.2;     // power while searching backward (negative = backward)

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready - waiting for start");
        telemetry.update();

        waitForStart();
        limelight.start();

        long lastTagSeenTime = System.currentTimeMillis();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();
            LLResultTypes.FiducialResult targetTag = null;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult tag : fiducials) {
                    if (TARGET_TAG_ID == -1 || tag.getFiducialId() == TARGET_TAG_ID) {
                        targetTag = tag;
                        break;
                    }
                }
            }

            if (targetTag != null) {
                lastTagSeenTime = System.currentTimeMillis();

                // Pose of the tag relative to the CAMERA: x = left/right, z = forward distance (meters)
                Position camSpacePosition = targetTag.getTargetPoseCameraSpace().getPosition();
                double forwardDistIn = camSpacePosition.z * 39.3701;
                double lateralOffsetIn = camSpacePosition.x * 39.3701;

                double distanceError = forwardDistIn - TARGET_DISTANCE_IN;

                telemetry.addData("Tag ID", targetTag.getFiducialId());
                telemetry.addData("Distance (in)", forwardDistIn);
                telemetry.addData("Lateral offset (in)", lateralOffsetIn);

                if (Math.abs(distanceError) <= DISTANCE_TOLERANCE_IN) {
                    stopDrive();
                    telemetry.addLine("Target distance reached - stopping");
                    telemetry.update();
                    break; // done - exit autonomous
                }

                double drivePower = clampPower(distanceError * kP_DRIVE);
                double strafePower = clampStrafe(lateralOffsetIn * kP_STRAFE);

                driveMecanumPower(drivePower, strafePower, 0);

            } else {
                // No tag visible this cycle - drive backward while searching
                if (System.currentTimeMillis() - lastTagSeenTime > NO_TAG_TIMEOUT_MS) {
                    stopDrive();
                    telemetry.addLine("No tag found - timed out, stopping");
                    telemetry.update();
                    break;
                }
                driveMecanumPower(SEARCH_POWER, 0, 0);
                telemetry.addLine("No tag in view - searching (driving backward)");
            }

            telemetry.update();
        }

        stopDrive();
    }

    /** Applies a minimum power so the robot doesn't stall out near the target, capped at MAX_DRIVE_POWER. */
    private double clampPower(double raw) {
        double magnitude = Math.min(Math.abs(raw), MAX_DRIVE_POWER);
        if (magnitude < MIN_DRIVE_POWER) magnitude = MIN_DRIVE_POWER;
        return Math.copySign(magnitude, raw);
    }

    private double clampStrafe(double raw) {
        return Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, raw));
    }

    /**
     * y = forward power, x = strafe power, rx = rotate power.
     */
    private void driveMecanumPower(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower((y + x - rx) / denominator);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}