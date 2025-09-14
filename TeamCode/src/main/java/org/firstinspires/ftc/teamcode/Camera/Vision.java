package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;

@TeleOp(name = "Vision (AprilTag + VisionPortal)")
public class Vision extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Build the AprilTag processor ---
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)

                // fx, fy, cx, cy, imageWidth, imageHeight  (all in *pixels*)
                .setLensIntrinsics(1496.12, 1496.12, 966.425, 541.166)


                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))  // match RC config
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new android.util.Size(1920, 1080))// must match the intrinsics width/height above
                .build();


        telemetry.addLine("Initialized. Press START to begin scanning.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();

            telemetry.addData("Detections", detections.size());

            if (!detections.isEmpty()) {
                AprilTagDetection d = detections.get(0); // show the first (or choose best)
                telemetry.addData("id", d.id);
                telemetry.addData("range (m)", "%.3f", d.ftcPose.range);
                //telemetry.addData("x/y/z (m)", "%.3f / %.3f / %.3f",
                // d.ftcPose.x, d.ftcPose.y, d.ftcPose.z);
                //telemetry.addData("yaw/pitch/roll (deg)", "%.1f / %.1f / %.1f",
                //Math.toDegrees(d.ftcPose.yaw),
                // Math.toDegrees(d.ftcPose.pitch),
                //Math.toDegrees(d.ftcPose.roll));
                telemetry.addData("margin", "%.2f", d.decisionMargin);
                double dist = d.ftcPose.range;
                double xyDist = Math.sqrt(d.ftcPose.x * d.ftcPose.x + d.ftcPose.y * d.ftcPose.y);
                telemetry.addData("range (xy)", "%.3f", xyDist);
                telemetry.addData("powa: ", suvat(dist));
            }

            telemetry.update();
            sleep(50);
        }

    }

    public double suvat(double dist){

        double g = 9.82;
        double wheelRadius = 0.048;
        double wheelWeight = 0.082;
        double wheelInertia = 0.5 * wheelWeight * wheelRadius * wheelRadius;

        double maxRPM = 6000;
        double maxTorque = 24.3; //Nm

        double launchAngle = Math.toRadians(60);
        double launchHeight = 0.04318;
        double goalHeight = 0.1;
        double s = goalHeight - launchHeight;

        double denom = 2.0 * Math.pow(Math.cos(launchAngle), 2) * (dist * Math.tan(launchAngle) - s);

        double vSq = (g * dist * dist) / denom;
        double v = Math.sqrt(vSq);

        double angularVelocity = v / wheelRadius;

        double requiredRPM = angularVelocity * 60.0 / (2 * Math.PI);

        double power = requiredRPM / maxRPM;
        power = Math.min(1.0, Math.max(0.0, power));

        //set power = p * maxRPM.. p * maxTorque
        return power;
    }
}
