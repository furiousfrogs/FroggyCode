package org.firstinspires.ftc.teamcode.Camera;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;

@TeleOp(name = "Detect & Shoot")
public class Detect extends LinearOpMode {

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


        //smooth power var setup
        final int HISTORY_SIZE = 10;
        double[] powerHistory = new double[HISTORY_SIZE];
        int historyIndex = 0;
        boolean arrayFull = false;
        double smoothedPower = 0.0;

        //hardware def
        DcMotor shoot;
        Servo set;

        waitForStart();

        // Init hardware
        shoot = hardwareMap.get(DcMotor.class, "shoot");
        set = hardwareMap.get(Servo.class, "set");

// Set initial position: servo "set" starts at bottom (0.0)
        set.setPosition(0.0);  // assuming 0 = down / reset position

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
                // 🟢 Inside the loop, after computing 'double rawPower = suvat(xyDist);'
// Update history buffer
                powerHistory[historyIndex] = suvat(dist);
                historyIndex = (historyIndex + 1) % HISTORY_SIZE;

                if (historyIndex == 0) {
                    arrayFull = true;  // We've wrapped around → buffer is now full
                }

// Compute average over available samples
                int validCount = arrayFull ? HISTORY_SIZE : historyIndex;
                double sum = 0.0;
                for (int i = 0; i < validCount; i++) {
                    sum += powerHistory[i];
                }
                smoothedPower = sum / validCount;

// Use 'smoothedPower' instead of 'rawPower' when setting motor speed
                telemetry.addData("powa (avg)", "%.3f", smoothedPower);
                telemetry.addData("range (xy)", "%.3f", xyDist);
            }

            telemetry.update();
            sleep(50);

            // Check if gamepad circle button is pressed (PS4) or B button (Xbox)
            if (gamepad1.circle && smoothedPower>0) {  // 'circle' for PS4, 'b' for Xbox

                // Start shooter motor at smoothed power
                shoot.setPower(smoothedPower);
                sleep(2000);

                set.setPosition(1.0);
                sleep(500);       // wait till servo reaches max
                set.setPosition(0.0);
                shoot.setPower(0.0);
            }
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
