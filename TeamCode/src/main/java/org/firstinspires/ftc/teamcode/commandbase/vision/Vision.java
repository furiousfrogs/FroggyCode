package org.firstinspires.ftc.teamcode.Camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.var;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;

@TeleOp(name = "Vision (AprilTag + VisionPortal)")
public class Vision extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor fl = hardwareMap.dcMotor.get("fl");
        DcMotor bl = hardwareMap.dcMotor.get("bl");
        DcMotor fr = hardwareMap.dcMotor.get("fr");
        DcMotor br = hardwareMap.dcMotor.get("br");

        // Set directions as needed for your drivetrain
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(914.101, 914.101, 645.664, 342.333)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "ov9281"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new android.util.Size(1280, 720))
                .build();

        telemetry.addLine("Initialized. Press START to begin scanning.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            List<AprilTagDetection> detections = tagProcessor.getDetections();
            telemetry.addData("Detections", detections.size());

            if (gamepad1.circle) {
                if (!detections.isEmpty()) {
                    AprilTagDetection d = detections.get(0);
                    telemetry.addData("id", d.id);
                    telemetry.addData("range (m)", "%.3f", d.ftcPose.range);
                    telemetry.addData("margin", "%.2f", d.decisionMargin);
                    double xyDist = Math.sqrt(d.ftcPose.x * d.ftcPose.x + d.ftcPose.y * d.ftcPose.y);
                    telemetry.addData("range (xy)", "%.3f", xyDist);

                    double bearingRad = Math.atan2(d.ftcPose.x, d.ftcPose.y);
                    double bearingDeg = Math.toDegrees(bearingRad);
                    telemetry.addData("bearing (deg)", "%.1f", bearingDeg);

                    if (d.decisionMargin > var.minDecisionMargin && d.ftcPose.y  > 0.1) {
                        double error = bearingDeg;              // target = 0°
                        double turn = var.kPturn * error;           // proportional control
                        turn = Math.max(-var.maxPower, Math.min(var.maxPower, turn));

                        if (Math.abs(error) <= var.alignDegTol) {
                            fl.setPower(0.0);
                            bl.setPower(0.0);
                            fr.setPower(0.0);
                            br.setPower(0.0);
                            telemetry.addLine("Aligned ✓");
                        } else {
                            if (Math.abs(turn) < var.minPower) {
                                turn = Math.copySign(var.minPower, turn);
                            }
                            // Positive 'turn' spins right; negative spins left
                            // Left side gets +turn, right side gets -turn (given your motor directions)
                            fl.setPower(+turn);
                            bl.setPower(+turn);
                            fr.setPower(-turn);
                            br.setPower(-turn);
                            telemetry.addData("turnCmd", "%.2f", turn);
                        }
                    } else {
                        telemetry.addLine("No reliable forward-facing tag.");
                        fl.setPower(0.0); bl.setPower(0.0); fr.setPower(0.0); br.setPower(0.0);
                    }
                } else {
                    telemetry.addLine("No tags detected.");
                    fl.setPower(0.0); bl.setPower(0.0); fr.setPower(0.0); br.setPower(0.0);
                }
            } else {
                fl.setPower(0.0);
                bl.setPower(0.0);
                fr.setPower(0.0);
                br.setPower(0.0);
            }

            telemetry.update();
        }
    }
}

