package org.firstinspires.ftc.teamcode.chuckFull.subsystems;


import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


import java.util.List;


import org.firstinspires.ftc.teamcode.hardware.Globals;

public class TurretSubsystem extends SubsystemBase {
    private final SimpleServo turret;
    private final PIDFController pidf;
    private final VisionPortal portal;
    private final AprilTagProcessor tagProcessor;


    private double turretTarget = 150.0;
    private boolean aligned = false;
    private double lastDetectedDistance = -1.0;


    public TurretSubsystem(HardwareMap hw) {
        turret = new SimpleServo(hw, "turret", 0, 300);
        turret.turnToAngle(turretTarget);
        pidf = new PIDFController(Globals.turret.turretKP, Globals.turret.turretKI, Globals.turret.turretKD, Globals.turret.turretKF);


        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(914.101, 914.101, 645.664, 342.333)
                .build();


        portal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hw.get(WebcamName.class, "ov9281"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new android.util.Size(1280, 720))
                .build();


    }


    public void periodic(GamepadEx g) {
        pidf.setPIDF(Globals.turret.turretKP, Globals.turret.turretKI, Globals.turret.turretKD, Globals.turret.turretKF);


        boolean lb = g.getButton(com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.LEFT_BUMPER);
        boolean rb = g.getButton(com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.RIGHT_BUMPER);


        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (!lb ^ rb) {
            portal.setProcessorEnabled(tagProcessor, true);
            AprilTagDetection chosen = null;
            double chosenBearing = 0.0;
            if (detections != null && !detections.isEmpty()) {
                for (AprilTagDetection d : detections) {
                    if (d.ftcPose != null) {
                        double bearing = d.ftcPose.bearing;
                        if (chosen == null || Math.abs(bearing) < Math.abs(chosenBearing)) {
                            chosen = d;
                            chosenBearing = bearing;
                        }
                    }
                }
            }

            if (chosen != null) {
                double err = chosenBearing - Globals.turret.turretLocationError;
                aligned = Math.abs(err) <= Globals.turret.turretTol;
                double delta = aligned ? 0.0 : pidf.calculate(err, 0.0);
                turretTarget -= delta;
                lastDetectedDistance = chosen.ftcPose.range;
            } else {
                aligned = false;
            }
        } else {
            portal.setProcessorEnabled(tagProcessor, false);
            aligned = false;
            if (lb ^ rb) {
                turretTarget += lb ? +Globals.turret.nudge : -Globals.turret.nudge;
            }
        }


        if (turretTarget > 300) turretTarget = 300;
        if (turretTarget < 0) turretTarget = 0;
        turret.turnToAngle(turretTarget);
    }


    public double getLastDetectedDistance() { return lastDetectedDistance; }
}

