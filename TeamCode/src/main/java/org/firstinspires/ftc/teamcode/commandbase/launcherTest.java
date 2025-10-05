package org.firstinspires.ftc.teamcode.commandbase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.util.Size;

import org.firstinspires.ftc.teamcode.hardware.Globals;

import java.util.List;

@TeleOp(name = "launcher test")
public class launcherTest extends OpMode {
    private boolean aligned = false;
    private boolean speed = false;
    private boolean sight = false;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private double distance;
    private double power;
    private PIDFController pidf;
    private double turretPower = 0.0;


    private Motor launcher;
    private SimpleServo set;
    private CRServo rotate;

    private double RPM;
    private double lastTime;
    private int lastPosition;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private GamepadEx gamepadEx;
    private double bearing = 0.0;
    @Override
    public void init() {

        launcher = new Motor(hardwareMap, "fl", 28, 6000);
        launcher.setRunMode(Motor.RunMode.RawPower);

        set = new SimpleServo(hardwareMap, "set", 0, 180, AngleUnit.DEGREES);
        rotate = new CRServo(hardwareMap, "rotate");

        lastTime = getRuntime();
        lastPosition = launcher.getCurrentPosition();
        gamepadEx = new GamepadEx(gamepad1);


        pidf = new PIDFController(Globals.turretKP, Globals.turretKI, Globals.turretKD, Globals.turretKF);


        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(914.101, 914.101, 645.664, 342.333)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "ov9281"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new android.util.Size(1280, 720))
                .build();

        telemetry.addLine("Initialized. Press START.");
        telemetry.update();
    }

    @Override
    public void loop() {
        calculateRPM();
        launcherawe();
        autoAim();      // now only reads/controls; does NOT rebuild vision
        doTelemetry();
    }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }

    // ----------------- AUTO AIM (bearing -> power) -----------------
    private void autoAim() {
        pidf.setP(Globals.turretKP);
        pidf.setI(Globals.turretKI);
        pidf.setD(Globals.turretKD);
        pidf.setF(Globals.turretKF);


        boolean lb = gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rb = gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER);
        if (!(lb || rb)) {  // neither bumper -> auto aim

            List<AprilTagDetection> detections = tagProcessor.getDetections();

            AprilTagDetection chosen = null;
            double chosenBearing = 0.0;

            if (detections != null && !detections.isEmpty()) {
                if (turretPower == 0) {
                    aligned = true;
                }
                for (AprilTagDetection d : detections) {
                    if (d.ftcPose != null) {
                        bearing = d.ftcPose.bearing;
                        double b = d.ftcPose.bearing; // deg; +left, -right
                        if (chosen == null || Math.abs(b) < Math.abs(chosenBearing)) {
                            chosen = d;
                            chosenBearing = b;
                        }
                    }
                }
            }

            if (chosen != null) {
                double err = chosenBearing;
                boolean onTargetNow = Math.abs(err) <= Globals.turretTol;


                aligned = onTargetNow;
                double bearing = Math.abs(chosenBearing) < Globals.turretTol ? 0.0 : chosenBearing;

                double raw = pidf.calculate(bearing, 0.0);          // measured, setpoint
                double out = applyMinEffort(raw, Globals.turretMin);       // beat stiction

                turretPower = clamp(-out, -Globals.turretMax, +Globals.turretMax);   // safety
            } else {
                turretPower = 0.0; // no tag -> stop (or add slow scan here)
                aligned = false;
            }

        } else if (lb ^ rb) {  // exactly one bumper pressed -> manual nudge
            // use max safe speed, not ±1
            turretPower = lb ? +1 : -1;
            aligned = false;

        } else {
            // both pressed (or any other case) -> stop
            turretPower = 0.0;
            aligned = false;
        }

        rotate.set(turretPower);
    }

    // ----------------- Launcher utilities (unchanged logic) -----------------
    private void calculateRPM() {
        double currentTime = getRuntime();
        int currentPosition = launcher.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        int deltaTicks = currentPosition - lastPosition;

        if (deltaTime > 0.1) {
            double revs = (double) deltaTicks / 28.0; // GoBILDA CPR
            RPM = (revs / deltaTime) * 60.0;

            lastTime = currentTime;
            lastPosition = currentPosition;
        }
    }


    private void launcherawe() {
        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (detections != null && !detections.isEmpty() && aligned) {
            for (AprilTagDetection d : detections) {
                distance = d.ftcPose.range;
                power = Globals.targetrpm; // TODO FORMULA FOR POWER. replace targetrpm with the formula
                speed = true;
            }
        } else {
            power = 0;
            speed = false;
        }

        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Globals.fwKs, Globals.fwKv, Globals.fwKa);
        double feedforwardPower = ff.calculate(power, 0.0);

        if (gamepadEx.getButton(GamepadKeys.Button.CROSS)) {
            launcher.set(feedforwardPower);
            if (power > 1000 && Math.abs(Globals.targetrpm - RPM) < Globals.launcherTol) { // TODO replace targetrpm with power
                set.turnToAngle(Globals.upset);
            }
        } else {
            launcher.set(0);
            set.turnToAngle(Globals.downset);
        }

    }


    private void doTelemetry() {
        telemetry.addData("RPM", RPM);
        telemetry.addData("bearing: ", bearing);
        telemetry.addData("distance: ", distance);
        telemetry.addData("aligned? ", aligned);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("RPM", RPM);
        packet.put("TurretPower", turretPower);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    // ----------------- Helpers -----------------
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double applyMinEffort(double v, double minEffort) {
        if (v == 0.0) return 0.0;
        double sign = Math.signum(v);
        double mag = Math.abs(v);
        if (mag < minEffort) mag = minEffort;
        return sign * mag;
    }
}
