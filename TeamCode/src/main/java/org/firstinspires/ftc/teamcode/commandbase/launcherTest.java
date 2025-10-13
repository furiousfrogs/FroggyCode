package org.firstinspires.ftc.teamcode.commandbase;

import static java.lang.Math.pow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.SensorColor;
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
    private boolean patternDetected = false;
    private boolean sight = false;
    private boolean autoAimEnabled = false;
    private boolean prevTri = false;

    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private double distance;
    private double power;
    private PIDFController pidf;
    private double turretPower = 0.0;

    private Motor launcher, revolver;
    private SimpleServo set;
    private CRServo rotate;

    private double RPM;
    private double lastTime;
    private int lastPosition;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private GamepadEx gamepadEx;
    private double bearing = 0.0;

    private NormalizedColorSensor colour;
    private DistanceSensor dist;
    public enum pattern {
        PPG,
        PGP,
        GPP
    }

    pattern currentPattern = pattern.PPG;
    @Override
    public void init() {

        //colour = (NormalizedColorSensor) hardwareMap.get(ColorSensor.class, "color");
        //dist = hardwareMap.get(DistanceSensor.class, "colour");

        revolver = new Motor(hardwareMap, "revolver", 28, 1150);
        revolver.setRunMode(Motor.RunMode.PositionControl);
        revolver.setPositionCoefficient(Globals.revolverKP);


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
    public void loop() { // TODO add revolver sequence logic
        calculateRPM();
        launcherawe();
        autoAim();      // now only reads/controls; does NOT rebuild vision
        doTelemetry();
        findPattern();
        //revolverRotate();
    }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }

    private void revolverRotate() {
        revolver.setPositionCoefficient(Globals.revolverKP);
        revolver.setPositionTolerance(Globals.revolverTol);
        if (gamepadEx.getButton(GamepadKeys.Button.SQUARE)) {
            revolver.setTargetPosition(revolver.getCurrentPosition() + 1160); // 2 rotations
        } else if (gamepadEx.getButton(GamepadKeys.Button.CROSS)) {
            revolver.setTargetPosition(revolver.getCurrentPosition() - 1160);
        }
        if (!revolver.atTargetPosition()) {
            revolver.set(0.20);
        }
        revolver.stopMotor();
    }

    private void findPattern() {
        List<AprilTagDetection> code = tagProcessor.getDetections();
        if (code!= null && !code.isEmpty() && !patternDetected) {
            for (AprilTagDetection c : code) {
                if (c.id == 21) {
                    currentPattern = pattern.GPP;
                    patternDetected = true;
                } else if (c.id == 22) {
                    currentPattern = pattern.PGP;
                    patternDetected = true;
                } else if (c.id == 23) {
                    currentPattern = pattern.PPG;
                    patternDetected = true;
                } else {
                    patternDetected = false;
                    telemetry.addLine("NO PATTERN FOUND");
                }
            }
        }

    } //TODO currently is always on, add a toggle.

    private void patternAlgo() {
        switch (currentPattern) {
            case PPG:


                break;
            case PGP:
                break;
            case GPP:
                break;

        }
    }

    private void autoAim() {
        // Pull live gains (Dashboard)
        pidf.setP(Globals.turretKP);
        pidf.setI(Globals.turretKI);
        pidf.setD(Globals.turretKD);
        pidf.setF(Globals.turretKF);

        // Toggle with TRIANGLE (edge)
        boolean tri = gamepadEx.getButton(GamepadKeys.Button.TRIANGLE);
        if (tri && !prevTri) {
            autoAimEnabled = !autoAimEnabled;
            pidf.reset(); // avoid D/I kick
        }
        prevTri = tri;

        boolean lb = gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rb = gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER);


        List<AprilTagDetection> detections = tagProcessor.getDetections();


        AprilTagDetection chosen = null;
        double chosenBearing = 0.0;
        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection d : detections) {
                if (d.ftcPose != null) {
                    double b = d.ftcPose.bearing;
                    if (chosen == null || Math.abs(b) < Math.abs(chosenBearing)) {
                        chosen = d;
                        chosenBearing = b;
                    }
                }
            }
        }

        if (autoAimEnabled && !(lb || rb)) {
            if (chosen != null) {
                double err = chosenBearing;
                boolean onTarget = Math.abs(err) <= Globals.turretTol;
                aligned = onTarget;

                double ctrlErr = onTarget ? 0.0 : err;
                double raw = pidf.calculate(ctrlErr, 0.0);
                double out = applyMinEffort(raw, Globals.turretMin);


                turretPower = -clamp(out, -Globals.turretMax, +Globals.turretMax);
            } else {
                aligned = false;
                turretPower = 0.0;
            }

        } else if (lb ^ rb) {
            aligned = false;
            turretPower = lb ? -Globals.turretMax : +Globals.turretMax;

        } else {
            aligned = false;
            turretPower = 0.0;
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

                power = (2358.7* pow(2.71828, 0.008*distance)); // TODO FORMULA FOR POWER. replace targetrpm with the formula
                speed = true;
            }
        } else {
            power = 0;
            speed = false;
        }

        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Globals.fwKs, Globals.fwKv, Globals.fwKa);
        double feedforwardPower = ff.calculate(power, 0.0);

        if (gamepadEx.getButton(GamepadKeys.Button.CROSS) && aligned) {
            launcher.set(feedforwardPower);
            if (power > 1000 && Math.abs(power - RPM) < Globals.launcherTol) { // TODO replace targetrpm with power
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
        telemetry.addData("pattern?: ", currentPattern);
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
