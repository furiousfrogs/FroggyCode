package org.firstinspires.ftc.teamcode.commandbase;

import static java.lang.Math.pow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.firstinspires.ftc.teamcode.hardware.turretConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import android.util.Size;

import org.firstinspires.ftc.teamcode.hardware.Globals;

import java.util.List;

@TeleOp(name = "Subsystem Testing")
public class subsystems extends OpMode {
    private boolean aligned = false;
    private boolean speed = false;
    private boolean patternDetected = false;
    private boolean sight = false;
    private boolean autoAimEnabled = true;
    private boolean prevTri = false;

    double turretTarget = 150F;

    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private double distance;
    private double power;
    private PIDFController turretPIDF;
    private double turretPower = 0.0;

    private Motor launcher, revolver,fl,bl,fr,br;
    private SimpleServo set, rotate;

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

    private int revolverTarget = 0;      // persistent target (ticks or degrees—match your units!)
    private boolean indexInProgress = false;  // true while a step is underway

    pattern currentPattern = pattern.PPG;
    @Override
    public void init() {

        //colour = (NormalizedColorSensor) hardwareMap.get(ColorSensor.class, "color");
        //dist = hardwareMap.get(DistanceSensor.class, "colour");

        //revolver = new Motor(hardwareMap, "revolver", 28, 1150);
        //revolver.setRunMode(Motor.RunMode.PositionControl);
        //revolver.setPositionCoefficient(Globals.revolverKP);
        //revolver.setPositionTolerance(Globals.revolverTol);
        //revolverTarget = revolver.getCurrentPosition();
        //revolver.setTargetPosition(revolverTarget);
        //revolver.stopMotor();



        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.


        fl = new Motor(hardwareMap,"fl");
        fl.setRunMode(Motor.RunMode.RawPower);
        fl.setInverted(true);

        bl = new Motor(hardwareMap,"bl");
        bl.setRunMode(Motor.RunMode.RawPower);
        bl.setInverted(true);

        fr = new Motor(hardwareMap,"fr");
        fr.setRunMode(Motor.RunMode.RawPower);

        br = new Motor(hardwareMap,"br");
        br.setRunMode(Motor.RunMode.RawPower);


        launcher = new Motor(hardwareMap, "l1", 28, 6000);
        launcher.setRunMode(Motor.RunMode.RawPower);

        set = new SimpleServo(hardwareMap, "set", 0, 180, AngleUnit.DEGREES);
        rotate = new SimpleServo(hardwareMap, "turret", 0, 300, AngleUnit.DEGREES);
        rotate.turnToAngle(turretTarget);

        lastTime = getRuntime();
        lastPosition = launcher.getCurrentPosition();
        gamepadEx = new GamepadEx(gamepad1);


        turretPIDF = new PIDFController(Globals.turretKP, Globals.turretKI, Globals.turretKD, Globals.turretKF);


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
        autoAimServoMode();      // now only reads/controls; does NOT rebuild vision
        doTelemetry();
        //findPattern();
        //revolverRotate();
        drive();
    }

    private void rotate(boolean clockwise) {
        if (indexInProgress) {

            if (revolver.atTargetPosition()) {
                revolver.stopMotor();
                indexInProgress = false;
            } else {
                revolver.set(0.8);
            }
            return;
        }

        // Start a new step
        double step = clockwise ? Globals.oneRotation : -Globals.oneRotation;
        revolverTarget += step;
        revolver.setTargetPosition(revolverTarget);
        revolver.set(0.8);
        indexInProgress = true;
    }

    private void findPattern() {
        if (!patternDetected && !gamepadEx.getButton(GamepadKeys.Button.OPTIONS)) {
            List<AprilTagDetection> code = tagProcessor.getDetections();
            if (code != null && !code.isEmpty() && !patternDetected) {
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
        } if (gamepadEx.getButton(GamepadKeys.Button.OPTIONS)) {
            patternDetected = false;
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

    private void autoAimServoMode() {
        // Pull live gains (Dashboard)
        turretPIDF.setP(Globals.turretKP);
        turretPIDF.setI(Globals.turretKI);
        turretPIDF.setD(Globals.turretKD);
        turretPIDF.setF(Globals.turretKF);

        // Toggle with TRIANGLE (edge)
        boolean tri = gamepadEx.getButton(GamepadKeys.Button.TRIANGLE);
        if (tri && !prevTri) {
            autoAimEnabled = !autoAimEnabled;
            turretPIDF.reset(); // avoid D/I kick
        }
        prevTri = tri;

        boolean lb = gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rb = gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER);


        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (autoAimEnabled && !lb ^ rb) {
            visionPortal.setProcessorEnabled(tagProcessor, true);

            AprilTagDetection chosen = null;
            double chosenBearing = 0.0;
            if (detections != null && !detections.isEmpty()) {
                for (AprilTagDetection d : detections) {
                    if (d.ftcPose != null) {
                        double bearing = d.ftcPose.bearing; // +- based on red/blue side
                        if (chosen == null || Math.abs(bearing) < Math.abs(chosenBearing)) {
                            chosen = d;
                            chosenBearing = bearing;
                        }
                    }
                }
            }

            if (chosen != null) {

                double err = chosenBearing
                        - Globals.turretCamOffset                 // camera not centered
                        - turretConstants.turretLocationError;    // mechanical mount offset

                aligned = Math.abs(err) <= Globals.turretTol;

                // PID output is a delta-angle; setpoint is 0 (we want zero error)
                double delta = aligned ? 0.0 : turretPIDF.calculate(err, 0.0);

                // Limit how much we move the target this loop (smooths big swings)
                if (delta >  turretConstants.maxStep) delta =  turretConstants.maxStep;
                if (delta < -turretConstants.maxStep) delta = -turretConstants.maxStep;

                turretTarget -= delta; //THIS IS NEGATIVE
            } else {
                aligned = false;

            }

        } else if (!autoAimEnabled && !patternDetected) {
            visionPortal.setProcessorEnabled(tagProcessor, false);
        }

        if (lb ^ rb) {
            aligned = false;
            turretTarget += lb ? +turretConstants.nudge : -turretConstants.nudge;

        }
        if (turretTarget > 300) {
            turretTarget = 300;
        } else if (turretTarget < 0) {
            turretTarget = 0;
        }

        rotate.turnToAngle(turretTarget);

    }
    // ----------------- Launcher utilities (unchanged logic) -----------------



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

        if (gamepadEx.getButton(GamepadKeys.Button.CROSS) && aligned) {
            launcher.set(feedforwardPower);
            if (Math.abs(Globals.targetrpm - RPM) < Globals.launcherTol) { // TODO replace targetrpm with power
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



    private void drive(){
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.left_trigger - gamepad1.right_trigger;
        if (Math.abs(y)<0.2){
            y=0.0;
        }
        if (Math.abs(x)<0.2){
            x=0.0;
        }

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fl.set(frontLeftPower);
        bl.set(backLeftPower);
        fr.set(frontRightPower);
        br.set(backRightPower);

        telemetry.addData("y",y);
        telemetry.addData("x",x);
        telemetry.addData("frontleftPower",frontLeftPower);
        telemetry.addData("backleftPower",backLeftPower);
        telemetry.addData("frontRightPower",frontRightPower);
        telemetry.addData("backrightPower",backRightPower);
    }
}
