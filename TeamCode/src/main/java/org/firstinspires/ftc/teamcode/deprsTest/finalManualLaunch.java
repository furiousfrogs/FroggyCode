package org.firstinspires.ftc.teamcode.deprsTest;

import static java.lang.Math.pow;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.hardware.Globals;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import java.util.Collections;
import java.util.Objects;


@TeleOp(name = "finalManualLaunch")
public class finalManualLaunch extends OpMode {

    // ----- booleans/toggles
    private boolean aligned = false;
    private boolean patternDetected = false;
    private boolean autoAimEnabled = true;
    private boolean prevTri = false;
    private boolean previousRotation;
    private boolean revolverReadytoLaunch = false;
    private boolean clockwise;
    private int filled;
    double revolverSetupTimer = Double.MAX_VALUE;
    private boolean revolverReady = true;


    // ----- action booleans -----
    private boolean shootLoop = false;
    public boolean rotating = false;
    private boolean ejectAction = false;
    public boolean shootAction = false;

    // ----- timers -----
    private ElapsedTime globalTimer = new ElapsedTime();
    private double ejectTimer;
    private double shootTimer;
    private double rotateTimer;

    // ----- doubles -----
    private double feedforwardPower = 0;
    private double distance;
    private double power;
    private int shotsFired = 0;
    private double previousRevolverPosition;


    // ----- pid's -----
    private PIDFController turretPIDF, ff, revolverPID;


    // ----- Motors, servos, sensors -----
    private Motor launcher1, launcher2, revolver,fl,bl,fr,br, intake;
    private SimpleServo set, rotate, eject;
    private NormalizedColorSensor colourSensor;
    private DistanceSensor distanceSensor;

    // ----- launcher -----
    private double RPM;
    private double lastTime;
    private int lastPosition;

    // ----- turret -----
    private double bearing = 0.0;
    double turretTarget = 150F; // inital turret angle

    // ----- revolver -----
    private final float[] hsv = new float[3];

    private boolean shootCounterClockwise = false;



    //index 0 is the top, 1 is the left, 2 is the right when looking from the front view
    private List<String> revolverState = new ArrayList<>(Arrays.asList("EMPTY", "EMPTY", "EMPTY"));
    private List<String> finalRevolver = new ArrayList<>(Arrays.asList("EMPTY", "EMPTY", "EMPTY"));
    private String color;
    private int revolverTarget = 0;
    private double revolverPower;
    public enum pattern {
        PPG,
        PGP,
        GPP
    } pattern currentPattern = pattern.PPG;
    private enum shooting {
        shootIdle,
        shootRotating,
        shootEjecting,
        shootFire
    } shooting currentShooting = shooting.shootIdle;

    // ----- misc -----
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    //--gamepad--
    private boolean prevCircle = false;
    private boolean currCircle = false;
    private GamepadEx gamepadEx;

    private boolean inCycle = false;
    @Override
    public void init() {
        // ----- drive -----
        fl = new Motor(hardwareMap,"fl");
        fl.setRunMode(Motor.RunMode.RawPower);
        fl.setInverted(true);
        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl = new Motor(hardwareMap,"bl");
        bl.setRunMode(Motor.RunMode.RawPower);
        bl.setInverted(true);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr = new Motor(hardwareMap,"fr");
        fr.setRunMode(Motor.RunMode.RawPower);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br = new Motor(hardwareMap,"br");
        br.setRunMode(Motor.RunMode.RawPower);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // ----- launcher -----
        launcher1 = new Motor(hardwareMap, "l1", 28, 6000);
        launcher1.setRunMode(Motor.RunMode.RawPower);
        launcher2 = new Motor(hardwareMap, "l2", 28, 6000);
        launcher2.setRunMode(Motor.RunMode.RawPower);
        launcher1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        launcher2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        ff = new PIDFController(Globals.launcher.flykP, Globals.launcher.flykI, Globals.launcher.flykD, Globals.launcher.flykF);
        set = new SimpleServo(hardwareMap, "set", 0, 180, AngleUnit.DEGREES);
        // ----- launcher helpers -----
        lastTime = getRuntime();
        lastPosition = launcher1.getCurrentPosition();

        // ----- revolver -----
        revolver = new Motor(hardwareMap, "revolver", 28, 1150);
        revolver.setRunMode(Motor.RunMode.RawPower);
        revolver.resetEncoder();
        revolverPID = new PIDFController(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD, Globals.revolver.revolverKF);
        // ----- colour sensor -----
        colourSensor = hardwareMap.get(NormalizedColorSensor.class,"colour1");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colour1");
        colourSensor.setGain(2.0f); //CAMERA SENSITIVITY, increase for darker environemnts

        // ----- turret -----
        rotate = new SimpleServo(hardwareMap, "turret", 0, 300, AngleUnit.DEGREES);
        rotate.turnToAngle(turretTarget);
        turretPIDF = new PIDFController(Globals.turret.turretKP, Globals.turret.turretKI, Globals.turret.turretKD, Globals.turret.turretKF);

        // ----- build ov9281 -----
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

        gamepadEx = new GamepadEx(gamepad1);

        intake = new Motor(hardwareMap, "intake");
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        eject = new SimpleServo(hardwareMap, "eject", 0, 70);
        eject.setInverted(true);
        eject.turnToAngle(Globals.pushServo.defualt);

    }

    @Override
    public void loop() { // TODO add revolver sequence logic
        calculateRPM();
        launcherawe();
        autoAimServoMode();      // now only reads/controls; does NOT rebuild vision
        doTelemetry();
        intake();
        drive();

        findPattern();
        launch3();

        revolverPID.setTolerance(0);
        revolverPID.setPIDF(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD, Globals.revolver.revolverKF);
        revolverPower = revolverPID.calculate(revolver.getCurrentPosition(), revolverTarget);
        revolver.set(revolverPower);

    }


    public void launch3() {
            gamepadEx.readButtons();
        //if (revolverReadytoLaunch) {
            // Arm spinner & intake only while we are in a shooting cycle
            boolean startPressed = gamepadEx.getButton(GamepadKeys.Button.CROSS);

            // Start a new cycle
            if (startPressed && !shootLoop) {

                shootLoop = true;
                currentShooting = shooting.shootRotating;

                shootAction = false;
                ejectAction = false;
                rotating = false;
                shotsFired = 0;

            }
            if (!shootLoop) {
                // idle; ensure things are safe
                launcher1.set(0);
                launcher2.set(0);

                set.turnToAngle(Globals.launcher.downset);
                return;
            }
            if (shootLoop) {
                currCircle = gamepadEx.getButton(GamepadKeys.Button.CIRCLE);

                launcher1.set(feedforwardPower);
                launcher2.set(feedforwardPower);

                // shooting cycle state machine
                switch (currentShooting) {
                    case shootRotating: {
                        shootTimer = Double.MAX_VALUE;
                        if (!currCircle && prevCircle) { // POTENTIAL ERROR CUZ THE BUTTON COULD CARRY OVER
                            if (!rotating && shotsFired > 0) {
                                // if true -> +1 -> 2->0 (counterclockwise)
                                oneRotationRevolver(shootCounterClockwise);
                                rotating = true;
                                currentShooting = shooting.shootEjecting;
                            } else if (!rotating && shotsFired == 0) {
                                rotating = true;
                                previousRevolverPosition = revolverTarget;
                                currentShooting = shooting.shootEjecting;

                            }
                        }

                        break;
                    }

                    case shootEjecting: {
                        if (!ejectAction && (currCircle)) {
                            eject.turnToAngle(Globals.pushServo.eject);
                            ejectAction = true;
                        }

                        // retract when driver releases the button, then advance state
                        if (ejectAction && (!currCircle && prevCircle)) {
                            eject.turnToAngle(Globals.pushServo.defualt);
                            currentShooting = shooting.shootFire;
                        }
                        // hold push until timer expires, then retract

                        break;
                    }

                    case shootFire: {
                        // wait for spin-up and aim, then flick
                        boolean atSpeed = Math.abs(power - RPM) < Globals.launcher.launcherTol;

                        if (atSpeed && aligned && shootTimer > globalTimer.seconds() && !shootAction) {
                            set.turnToAngle(Globals.launcher.upset);
                            shootTimer = globalTimer.seconds() + Globals.timers.pushUpTime; // 1s gate-open
                            shootAction = true;
                        }

                        // close gate and finish cycle
                        if (globalTimer.seconds() > shootTimer) {
                            set.turnToAngle(Globals.launcher.downset);
                            shotsFired++;
                            if (shotsFired < 3) {
                                // Prepare next round: rotate again for the next chamber
                                currentShooting = shooting.shootRotating;
                                shootAction = false;
                                ejectAction = false;
                                rotating = false;

                                shootTimer = Double.MAX_VALUE;
                                // (keep launcher + intake running during the burst)
                            } else {
                                // Burst done — shut down
                                currentShooting = shooting.shootIdle;
                                shootLoop = false;
                                revolverReadytoLaunch = false;
                                launcher1.set(0);
                                launcher2.set(0);

                                shootTimer = Double.MAX_VALUE;
                            }
                        }
                        break;
                    }

                    case shootIdle:
                    default: {
                        // Safety – shouldn’t sit here during an active cycle
                        shootLoop = false;
                        launcher1.set(0);
                        launcher2.set(0);

                        break;
                    }

                }
                prevCircle = currCircle;
           // }
        }

    }



    public void ejection() {
        if (gamepad1.right_stick_x > 0.5) {
            eject.turnToAngle(28);
        } else if (gamepad1.right_stick_x < -0.5) {
            eject.turnToAngle(51);
        } else {
            eject.turnToAngle(44);

        }// eject is 30, default is 44, push is 51

    }

    private String senseColour() {
        if (distanceSensor.getDistance(DistanceUnit.CM) > 3) return "EMPTY";

        NormalizedRGBA rgba = colourSensor.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsv); // hsv[0]=H, hsv[1]=S, hsv[2]=V

        if (hsv[0] >= 150 && hsv[0] <= 180 &&
                hsv[1] >= 0.75 && hsv[1] <= 1.00 &&
                hsv[2] > 0.00 && hsv[2] < 0.3) {
            return "G";
        }

        if (hsv[0] >= 220 && hsv[0] <= 250 &&
                hsv[1] >= 0.40 && hsv[1] <= 0.60 &&
                hsv[2] > 0.00 && hsv[2] < 0.3) {
            return "P";
        }
        return "EMPTY";
    }

    private String[] desiredByPattern() {
        // Order: [top (0), left (1), right (2)]
        switch (currentPattern) {
            case PPG: return new String[]{"P","P","G"};
            case PGP: return new String[]{"P","G","P"};
            case GPP: return new String[]{"G","P","P"};
            default:  return new String[]{"EMPTY","EMPTY","EMPTY"};
        }
    }
    public void oneRotationRevolver(boolean left) {
        // PID setup

        // PID setup
        revolverPID.setTolerance(0);
        revolverPID.setPIDF(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD, Globals.revolver.revolverKF);
        previousRevolverPosition = revolverTarget;
        revolverTarget += left ? +Globals.revolver.oneRotation : -Globals.revolver.oneRotation; //TODO Chekc if this is right

    }




    public void intake() {
        revolverPower = revolverPID.calculate(revolver.getCurrentPosition(), revolverTarget);
        revolver.set(revolverPower);

        if (!revolverReady &&
                Math.abs(Math.abs(revolver.getCurrentPosition() - previousRevolverPosition) - Globals.revolver.oneRotation) < 10) {
            revolverReady = true;
        }

        intake.set(gamepadEx.getButton(GamepadKeys.Button.TRIANGLE) ? Globals.intakePower : 0);

        int filled = revolverState.size() - Collections.frequency(revolverState, "EMPTY");
        String color = senseColour();  // "P", "G", or "EMPTY"

        if (gamepadEx.getButton(GamepadKeys.Button.SQUARE) && patternDetected) {
            if (!"EMPTY".equals(color) && revolverReady) {

                String wantTop = desiredByPattern()[0];

                switch (filled) {
                    case 0: {
                        // put new ball into slot 2, then ALWAYS rotate it to slot 1
                        revolverReady = false;
                        revolverState.set(2, color);

                        oneRotationRevolver(false);
                        Collections.rotate(revolverState, -1);

                        previousRotation = false;
                        break;
                    }

                    case 1: {
                        //same idea just move it into slot 1
                        revolverReady = false;
                        revolverState.set(2, color);

                        oneRotationRevolver(false);
                        Collections.rotate(revolverState, -1);
                        previousRotation = false;

                        break;
                    }
                    case 2: {
                        revolverState.set(2, color);

                        if (color.equals(wantTop)) {
                            // new ball IS the one we want on top → bring index 2 -> index 0
                            revolverReady = false;
                            oneRotationRevolver(true);
                            Collections.rotate(revolverState, 1);
                            previousRotation = true;

                        } else if (revolverState.get(0).equals(wantTop)) {
                            // already on top → do nothing
                        } else if (revolverState.get(1).equals(wantTop)) {
                            // we have it in slot 1 → bring 1 -> 0
                            revolverReady = false;
                            oneRotationRevolver(false);
                            Collections.rotate(revolverState, -1);
                            previousRotation = false;
                        } else {

                        }

                        //describes whether to launch clockwise or counter clockwise in launch 3
                        String[] want = desiredByPattern();
                        String secondBall = want[1];
                        if (revolverState.get(1).equals(secondBall)) {
                            shootCounterClockwise = false;
                        } else if (revolverState.get(2).equals(secondBall)) {
                            shootCounterClockwise = true;
                        }



                        // now we are full
                        revolverReadytoLaunch = true;
                        break;
                    }
                    case 3:
                    default: {
                        break;
                    }
                }
            }
        }
    }





    private void findPattern() {
        if (gamepadEx.getButton(GamepadKeys.Button.OPTIONS)) {
            List<AprilTagDetection> code = tagProcessor.getDetections();
            if (code != null && !code.isEmpty()) {
                code.sort(Comparator.comparingDouble((AprilTagDetection d) -> d.decisionMargin).reversed());
                AprilTagDetection best = code.get(0);

                if (best.id == 21) {
                    currentPattern = pattern.GPP; patternDetected = true;
                } else if (best.id == 22) {
                    currentPattern = pattern.PGP; patternDetected = true;
                } else if (best.id == 23) {
                    currentPattern = pattern.PPG; patternDetected = true;
                } else {
                    patternDetected = false;
                    telemetry.addLine("NO PATTERN FOUND");
                }
            }
        }
    }


    private void calculateRPM() {
        double currentTime = getRuntime();
        int currentPosition = launcher1.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        int deltaTicks = currentPosition - lastPosition;

        if (deltaTime > 0.05) {
            double revs = (double) deltaTicks / 28.0; // GoBILDA CPR
            RPM = (revs / deltaTime) * 60.0;

            lastTime = currentTime;
            lastPosition = currentPosition;
        }
    }

    private void autoAimServoMode() {
        turretPIDF.setPIDF(Globals.turret.turretKP, Globals.turret.turretKI, Globals.turret.turretKD, Globals.turret.turretKF);

//        boolean tri = gamepadEx.getButton(GamepadKeys.Button.TRIANGLE);
//        if (tri && !prevTri) {
//            autoAimEnabled = !autoAimEnabled;
//            turretPIDF.reset();
//        }
//        prevTri = tri;

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

                double delta = aligned ? 0.0 : turretPIDF.calculate(err, 0.0);


                turretTarget -= delta; //THIS IS NEGATIVE
            } else {
                aligned = false;

            }

        } else if (!autoAimEnabled && !patternDetected) {
            visionPortal.setProcessorEnabled(tagProcessor, false);
        }

        if (lb ^ rb) {
            aligned = false;
            turretTarget += lb ? +Globals.turret.nudge : -Globals.turret.nudge;

        }

        if (turretTarget > 300) {
            turretTarget = 300;
        } else if (turretTarget < 0) {
            turretTarget = 0;
        }

        rotate.turnToAngle(turretTarget);

    }


    private void launcherawe() {
        ff.setP(Globals.launcher.flykP);
        ff.setI(Globals.launcher.flykI);
        ff.setD(Globals.launcher.flykD);
        ff.setF(Globals.launcher.flykF);



        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection d : detections) {

                if (d != null) {
                    distance = d.ftcPose.range;
                    power = (2547.5 * pow(2.718281828459045, (0.0078 * distance)))/Globals.launcher.launcherTransformation; // here
                }
            }
        } else {
            power = 0;

        }

        feedforwardPower = ff.calculate(RPM, power);


    }


    private void doTelemetry() {
        telemetry.addData("aligned? ", aligned);
        telemetry.addData("pattern?: ", currentPattern);

        telemetry.addLine("HSV")
                .addData("H (deg)", "%.1f", hsv[0])
                .addData("S", "%.3f", hsv[1])
                .addData("V", "%.3f", hsv[2]);

        telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("filled", filled);

        telemetry.addLine("revolver")
                .addData("0", revolverState.get(0))
                .addData("1", revolverState.get(1))
                .addData("2", revolverState.get(2));

//        telemetry.addLine("actual revolver")
//                .addData("0", finalRevolver.get(0))
//                .addData("1", finalRevolver.get(1))
//                .addData("2", finalRevolver.get(2));


        telemetry.addData("ready?", revolverReadytoLaunch);
        telemetry.addData("revolverready?", revolverReady);
        telemetry.addData("color?", color);
        telemetry.addData("shootloop", shootLoop);
        telemetry.addData("shot rotation?: ", currentShooting);
        telemetry.update();

        TelemetryPacket rpmPacket = new TelemetryPacket();
        rpmPacket.put("RPM", RPM);

        TelemetryPacket powerPacket = new TelemetryPacket();
        powerPacket.put("targetRPM", power);

        FtcDashboard.getInstance().sendTelemetryPacket(powerPacket);
        FtcDashboard.getInstance().sendTelemetryPacket(rpmPacket);
    }


    private void drive(){
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_trigger - gamepad1.left_trigger;
        double slowdown = 0.6;
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

        fl.set(frontLeftPower * slowdown);
        bl.set(backLeftPower * slowdown);
        fr.set(frontRightPower * slowdown);
        br.set(backRightPower * slowdown);


    }
}
