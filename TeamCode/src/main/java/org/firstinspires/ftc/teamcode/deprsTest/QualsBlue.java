package org.firstinspires.ftc.teamcode.deprsTest;

import static java.lang.Math.pow;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.SimpleServoExtKt;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@TeleOp(name = "Blue Qual")
public class QualsBlue extends OpMode {

    // ----- booleans/toggles
    private boolean aligned = false;
    private boolean autoAimEnabled = true;
    private boolean revolverReady = true;
    private boolean revolverOn = false;
    private boolean shootcycle = false;
    private boolean rotated = false;


    // ---- prev gamepad's
    private boolean prevPS = false;
    private boolean prevPad = false;
    private boolean prevCircle;
    private boolean prevCross;
    private boolean prevSquare2 = false;

    // ----- doubles -----
    private double feedforwardPower = 0;
    private double power;
    private double previousRevolverPosition;
    private double revolverPower;
    private double dist;
    private double ang;
    private int revolverTarget = 0;
    double bearing = 0;

    // ----- pid's -----
    private PIDFController turretPIDF, ff, revolverPID;

    // ----- Motors, servos, sensors -----
    private Motor launcher1, launcher2, revolver, fl, bl, fr, br, intake;
    private SimpleServo set, t1, t2, eject, gate;
    private NormalizedColorSensor colourSensor, secondColourSensor;
    private DistanceSensor launchDistanceSensor, distanceSensor, secondDistanceSensor;

    // ----- launcher -----
    private double RPM;
    private double previousRPM = 0;
    private double previouspreviosRPM = 0;
    private double lastTime;
    private int lastPosition;
    private double slopeLaunch1;
    private double slopeLaunch2;

    // ----- turret -----
    double turretTarget = 183F; // inital turret angle


    //index 0 is the top, 1 is the left, 2 is the right when looking from the front view
    private List<String> revolverState = new ArrayList<>(Arrays.asList("EMPTY", "EMPTY", "EMPTY"));

    private enum shoot3 {
        idle,
        pushin,
        rotate,
        rotateFailed,
        setup,

    } shoot3 currentshoot3 = shoot3.idle;

    // ---- misc imports ----
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private GamepadEx gamepadEx1, gamepadEx2;
    private AnalogInput ejectAnalog;

    // ---- timers ----
    private ElapsedTime pushupTimer = new ElapsedTime();
    private ElapsedTime revolverTimer = new ElapsedTime();
    private ElapsedTime cameraTimer = new ElapsedTime();
    private boolean noTag = true;
    private double range = 20;
    @Override
    public void init() {
        // ----- drive -----
        fl = new Motor(hardwareMap, "fl");
        fl.setRunMode(Motor.RunMode.RawPower);
        fl.setInverted(true);
        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl = new Motor(hardwareMap, "bl");
        bl.setRunMode(Motor.RunMode.RawPower);
        bl.setInverted(true);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr = new Motor(hardwareMap, "fr");
        fr.setRunMode(Motor.RunMode.RawPower);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br = new Motor(hardwareMap, "br");
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
        set = new SimpleServo(hardwareMap, "set", 0, 300, AngleUnit.DEGREES);
        set.turnToAngle(Globals.launcher.downset);

        // ----- launcher helpers -----
        lastTime = getRuntime();
        lastPosition = launcher1.getCurrentPosition();

        // ----- revolver -----
        revolver = new Motor(hardwareMap, "revolver", 28, 1150);
        revolver.setRunMode(Motor.RunMode.RawPower);
        revolver.resetEncoder();
        revolverPID = new PIDFController(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD, Globals.revolver.revolverKF);

        // ----- sensors -----
        launchDistanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colour1");
        secondDistanceSensor = hardwareMap.get(DistanceSensor.class, "colour2");

        // ----- turret -----
        t1 = new SimpleServo(hardwareMap, "t1", 90, 270, AngleUnit.DEGREES);
        t1.turnToAngle(turretTarget);
        t2 = new SimpleServo(hardwareMap, "t2", 90, 270, AngleUnit.DEGREES);
        t2.turnToAngle(turretTarget);

        turretPIDF = new PIDFController(Globals.turret.turretKP, Globals.turret.turretKI, Globals.turret.turretKD, Globals.turret.turretKF);


        // ---- gate ----
        gate = new SimpleServo(hardwareMap,"gate", 0, 300, AngleUnit.DEGREES);
        gate.turnToAngle(Globals.closeGate);

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
        gamepadEx2 = new GamepadEx(gamepad2);
        gamepadEx1 = new GamepadEx(gamepad1);

        // ---- intake ----
        intake = new Motor(hardwareMap, "intake");
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // ---- eject servo ----
        eject = new SimpleServo(hardwareMap, "eject", 0, 70);
        eject.setInverted(true);
        eject.turnToAngle(Globals.pushServo.defualt);
        ejectAnalog = hardwareMap.get(AnalogInput.class, "ejectAnalog");  // REAL SENSOR
    }

    @Override
    public void loop() { // TODO add revolver sequence logic
        calculateRPM();
        autoAimServoMode();      // now only reads/controls; does NOT rebuild vision
        doTelemetry();
        intakeSystem();
        drive();
        revolver();
        launch3();



        feedforwardPower = ff.calculate(RPM, power);
        t1.turnToAngle(turretTarget);
        t2.turnToAngle(turretTarget);

    }

    public void launch3() {
        ang = (ejectAnalog.getVoltage()/3.3) * 360;
        dist = launchDistanceSensor.getDistance(DistanceUnit.CM);

            if (gamepadEx2.getButton(GamepadKeys.Button.CROSS) && !prevCross) {
                if (!shootcycle) {
                    currentshoot3 = shoot3.pushin;
                    eject.turnToAngle(Globals.pushServo.defualt);
                    set.turnToAngle(Globals.launcher.downset);
                    pushupTimer.reset();


                    shootcycle = true;
                    rotated = false;
                } else {
                    shootcycle = false;
                    currentshoot3 = shoot3.idle;
                    eject.turnToAngle(Globals.pushServo.defualt);
                    set.turnToAngle(Globals.launcher.downset);

                    launcher1.set(0);
                    launcher2.set(0);
                }
            } prevCross = gamepadEx2.getButton(GamepadKeys.Button.CROSS);
            if (shootcycle) {
                gate.turnToAngle(Globals.openGate);
                if (Collections.frequency(revolverState, "EMPTY") < 3) {
                launcher1.set(feedforwardPower);
                launcher2.set(feedforwardPower);
                switch (currentshoot3) {
                    case pushin:
                        if (pushupTimer.seconds() > 0.5) {
                            eject.turnToAngle(Globals.pushServo.eject);
                            if ((ang > 176 && ang < 210)|| dist < 5.5) {
                                eject.turnToAngle(Globals.pushServo.defualt);
                                rotated = false;
                                currentshoot3 = shoot3.rotate;
                            }
                        }
                        break;

                    case rotate:
                        eject.turnToAngle(Globals.pushServo.defualt);
                        if (ang < 168 && !rotated) {
                            oneRotationRevolver(true);
                            Collections.rotate(revolverState, 1);
                            revolverTimer.reset();
                            revolverReady = false;
                            rotated = true;

                        }
                        if (Math.abs(Math.abs(revolver.getCurrentPosition() - previousRevolverPosition) - Globals.revolver.oneRotation) < 10 && rotated) {
                            currentshoot3 = shoot3.setup;
                        } else if (rotated && revolverTimer.seconds() > 1 && Math.abs(revolver.getCurrentPosition() - revolverTarget) < 90){
                            currentshoot3 = shoot3.rotateFailed;
                            rotated = false;
                        }
                        break;
                    case rotateFailed:
                        if (!rotated) {
                            rotated = true;
                            oneRotationRevolver(false);
                            Collections.rotate(revolverState, -1);
                        }
                        if (Math.abs(Math.abs(revolver.getCurrentPosition() - previousRevolverPosition) - Globals.revolver.oneRotation) < 10 && rotated) {
                            eject.turnToAngle(Globals.pushServo.eject);
                            if ((ang > 180 && ang < 210) || dist < 5.5) {
                                rotated = false;
                                currentshoot3 = shoot3.rotate;
                            }
                        }
                        break;

                    case setup:
                        if (aligned && Math.abs(power - RPM) < Globals.launcher.launcherTol) {
                            set.turnToAngle(Globals.launcher.upset);
                        }
                        if (Math.abs(previousRPM - RPM) > 300 && set.getPosition() > 0.5) {
                            currentshoot3 = shoot3.pushin;
                            revolverState.set(0, "EMPTY");
                            set.turnToAngle(Globals.launcher.downset);
                            pushupTimer.reset();

                            rotated = false;
                        }
                        break;
                    default:

                        break;
                }

                } else {

                    currentshoot3 = shoot3.idle;
                    shootcycle = false;
                    launcher1.set(0);
                    launcher2.set(0);
                }


            } else {
                gate.turnToAngle(Globals.closeGate);
            }


    }
    public void oneRotationRevolver(boolean left) {

        revolverPID.setTolerance(10);
        revolverPID.setPIDF(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD, Globals.revolver.revolverKF);
        previousRevolverPosition = revolver.getCurrentPosition();
        revolverTarget += left ? +Globals.revolver.oneRotation : -Globals.revolver.oneRotation;

    }

    public void intakeSystem() {

        revolverPower = revolverPID.calculate(revolver.getCurrentPosition(), revolverTarget);
        revolver.set(revolverPower);
        revolverTarget += (int) ((gamepadEx2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepadEx2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) * Globals.revolver.revolverNudge);

        if (gamepadEx1.getButton(GamepadKeys.Button.TRIANGLE) || gamepadEx2.getButton(GamepadKeys.Button.TRIANGLE)) {
            intake.set(Globals.intakePower);
        } else if (gamepadEx1.getButton(GamepadKeys.Button.SQUARE)) {
            intake.set(-Globals.intakePower);
        } else {
            intake.set(0);
        }
    }
    public void revolver() {
        double cDist1 = distanceSensor.getDistance(DistanceUnit.CM);
        double cDist2 = secondDistanceSensor.getDistance(DistanceUnit.CM);
        boolean ballExists = cDist1 < 3 || cDist2 < 3;

        if (!revolverReady &&
            Math.abs(Math.abs(
            revolver.getCurrentPosition() - previousRevolverPosition)
            - Globals.revolver.oneRotation) < 10) {

            revolverReady = true;
        }


        if (gamepadEx2.getButton(GamepadKeys.Button.CIRCLE) && !revolverOn && !prevCircle) {
            revolverState.set(0, "P");
            revolverState.set(1, "P");
            revolverState.set(2, "P");
        }

        if (gamepadEx2.getButton(GamepadKeys.Button.PS) && !revolverOn && !prevPS) {
            revolverState.set(0, "EMPTY");
            revolverState.set(1, "EMPTY");
            revolverState.set(2, "EMPTY");
        }
        prevPS = gamepadEx2.getButton(GamepadKeys.Button.PS);
        prevCircle = gamepadEx2.getButton(GamepadKeys.Button.CIRCLE);

        if (!shootcycle) {
            if (gamepadEx2.getButton(GamepadKeys.Button.SQUARE) && !prevSquare2 && revolverReady) {
                revolverReady = false;
                oneRotationRevolver(true);
                Collections.rotate(revolverState, 1);
            }
            if (ballExists && revolverReady && Collections.frequency(revolverState, "P") < 3 && !gamepadEx2.getButton(GamepadKeys.Button.SQUARE)) {
                revolverReady = false;
                revolverState.set(2, "P");
                oneRotationRevolver(true);
                Collections.rotate(revolverState, 1);
            }
        } prevSquare2 = gamepadEx2.getButton(GamepadKeys.Button.SQUARE);
    }
    private void calculateRPM() {
        double currentTime = getRuntime();
        int currentPosition = launcher1.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        int deltaTicks = currentPosition - lastPosition;


        slopeLaunch1 = (RPM - previousRPM)/0.02;
        slopeLaunch2 = (RPM - previouspreviosRPM)/0.04;
        if (deltaTime > 0.02) {
            previouspreviosRPM = previousRPM;
            previousRPM = RPM;
            double revs = (double) deltaTicks / 28.0; // GoBILDA CPR
            RPM = (revs / deltaTime) * 60.0;
            lastTime = currentTime;
            lastPosition = currentPosition;
        }
    }
    private void autoAimServoMode() {


        ff.setPIDF(Globals.launcher.flykP, Globals.launcher.flykI, Globals.launcher.flykD, Globals.launcher.flykF);
        turretPIDF.setPIDF(Globals.turret.turretKP, Globals.turret.turretKI, Globals.turret.turretKD, Globals.turret.turretKF);

        boolean pad = gamepadEx2.getButton(GamepadKeys.Button.DPAD_DOWN);
        if (pad && !prevPad) {
            if (autoAimEnabled) {
                visionPortal.setProcessorEnabled(tagProcessor, false);
                autoAimEnabled = false;

            } else {
                visionPortal.setProcessorEnabled(tagProcessor, true);
                autoAimEnabled = true;
            }
        }
        prevPad = pad;

        boolean lb = gamepad2.left_bumper;
        boolean rb = gamepad2.right_bumper;

        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (!(lb || rb) && autoAimEnabled) {
            if (detections != null && !detections.isEmpty()) {
                for (AprilTagDetection d : detections) {
                    if (d.ftcPose != null) {
                        range = d.ftcPose.range;
                        bearing = d.ftcPose.bearing;
                        noTag = false;
                    }
                }
            } else if (!noTag) {
                noTag = true;
                cameraTimer.reset();
            }

        } else if (lb ^ rb) {
            aligned = false;
            turretTarget -= lb ? +Globals.turret.nudge : -Globals.turret.nudge; //THIS IS NEGATIVE
        } else if (!autoAimEnabled) {
            aligned = true;
            power = Globals.targetRPM;
        }

        if (!noTag || cameraTimer.seconds() < 1) {
            power = (2547.5 * pow(2.718281828459045, (0.0078 * range))) / Globals.launcher.launcherTransformation;
            aligned = Math.abs(bearing) <= Globals.turret.turretTol;
            double delta = aligned ? 0.0 : turretPIDF.calculate(bearing, -Globals.turret.turretLocationError);
            turretTarget += delta;
        } else {
            power = Globals.targetRPM;
            aligned = false;
        }
        if (turretTarget >= 245) {
            turretTarget = 245;
        } else if (turretTarget <= 120) {
            turretTarget = 120;
        }
    }



    private void doTelemetry() {
//        telemetry.addLine("revolver")
//                .addData("0", revolverState.get(0))
//                .addData("1", revolverState.get(1))
//                .addData("2", revolverState.get(2));
//        telemetry.addData("# balls", Collections.frequency(revolverState, "P"));
//        telemetry.addLine(" ");
//        telemetry.addData("aligned? ", aligned);
//        telemetry.addData("autoaim enabled? ", autoAimEnabled);
//        telemetry.addLine(" ");
//        telemetry.addData("launch cycle, ", currentshoot3);
//
//        telemetry.addData("ang ", (ejectAnalog.getVoltage()/3.3) * 360);
//        telemetry.addData("dist ", launchDistanceSensor.getDistance(DistanceUnit.CM));
//        telemetry.update();

        telemetry.addData("bear", bearing);
        telemetry.addData("powe", power);
        telemetry.addData("sl1", slopeLaunch1);
        telemetry.addData("se2", slopeLaunch2);

        TelemetryPacket rpmPacket = new TelemetryPacket();
        rpmPacket.put("RPM", RPM);

        TelemetryPacket powerPacket = new TelemetryPacket();
        powerPacket.put("targetRPM", power);

        FtcDashboard.getInstance().sendTelemetryPacket(powerPacket);
        FtcDashboard.getInstance().sendTelemetryPacket(rpmPacket);
    }


    private void drive() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_trigger - gamepad1.left_trigger;
        double slowdown = Globals.slowdown;
        if (Math.abs(y) < 0.2) {
            y = 0.0;
        }
        if (Math.abs(x) < 0.2) {
            x = 0.0;
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