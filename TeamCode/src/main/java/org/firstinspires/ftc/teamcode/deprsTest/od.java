package org.firstinspires.ftc.teamcode.deprsTest;

import static java.lang.Math.pow;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.SimpleServoExtKt;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
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

@TeleOp(name = "odo tele")
public class od extends OpMode {
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
    private double lastTime;
    private int lastPosition;

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
    private boolean notag = true;
    private double range;

    private MecanumDrive drive;
    private GoBildaPinpointDriver pinpoint;
    @Override
    public void init() {

        // ----- drive -----
        fl = new Motor(hardwareMap, "fl");
        fl.setRunMode(Motor.RunMode.RawPower);
        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl = new Motor(hardwareMap, "bl");
        bl.setRunMode(Motor.RunMode.RawPower);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr = new Motor(hardwareMap, "fr");
        fr.setRunMode(Motor.RunMode.RawPower);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br = new Motor(hardwareMap, "br");
        br.setRunMode(Motor.RunMode.RawPower);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        drive = new MecanumDrive(fl, fr, bl, br);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(-3.75, 5.25, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setHeading(0, AngleUnit.RADIANS);
        pinpoint.setPosX(0, DistanceUnit.METER);
        pinpoint.setPosY(0, DistanceUnit.METER);
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
    public void loop() {
        drive();
        pinpointTests();
        autoaimodo();
        telemetry.addData("be", bearing);
        telemetry.addData("range", range);
    }

    public void drive(){
        drive.driveRobotCentric(gamepadEx1.getLeftX(), gamepadEx1.getLeftY(), gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)-gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
    }

    public void pinpointTests(){
        telemetry.addData("x", pinpoint.getPosX(DistanceUnit.METER));
        telemetry.addData("y", pinpoint.getPosY(DistanceUnit.METER));
        telemetry.addData("heading", pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES));
        telemetry.update();
    }
    private void autoaimodo(){
//125 = 180 degrees
        turretTarget = -0.6944* (pinpoint.getHeading(AngleUnit.DEGREES) <= 180 ? pinpoint.getHeading(AngleUnit.DEGREES) : pinpoint.getHeading(AngleUnit.DEGREES) - 360) + 183;
        t1.turnToAngle(turretTarget);
        t2.turnToAngle(turretTarget);


    }
    private void autoaimcam() {

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
                        notag = false;
                        range = d.ftcPose.range;
                        bearing = d.ftcPose.bearing;
                    }
                }
            } else if (!notag){
                notag = true;
                cameraTimer.reset();
            }
        } else if (lb ^ rb) {
            aligned = false;
            turretTarget -= lb ? +Globals.turret.nudge : -Globals.turret.nudge; //THIS IS NEGATIVE
        } else if (!autoAimEnabled) {
            aligned = true;
            power = Globals.targetRPM;
        }
        if (!notag || cameraTimer.seconds() < 1) {
            power = (2547.5 * pow(2.718281828459045, (0.0078 * range))) / Globals.launcher.launcherTransformation; // here
            aligned = Math.abs(bearing) <= Globals.turret.turretTol;
            double delta = aligned ? 0.0 : turretPIDF.calculate(bearing, -Globals.turret.turretLocationError);
            turretTarget += delta; //THIS IS POSITIVE
        } else {
            power = Globals.targetRPM;
            aligned = false;
            bearing = 0;
        }

        if (turretTarget >= 245) {
            turretTarget = 245;
        } else if (turretTarget <= 120) {
            turretTarget = 120;
        }

        t1.turnToAngle(turretTarget);
        t2.turnToAngle(turretTarget);

    }
}
