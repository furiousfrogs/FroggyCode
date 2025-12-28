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
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
    private double previousPreviousRPM = 0;
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

    }

    shoot3 currentshoot3 = shoot3.idle;


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
    Limelight3A limelight;
    Pose2D pose2D;
    private boolean clockwise;

    @Override
    public void init() {

        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
//
        //limelight.setPollRateHz(20);
//
        //limelight.pipelineSwitch(0);
//
        //limelight.start();
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
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(-3.75, 5.25, DistanceUnit.INCH);

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 33, 135, AngleUnit.DEGREES, 90));
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
        gate = new SimpleServo(hardwareMap, "gate", 0, 300, AngleUnit.DEGREES);
        gate.turnToAngle(Globals.closeGate);

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
        //limelight();
        calculateRPM();
        intakeSystem();
        drive();
        revolver();
        launch3();
        tangentVelocity();


        feedforwardPower = ff.calculate(RPM, power);
//        telemetry.addData("be", bearing);
//        telemetry.addData("range", range);
    }

    public void limelight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            if (result.getStaleness() < 500) {

                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                if (!tags.isEmpty()) {

                    for (LLResultTypes.FiducialResult tag : tags) {

                        int id = tag.getFiducialId();

                        if (id == 20) {
                            telemetry.addData("tx", result.getTx());
                            telemetry.addData("ty", result.getTy());
                            Pose3D botpose = result.getBotpose();
                            telemetry.addData("Botpose", botpose.toString());

                        }

                    }

                }

            }
        }
    }

    public void drive() {
        if (gamepadEx1.getButton(GamepadKeys.Button.X)) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 33, 135, AngleUnit.DEGREES, 90));
        }
        drive.driveRobotCentric(-gamepadEx1.getLeftX(), -gamepadEx1.getLeftY(), gamepadEx1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepadEx1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
    }

    public void pinpointTests() {
        telemetry.update();

        pose2D = pinpoint.getPosition();
        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));
        pinpoint.update();
    }
    private void autoaimodo() {
        double targetAngle = 5 + Math.toDegrees(Math.atan2(144 - pose2D.getY(DistanceUnit.INCH), pose2D.getX(DistanceUnit.INCH)));
        double fieldAngle = 180 - targetAngle;
        double robotFieldHeading = pose2D.getHeading(AngleUnit.DEGREES) < 0 ? 180 + (180 - Math.abs(pose2D.getHeading(AngleUnit.DEGREES))) : pose2D.getHeading(AngleUnit.DEGREES);
        if (robotFieldHeading < fieldAngle + 90 && robotFieldHeading > fieldAngle - 90) {
            turretTarget = 0.6944 * (robotFieldHeading - fieldAngle) + 183;
        } else {
            turretTarget = 183;
        }
        t1.turnToAngle(turretTarget);
        t2.turnToAngle(turretTarget);

        dist = Math.pow(Math.pow(pose2D.getX(DistanceUnit.METER) - 16, 2) + Math.pow(135 - pose2D.getY(DistanceUnit.METER), 2), 0.5);
        power = (2547.5 * pow(2.718281828459045, (0.0078 * dist))) / Globals.launcher.launcherTransformation;
        aligned = robotFieldHeading < fieldAngle + 90 && robotFieldHeading > fieldAngle - 90;

        if (turretTarget >= 245) {
            turretTarget = 245;
        } else if (turretTarget <= 120) {
            turretTarget = 120;
        }

    }

    private void tangentVelocity() {
        double targetAngle = 5 + Math.toDegrees(Math.atan2(144 - pose2D.getY(DistanceUnit.INCH), pose2D.getX(DistanceUnit.INCH)));
        double fieldAngle = 180 - targetAngle;
        double robotFieldHeading = pose2D.getHeading(AngleUnit.DEGREES) < 0 ? 180 + (180 - Math.abs(pose2D.getHeading(AngleUnit.DEGREES))) : pose2D.getHeading(AngleUnit.DEGREES);
        double netVelocityMagnitude = Math.sqrt(Math.pow(pinpoint.getVelX(DistanceUnit.METER), 2) + Math.pow(pinpoint.getVelY(DistanceUnit.METER), 2));
        double netVelocityAngle = Math.toDegrees(Math.atan2(pinpoint.getVelY(DistanceUnit.METER), pinpoint.getVelX(DistanceUnit.METER)));
        double angle = Math.abs(netVelocityAngle - fieldAngle);
        double tangentVelocity = netVelocityMagnitude * Math.sin(Math.toDegrees(angle));
        double normalVelocity = netVelocityMagnitude * Math.cos(Math.toDegrees(angle));
        if (netVelocityAngle < fieldAngle) {
            clockwise = true;
        } else { clockwise = false; }
        telemetry.addData("clockwise", clockwise);
        telemetry.addData("normalVelocity", normalVelocity);
        telemetry.addData("tangentVelocity", tangentVelocity);
        telemetry.addData("netVelocityMagnitude", netVelocityMagnitude);
        telemetry.addData("netVelocityAngle", netVelocityAngle);
        telemetry.update();
    }
    private void calculateRPM() {
        double currentTime = getRuntime();
        int currentPosition = launcher1.getCurrentPosition();

        double deltaTime = currentTime - lastTime;
        int deltaTicks = currentPosition - lastPosition;

        if (deltaTime > 0.02) {
            previousRPM = RPM;
            double revs = (double) deltaTicks / 28.0; // GoBILDA CPR
            RPM = (revs / deltaTime) * 60.0;

            lastTime = currentTime;
            lastPosition = currentPosition;
        }
    }
    public void launch3() {
        ang = (ejectAnalog.getVoltage()/3.3) * 360;
        dist = launchDistanceSensor.getDistance(DistanceUnit.CM);

        if (gamepadEx2.getButton(GamepadKeys.Button.CROSS) && !prevCross) {
            if (!shootcycle) {
                currentshoot3 =  shoot3.pushin;
                eject.turnToAngle(Globals.pushServo.defualt);
                set.turnToAngle(Globals.launcher.downset);
                pushupTimer.reset();


                shootcycle = true;
                rotated = false;
            } else {
                shootcycle = false;
                currentshoot3 =  shoot3.idle;
                eject.turnToAngle(Globals.pushServo.defualt);
                set.turnToAngle(Globals.launcher.downset);

                launcher1.set(0);
                launcher2.set(0);
            }
        }prevCross = gamepadEx2.getButton(GamepadKeys.Button.CROSS);
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
                                currentshoot3 =  shoot3.rotate;


                            }
                        }
                        break;

                    case rotate: //TODO fix this

                        eject.turnToAngle(Globals.pushServo.defualt);

                        if (ang < 168 && !rotated) {
                            oneRotationRevolver(true);
                            Collections.rotate(revolverState, 1);
                            revolverTimer.reset();
                            revolverReady = false;
                            rotated = true;

                        }
                        if (Math.abs(Math.abs(revolver.getCurrentPosition() - previousRevolverPosition) - Globals.revolver.oneRotation) < 10 && rotated) {
                            currentshoot3 =  shoot3.setup;
                        } else if (rotated && revolverTimer.seconds() > 1 && Math.abs(revolver.getCurrentPosition() - revolverTarget) < 90){
                            currentshoot3 =  shoot3.rotateFailed;
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
                                currentshoot3 =  shoot3.rotate;
                            }
                        }
                        break;

                    case setup:
                        if (aligned && Math.abs(power - RPM) < Globals.launcher.launcherTol) {
                            set.turnToAngle(Globals.launcher.upset);
                        }
                        if (Math.abs(previousRPM - RPM) > 300 && set.getPosition() > 0.5) {
                            currentshoot3 =  shoot3.pushin;
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

                currentshoot3 =  shoot3.idle;
                shootcycle = false;
                launcher1.set(0);
                launcher2.set(0);
            }


        } else {
            gate.turnToAngle(Globals.closeGate);
        }


    }
    public void intakeSystem() {
        revolverPID.setTolerance(0);
        revolverPID.setPIDF(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD, Globals.revolver.revolverKF);
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
    public void oneRotationRevolver(boolean left) {
        // PID setup

        // PID setup
        revolverPID.setTolerance(0);
        revolverPID.setPIDF(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD, Globals.revolver.revolverKF);
        previousRevolverPosition = revolver.getCurrentPosition();
        revolverTarget += left ? +Globals.revolver.oneRotation : -Globals.revolver.oneRotation;

    }
    public void revolver() {
        double cDist1 = distanceSensor.getDistance(DistanceUnit.CM);
        double cDist2 = secondDistanceSensor.getDistance(DistanceUnit.CM);
        boolean ballExists = cDist1 < 3 || cDist2 < 3;

        if (!revolverReady &&
                Math.abs(Math.abs(revolver.getCurrentPosition() - previousRevolverPosition) - Globals.revolver.oneRotation) < 10) {
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

        if (!shootcycle) {// "P", "G", or "EMPTY"

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



}
