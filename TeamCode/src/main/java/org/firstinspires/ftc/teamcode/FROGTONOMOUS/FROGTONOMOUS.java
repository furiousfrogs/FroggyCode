package org.firstinspires.ftc.teamcode.FROGTONOMOUS;

import static java.lang.Math.pow;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandGroupBase;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.deprsTest.twoDriverScrimTele;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.testing.finalLaunch3;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.*;

@Autonomous
@Configurable
public class FROGTONOMOUS extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private PathChain shoot3, eat3, shoot6, eat6, shoot9, eat9, shoot12;
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
    private boolean shootLoop = false;
    public boolean rotating = false;
    private boolean ejectAction = false;
    public boolean shootAction = false;
    private ElapsedTime globalTimer = new ElapsedTime();
    private double ejectTimer;
    private double shootTimer;
    private double rotateTimer;
    private double feedforwardPower = 0;
    private double distance;
    private double power;
    private int shotsFired = 0;
    private double previousRevolverPosition;
    private PIDFController turretPIDF, ff, revolverPID;
    private SimpleServo set, rotate, eject;
    private NormalizedColorSensor colourSensor, secondColourSensor;
    private DistanceSensor distanceSensor, secondDistanceSensor;
    private double RPM;
    private double lastTime;
    private int lastPosition;
    private double bearing = 0.0;
    double turretTarget = 150F; // inital turret angle
    private final float[] hsv1 = new float[3];
    private final float[] hsv2 = new float[3];
    private boolean shootCounterClockwise = false;
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
    public enum shooting {
        shootIdle,
        shootRotating,
        shootEjecting,
        shootFire
    } shooting currentShooting = shooting.shootIdle;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private boolean inCycle = false;


    public void buildPaths() {
        shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.300, 119.350), new Pose(30.505, 111.925))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-36))
                .build();

        eat3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(30.505, 111.925),
                                new Pose(48.000, 90.617),
                                new Pose(21.308, 84.785)
                        )
                )
                .setBrakingStart(0.5)
                .setBrakingStrength(0.2)
                .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-180))
                .build();

        shoot6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(21.308, 84.785), new Pose(32.299, 98.243))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-90))
                .build();

        eat6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(32.299, 98.243),
                                new Pose(50.243, 62.804),
                                new Pose(21.533, 60.561)
                        )
                )
                .setBrakingStart(0.4)
                .setBrakingStrength(0.2)
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180))
                .build();

        shoot9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(21.533, 60.561), new Pose(45.308, 84.785))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-90))
                .build();

        eat9 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(45.308, 84.785),
                                new Pose(53.832, 39.477),
                                new Pose(22.430, 36.561)
                        )
                )
                .setBrakingStart(0.3)
                .setBrakingStrength(0.2)
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-180))
                .build();

        shoot12 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(22.430, 36.561), new Pose(54.505, 75.140))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-90))
                .build();
    }

    // Mechanism commands - replace these with your actual subsystem commands
    public class intakesubsys extends SubsystemBase {
        private final Motor intake, revolver;
        //2 is intake side 0 is top
        public intakesubsys(HardwareMap map) {
            intake = new Motor(map, "intake");
            intake.setRunMode(Motor.RunMode.RawPower);
            intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            intake.set(0.0);

            revolver = new Motor(hardwareMap, "revolver", 28, 1150);
            revolver.setRunMode(Motor.RunMode.RawPower);
            revolver.resetEncoder();

            revolverPID = new PIDController(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD);

            colourSensor = hardwareMap.get(NormalizedColorSensor.class,"colour1");
            colourSensor.setGain(2.0f);
            distanceSensor = hardwareMap.get(DistanceSensor.class, "colour1");

            secondColourSensor = hardwareMap.get(NormalizedColorSensor.class,"colour2");
            secondDistanceSensor = hardwareMap.get(DistanceSensor.class, "colour2");
            secondColourSensor.setGain(2.0f);
        }
        public void intakeon() {
            intake.set(Globals.intakePower);
        }
        public void oneRotationRevolver(boolean left) {
            // PID setup

            // PID setup
            revolverPID.setTolerance(0);
            revolverPID.setPIDF(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD, Globals.revolver.revolverKF);
            previousRevolverPosition = revolverTarget;
            revolverTarget += left ? +Globals.revolver.oneRotation : -Globals.revolver.oneRotation; //TODO Chekc if this is right

        }
        public void intakeoff() {
            intake.set(0);
        }
        public String senseColour(){//1 ppg 2 pgp 3 gpp
            if (distanceSensor.getDistance(DistanceUnit.CM) > 3 && secondDistanceSensor.getDistance(DistanceUnit.CM) > 3) return "EMPTY";

            NormalizedRGBA rgba1 = colourSensor.getNormalizedColors();
            Color.colorToHSV(rgba1.toColor(), hsv1); // hsv[0]=H, hsv[1]=S, hsv[2]=V

            NormalizedRGBA rgba2 = secondColourSensor.getNormalizedColors();
            Color.colorToHSV(rgba2.toColor(), hsv2); // hsv[0]=H, hsv[1]=S, hsv[2]=V
            if ( (hsv1[0] >= 150 && hsv1[0] <= 180 &&
                    hsv1[1] >= 0.75 && hsv1[1] <= 1.00 &&
                    hsv1[2] > 0.00 && hsv1[2] < 0.3) ||

                    (hsv2[0] >= 150 && hsv2[0] <= 180 &&
                            hsv2[1] >= 0.75 && hsv2[1] <= 1.00 &&
                            hsv2[2] > 0.00 && hsv2[2] < 0.3)) {
                return "G";
            } // if colour one or colour two return green/purpler

            if ((hsv1[0] >= 220 && hsv1[0] <= 250 &&
                    hsv1[1] >= 0.40 && hsv1[1] <= 0.60 &&
                    hsv1[2] > 0.00 && hsv1[2] < 0.3) ||

                    (hsv2[0] >= 220 && hsv2[0] <= 250 &&
                            hsv2[1] >= 0.40 && hsv2[1] <= 0.60 &&
                            hsv2[2] > 0.00 && hsv2[2] < 0.3)) {
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
        public void colorsort() {
            revolverPID.setTolerance(0);
            revolverPID.setPIDF(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD, Globals.revolver.revolverKF);
            revolverPower = revolverPID.calculate(revolver.getCurrentPosition(), revolverTarget);
            revolver.set(revolverPower);

            if (!revolverReady &&
                    Math.abs(Math.abs(revolver.getCurrentPosition() - previousRevolverPosition) - Globals.revolver.oneRotation) < 10) {
                revolverReady = true;
            }

            int filled = revolverState.size() - Collections.frequency(revolverState, "EMPTY");
            String color = senseColour();

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
    public static class froggyactions extends CommandBase {
        private final intakesubsys intake;
        public froggyactions(intakesubsys intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            intake.intakeon();
        }

        @Override
        public void execute() {
            intake.colorsort();
        }

        @Override
        public void end(boolean interrupted) {
            intake.intakeoff();
        }
    }

    public class outtakesubsys extends SubsystemBase {
        private Motor launcher1, launcher2, revolver;

        public outtakesubsys(HardwareMap map) {
            launcher1 = new Motor(hardwareMap, "l1", 28, 6000);
            launcher1.setRunMode(Motor.RunMode.RawPower);
            launcher2 = new Motor(hardwareMap, "l2", 28, 6000);
            launcher2.setRunMode(Motor.RunMode.RawPower);
            launcher1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            launcher2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            ff = new PIDFController(Globals.launcher.flykP, Globals.launcher.flykI, Globals.launcher.flykD, Globals.launcher.flykF);
            set = new SimpleServo(hardwareMap, "set", 0, 180, AngleUnit.DEGREES);
            lastTime = getRuntime();
            lastPosition = launcher1.getCurrentPosition();
            revolver = new Motor(hardwareMap, "revolver", 28, 1150);
            revolver.setRunMode(Motor.RunMode.RawPower);
            revolver.resetEncoder();
            revolverPID = new PIDController(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD);
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
            eject = new SimpleServo(hardwareMap, "eject", 0, 70);
            eject.setInverted(true);
            eject.turnToAngle(Globals.pushServo.defualt);
        }
        public void oneRotationRevolver(boolean left) {
            revolverPID.setTolerance(0);
            revolverPID.setPIDF(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD, Globals.revolver.revolverKF);
            previousRevolverPosition = revolverTarget;
            revolverTarget += left ? +Globals.revolver.oneRotation : -Globals.revolver.oneRotation; //TODO Chekc if this is right

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

            List<AprilTagDetection> detections = tagProcessor.getDetections();
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

                    if (d != null && d.metadata != null && d.ftcPose != null) {
                        distance = d.ftcPose.range;
                        power = (2547.5 * pow(2.718281828459045, (0.0078 * distance)))/Globals.launcher.launcherTransformation; // here
                    }
                }
            } else {
                power = 0;

            }

            feedforwardPower = ff.calculate(RPM, power);


        }

        public void launch3() {
            revolverReadytoLaunch = true;
            if (revolverReadytoLaunch) {
                if (!shootLoop) {
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
                    currCircle = gamepadEx2.getButton(GamepadKeys.Button.CIRCLE);

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
                                    revolverState.set(0, "EMPTY");
                                    revolverState.set(1, "EMPTY");
                                    revolverState.set(2, "EMPTY");
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
                }
            }

        }
    }







    @Override
    public void initialize() {
        super.reset();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19.300, 119.350));
        buildPaths();


        SequentialCommandGroup GPP = new SequentialCommandGroup(
                // Score preload
                new FollowPathCommand(follower, shoot3),

                new WaitCommand(1000), // Wait 1 second
                // First pickup cycle
                new FollowPathCommand(follower, eat3)// Sets globalMaxPower to 50% for all future paths
                // (unless a custom maxPower is given)
//                new ParallelDeadlineGroup(
//                        new FollowPathCommand(follower, eat3),
//                        new froggyeat()
//                )



//                // Second pickup cycle
//                new FollowPathCommand(follower, grabPickup2),
//                grabSample(),
//                new FollowPathCommand(follower, scorePickup2, 1.0), // Overrides maxPower to 100% for this path only
//                scoreSample(),
//
//                // Park
//                new FollowPathCommand(follower, park, false), // park with holdEnd false
//                level1Ascent()
        );
        schedule(GPP);
    }

    @Override
    public void run() {
        super.run();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}
