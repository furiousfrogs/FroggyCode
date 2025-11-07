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
    private boolean revolverReady = true;
    private boolean prevTri = false;
    private boolean previousRotation;
    private boolean revolverReadytoLaunch = false;
    private boolean clockwise;
    private int filled;
    double revolverSetupTimer = Double.MAX_VALUE;
    private boolean revolverting = false;
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

    ////////////////////////////////////////////////

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
                .setLinearHeadingInterpolation(Math.toRadians(-36), Math.toRadians(-180))
                .build();

        shoot6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(21.308, 84.785), new Pose(32.075, 97.121))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-75))
                .build();

        eat6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(32.075, 97.121),
                                new Pose(50.243, 62.804),
                                new Pose(21.533, 60.561)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-75), Math.toRadians(-180))
                .build();

        shoot9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(21.533, 60.561), new Pose(36.336, 92.636))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-75))
                .build();

        eat9 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(36.336, 92.636),
                                new Pose(53.832, 39.477),
                                new Pose(22.430, 36.561)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-75), Math.toRadians(-180))
                .build();

        shoot12 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(22.430, 36.561), new Pose(40.598, 88.598))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(-75))
                .build();
    }

    public void onerotation(boolean left) {
        previousRevolverPosition = revolverTarget;
        revolverTarget += left? +Globals.revolver.oneRotation : -Globals.revolver.oneRotation;
    }
    private void findPattern() {
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
    }//todo

    ///////////////////////////////////////////////

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

            revolverPID.setTolerance(0);
            revolverPID.setPIDF(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD, Globals.revolver.revolverKF);

            distanceSensor = hardwareMap.get(DistanceSensor.class, "colour1");
        }
        public void intakeon() {
            intake.set(Globals.intakePower);
        }
        public void intakeoff() {
            intake.set(0);
        }

        public void sort(boolean left){
            revolverPower = revolverPID.calculate(revolver.getCurrentPosition(), revolverTarget);
            revolver.set(revolverPower);

            if (distanceSensor.getDistance(DistanceUnit.CM) < 2 && !revolverting){
                revolverting = true;
                onerotation(true);

                if (!revolverReady && Math.abs(Math.abs(revolver.getCurrentPosition() - previousRevolverPosition) - Globals.revolver.oneRotation) < 10) {
                    revolverting = false;//detect if the rotation is done
                }



            }
        }



    }
    public static class froggyeat extends CommandBase {
        private final intakesubsys intake;
        public froggyeat(intakesubsys intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            intake.intakeon();
        }

        @Override
        public void execute() {
            //intake.colorsort();
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

        @Override
        public void periodic() {
            calculateRPM();
            launcherawe();
            autoAimServoMode();
        }
    }//todo
    public static class froggyspit extends CommandBase {
        private final outtakesubsys outtake;
        public froggyspit(outtakesubsys outtake) {
            this.outtake = outtake;
            addRequirements(outtake);
        }

        @Override
        public void initialize() {

        }

        @Override
        public void execute() {
        }

        @Override
        public void end(boolean interrupted) {

        }


    }//todo







    @Override
    public void initialize() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19.300, 119.350, -36));
        telemetry.update();

        outtakesubsys sub = new outtakesubsys(hardwareMap);
        register(sub);
        buildPaths();
        //findPattern();


        SequentialCommandGroup PPG = new SequentialCommandGroup(
                new FollowPathCommand(follower, shoot3),

                new WaitCommand(1000), // Wait 1 second
                // First pickup cycle
                new FollowPathCommand(follower, eat3)
//                new ParallelDeadlineGroup(
//                        new FollowPathCommand(follower, eat3),
//                        new froggyeat()
        );
        schedule(PPG);
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}
