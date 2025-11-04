package org.firstinspires.ftc.teamcode.FROGTONOMOUS;

import static java.lang.Math.pow;

import android.graphics.Color;
import android.util.Size;

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
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    private boolean patternDetected = false;
    private static int pattern;
    private PathChain shoot3, eat3, shoot6, eat6, shoot9, eat9, shoot12;


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
        private PIDController revolverPID;
        private NormalizedColorSensor colourSensor;
        private DistanceSensor distanceSensor;
        private final float[] hsv = new float[3];
        private int ballcount = 0;
        private int revolverTarget = 0;
        private double revolverPower;
        private ArrayList<String> froggystomach = new ArrayList<String>(3);//2 is intake side 0 is top
        public intakesubsys(HardwareMap map) {
            intake = new Motor(map, "intake");
            intake.setRunMode(Motor.RunMode.RawPower);
            intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            intake.set(0.0);

            revolver = new Motor(hardwareMap, "revolver", 28, 1150);
            revolver.setRunMode(Motor.RunMode.RawPower);
            revolver.resetEncoder();

            revolverPID = new PIDController(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD);

            froggystomach.add("empty");
            froggystomach.add("empty");
            froggystomach.add("empty");

            colourSensor = hardwareMap.get(NormalizedColorSensor.class,"colour1");
            colourSensor.setGain(2.0f);
            distanceSensor = hardwareMap.get(DistanceSensor.class, "colour1");
        }
        public void intakeon() {
            intake.set(Globals.intakePower);
        }
        public void intakeoff() {
            intake.set(0);
        }
        public void colorsort(int pattern){//1 ppg 2 pgp 3 gpp
            NormalizedRGBA rgba = colourSensor.getNormalizedColors();
            Color.colorToHSV(rgba.toColor(), hsv);
            revolverPID.setTolerance(0);
            revolverPID.setPIDF(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD, Globals.revolver.revolverKF);
            revolverPower = revolverPID.calculate(revolver.getCurrentPosition(), revolverTarget);
            revolver.set(revolverPower);

            if (hsv[0] >= 150 && hsv[0] <= 180 && hsv[1] >= 0.75 && hsv[1] <= 1.00 && hsv[2] > 0.00 && hsv[2] < 0.3) {
                    froggystomach.set(2, "g");
            }
            if (hsv[0] >= 220 && hsv[0] <= 250 && hsv[1] >= 0.40 && hsv[1] <= 0.60 && hsv[2] > 0.00 && hsv[2] < 0.3) {
                    froggystomach.set(2, "p");
                    ;
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
            intake.colorsort(pattern);
        }

        @Override
        public void end(boolean interrupted) {
            intake.intakeoff();
        }
    }

    public class outtakesubsys extends SubsystemBase {
        private Motor launcher1, launcher2;
        private PIDFController turretPIDF, ff;
        private SimpleServo lift, rotate, eject;
        private VisionPortal visionPortal;
        private AprilTagProcessor tagProcessor;
        private double RPM;
        private double lastTime;
        private int lastPosition;
        private boolean aligned = false;
        double turretTarget = 150F;
        private double distance;
        private double power;
        private double feedforwardPower = 0;
        public outtakesubsys(HardwareMap map) {
            launcher1 = new Motor(hardwareMap, "l1", 28, 6000);
            launcher1.setRunMode(Motor.RunMode.RawPower);
            launcher2 = new Motor(hardwareMap, "l2", 28, 6000);
            launcher2.setRunMode(Motor.RunMode.RawPower);
            launcher1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            launcher2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            ff = new PIDFController(Globals.launcher.flykP, Globals.launcher.flykI, Globals.launcher.flykD, Globals.launcher.flykF);
            lift = new SimpleServo(hardwareMap, "set", 0, 180, AngleUnit.DEGREES);
            turretPIDF = new PIDFController(Globals.turret.turretKP, Globals.turret.turretKI, Globals.turret.turretKD, Globals.turret.turretKF);
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
            rotate = new SimpleServo(hardwareMap, "turret", 0, 300, AngleUnit.DEGREES);
            rotate.turnToAngle(0);
            eject = new SimpleServo(hardwareMap, "eject", 0, 70);
            eject.setInverted(true);
            eject.turnToAngle(Globals.pushServo.defualt);
        }

        public void launchalign(){
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

        public void powercalc() {//0 ppg 1 pgp 2 gpp
            ff.setP(Globals.launcher.flykP);
            ff.setI(Globals.launcher.flykI);
            ff.setD(Globals.launcher.flykD);
            ff.setF(Globals.launcher.flykF);

            List<AprilTagDetection> detections = tagProcessor.getDetections();
            if (detections != null && !detections.isEmpty()) {
                for (AprilTagDetection d : detections) {
                    distance = d.ftcPose.range;

                    power = (2547.5 * pow(2.718281828459045, (0.0078 * distance)))/Globals.launcher.launcherTransformation; // here

                }
            } else {
                power = 0;

            }

            feedforwardPower = ff.calculate(RPM, power);
        }

        private void launch(String[] revolver, int pattern) {
            if (pattern == 0) {
                launcher1.set(feedforwardPower);
                launcher2.set(feedforwardPower);


            }
        }


        private void RPM() {
            double currentTime = getRuntime();
            int currentPosition = launcher1.getCurrentPosition();

            double deltaTime = currentTime - lastTime;
            int deltaTicks = currentPosition - lastPosition;

            if (deltaTime > 0.1) {
                double revs = (double) deltaTicks / 28.0; // GoBILDA CPR
                RPM = (revs / deltaTime) * 60.0;

                lastTime = currentTime;
                lastPosition = currentPosition;
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
