package org.firstinspires.ftc.teamcode.FROGTONOMOUS;

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
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

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
        private final Motor intake;
        public intakesubsys(HardwareMap map) {
            intake = new Motor(map, "intake");
            intake.setRunMode(Motor.RunMode.RawPower);
            intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            intake.set(0.0);
        }
        public void increasepower() {
            intake.set(0.7);
        }

        public void decreasepower() {
            intake.set(0.0);
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
            intake.increasepower();
        }

        @Override
        public void end(boolean interrupted) {
            intake.decreasepower();
        }
    }

    public class revolversubsys extends SubsystemBase {
        private final Motor revolver;
        private PIDController revolverPID;

        private ArrayList<String> froggystomach = new ArrayList<String>(3);
        public revolversubsys(HardwareMap map) {
            revolver = new Motor(hardwareMap, "revolver", 28, 1150);
            revolver.setRunMode(Motor.RunMode.RawPower);
            revolver.resetEncoder();
            revolverPID = new PIDController(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD);

            froggystomach.add("empty");
            froggystomach.add("empty");
            froggystomach.add("empty");
        }
        public void sort(int pattern) {
            if (pattern == 1) {

            } else if (pattern == 2) {

            } else if (pattern == 3) {

            }
        }
    }


    @Override
    public void initialize() {
        super.reset();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(914.101, 914.101, 645.664, 342.333)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "ov9281"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(1280, 720))
                .build();

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


        List<AprilTagDetection> code = tagProcessor.getDetections();
        if (code!= null && !code.isEmpty() && !patternDetected) {
            for (AprilTagDetection c : code) {
                if (c.id == 21) {
                    schedule(GPP);
                    patternDetected = true;
                } else if (c.id == 22) {
                    schedule(GPP);
                    patternDetected = true;
                } else if (c.id == 23) {
                    schedule(GPP);
                    patternDetected = true;
                } else {
                    patternDetected = false;
                    telemetry.addLine("NO PATTERN FOUND");
                }
            }
        }
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
