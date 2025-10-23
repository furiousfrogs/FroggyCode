package org.firstinspires.ftc.teamcode.FROGTONOMOUS;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class FROGTONOMOUS extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private boolean patternDetected = false;
    private final Pose startPose = new Pose(9, 111, Math.toRadians(-90));
    private final Pose scorePose = new Pose(16, 128, Math.toRadians(-45));
    private final Pose pickup1Pose = new Pose(30, 121, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(30, 131, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(45, 128, Math.toRadians(90));
    private final Pose parkPose = new Pose(68, 96, Math.toRadians(-90));
    private DcMotor intake, outtake;
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
    private InstantCommand openOuttakeClaw() {
        return new InstantCommand(() -> {
            // Example: outtakeSubsystem.openClaw();
        });
    }

    private InstantCommand grabSample() {
        return new InstantCommand(() -> {
            // Example: intakeSubsystem.grabSample();
        });
    }

    private InstantCommand scoreSample() {
        return new InstantCommand(() -> {
            // Example: outtakeSubsystem.scoreSample();
        });
    }

    private InstantCommand level1Ascent() {
        return new InstantCommand(() -> {
            // Example: hangSubsystem.level1Ascent();
        });
    }

    /**
     * This method is called once when the OpMode is initialized.
     * It sets up the robot's hardware, initializes the path-following controller,
     * builds all the autonomous paths, and constructs the command sequence that
     * the robot will execute during the autonomous period. The command sequence
     * is then scheduled to run.
     */
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
        follower.setStartingPose(startPose);
        buildPaths();


        SequentialCommandGroup GPP = new SequentialCommandGroup(
                // Score preload
                new FollowPathCommand(follower, shoot3),
                openOuttakeClaw(),
                new WaitCommand(1000), // Wait 1 second

                // First pickup cycle
                new FollowPathCommand(follower, eat3), // Sets globalMaxPower to 50% for all future paths
                // (unless a custom maxPower is given)
                grabSample()


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

    /**
     * This method is called repeatedly in a loop while the OpMode is running.
     * It handles the main execution of the OpMode's logic after initialization.
     * In this implementation, it first calls the superclass's run method to
     * execute scheduled commands. Then, it adds the robot's current position (X, Y)
     * and heading from the follower to the telemetry and updates the display on the
     * Driver Station.
     */
    @Override
    public void run() {
        super.run();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.update();
    }
}
