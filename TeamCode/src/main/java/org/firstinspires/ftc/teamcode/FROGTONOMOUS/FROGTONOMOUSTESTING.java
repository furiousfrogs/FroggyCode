package org.firstinspires.ftc.teamcode.FROGTONOMOUS;
import static java.lang.Math.addExact;
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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.robocol.Command;
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
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.*;


//TODO SHOOTING PATHING LIMELIGHT PATTERN DETECT CHECK INTAKE
@Autonomous
@Configurable
public class FROGTONOMOUSTESTING extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private PathChain shoot3, eat3, eat3rotate, shoot6, eat6, eat6rotate, shoot9, escape;
    private boolean aligned = false;
    private int revolverindex = 0;
    private boolean launchfinished = false;
    private boolean cumdown = false;
    private double ang;
    private double previousRPM = 0;
    private boolean one = false;
    private boolean two = false;
    private boolean revolverReady = true;
    private boolean ejecting = false;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private double feedforwardPower = 0;
    private static int pattern = 1;
    private double distance;
    private double power;
    private AnalogInput ejectAnalog;
    private boolean ejectreturn = false;
    private boolean launcherready = true;
    private double previousRevolverPosition;
    private PIDFController turretPIDF, ff, revolverPID;
    private SimpleServo set, turrot1, turrot2, eject, gate;
    private DistanceSensor intakedistone, intakedisttwo, launcherdist;
    private double RPM;
    private double lastTime;
    private int lastPosition;
    private double bearing = 0.0;
    double turretTarget = 180F; // inital turret angle red
    private boolean launching = false;
    private int revolverTarget = 0;
    private double revolverPower;
    private int ballcount = 0;
    private int ballsshot = 0;
    private boolean left;
    private Limelight3A limelight;



    ////////////////////////////////////////////////



    public void buildPaths() {
        shoot3 = follower.pathBuilder()
                .addPath(

                        new BezierLine(new Pose(18.790, 119.941), new Pose(44.078, 94.302))

                )
                .setLinearHeadingInterpolation(Math.toRadians(-126), Math.toRadians(-115))

                .build();



        eat3rotate = follower.pathBuilder()

                .addPath(

                        new BezierLine(new Pose(44.078, 94.302), new Pose(44.078, 84.941))

                )

                .setLinearHeadingInterpolation(Math.toRadians(-115), Math.toRadians(180))

                .build();



        eat3 = follower.pathBuilder()

                .addPath(

                        new BezierLine(new Pose(44.078, 84.941), new Pose(18.844, 84.941))

                )

                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .setTimeoutConstraint(500)

                .build();





        shoot6 = follower.pathBuilder()

                .addPath(

                        new BezierLine(new Pose(19.844, 84.941), new Pose(38.283, 94.127))

                )

                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-115))

                .build();



        eat6rotate = follower.pathBuilder()

                .addPath(

                        new BezierLine(new Pose(38.283, 94.127), new Pose(44.254, 60.3))

                )

                .setLinearHeadingInterpolation(Math.toRadians(-115), Math.toRadians(180))

                .build();



        eat6 = follower.pathBuilder()

                .addPath(

                        new BezierLine(new Pose(44.254, 60.3), new Pose(22.654, 60.3))

                )

                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .setTimeoutConstraint(500)

                .build();



        shoot9 = follower.pathBuilder()

                .addPath(

                        new BezierLine(new Pose(22.654, 60.3), new Pose(41.268, 95.532))

                )

                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-115))

                .build();

        escape = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(41.746, 92.873), new Pose(22.280, 54.645))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-115), Math.toRadians(-115))
                .build();

    }



    public void onerotation(boolean left) {
        revolverReady = false;

        if (left) {
            revolverindex += 1;
        } else {
            revolverindex -= 1;
        }

        revolverTarget = revolverindex * Globals.revolver.oneRotation;
    }



    public void getPattern() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(20);

        limelight.pipelineSwitch(0);

        limelight.start();



        long start = System.currentTimeMillis();

        while (System.currentTimeMillis() - start < 500) {  // 2s timeout

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                if (result.getStaleness() < 500) {

                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

                    if (!tags.isEmpty()) {

                        for (LLResultTypes.FiducialResult tag : tags) {

                            int id = tag.getFiducialId();

                            if (id == 21) pattern = 3;

                            if (id == 22) pattern = 2;

                            if (id == 23) pattern = 1;

                        }

                    }

                }

            }

        }

        if (pattern == 0) {

            pattern = 1;

        }

        limelight.stop();

    }



    public boolean ballcases(int pickupnum, boolean comingin) {

        if (pattern == 1){

            if (pickupnum == 1){

                if (comingin){

                    return true;

                } else if (!comingin){

                    return false;

                }

            } else if (pickupnum == 2) {

                if (comingin){

                    return false;

                } else if (!comingin){

                    return true;

                }

            } else if (pickupnum == 3) {

                if (comingin){

                    return true;

                } else if (!comingin){

                    return true;

                }

            } else if (pickupnum == 0) {

                if (!comingin){

                    return false;

                }

            }

        } else if (pattern == 2) {

            if (pickupnum == 1){

                if (comingin){

                    return true;

                } else if (!comingin){

                    return true;

                }

            } else if (pickupnum == 2) {

                if (comingin) {

                    return false;

                } else if (!comingin) {

                    return false;

                }

            } else if (pickupnum == 3) {

                if (comingin) {

                    return true;

                } else if (!comingin) {

                    return false;

                }

            }else if (pickupnum == 0) {

                if (!comingin){

                    return true;

                }

            }

        } else if (pattern == 3) {

            if (pickupnum == 1) {

                if (comingin) {

                    return true;

                } else if (!comingin) {

                    return true;

                }

            } else if (pickupnum == 2) {

                if (comingin) {

                    return true;

                } else if (!comingin) {

                    return true;

                }

            } else if (pickupnum == 3) {

                if (comingin) {

                    return false;

                } else if (!comingin) {

                    return true;

                }

            }else if (pickupnum == 0) {

                if (!comingin){

                    return false;

                }

            }

        }

        return true;

    }





    ///////////////////////////////////////////////



    public class intakesubsys extends SubsystemBase {
        private final Motor intake, revolver;

        public intakesubsys(HardwareMap map) {
            intake = new Motor(map, "intake");
            intake.setRunMode(Motor.RunMode.RawPower);
            intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            intake.set(0.0);

            revolver = new Motor(hardwareMap, "revolver", 28, 1150);
            revolver.setRunMode(Motor.RunMode.RawPower);
            revolver.resetEncoder();

            revolverPID = new PIDFController(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD, Globals.revolver.revolverKF);
            revolverPID.setTolerance(5);

            intakedistone = hardwareMap.get(DistanceSensor.class, "colour1");
            intakedisttwo = hardwareMap.get(DistanceSensor.class, "colour2");
            launcherdist = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        }

        public void intakeon() {
            intake.set(Globals.intakePower);
            ballcount = 0;
        }

        public void intakeoff() {
            intake.set(0);
            ballcount = 0;
        }


        @Override
        public void periodic() {
            revolverPower = revolverPID.calculate(revolver.getCurrentPosition(), revolverTarget);
            revolver.set(revolverPower);

            if (!revolverReady && Math.abs(revolver.getCurrentPosition() - revolverTarget) <= 3) {
                revolverReady = true;
            }
        }

        public void sort(int pickupnum){//1 = ppg 2 pgp 3 gpp,
            telemetry.addData("", ballcount);
            telemetry.addData("", revolverReady);
            telemetry.update();


            if (ballcount == 2 && revolverReady && pattern == 3 && pickupnum == 1) {
                onerotation(ballcases(pickupnum, true));
                ballcount += 1;
            }



            if (ballcount < 2) {
                if ((intakedistone.getDistance(DistanceUnit.CM) < 3 || intakedisttwo.getDistance(DistanceUnit.CM) < 3) && revolverReady) {
                    if (pattern == 1) {

                        if (pickupnum == 1) {
                            onerotation(ballcases(pickupnum, true));

                            ballcount += 1;

                        } else if (pickupnum == 2) {

                            onerotation(ballcases(pickupnum, true));

                            ballcount += 1;

                        } else if (pickupnum == 3) {
                            onerotation(ballcases(pickupnum, true));

                            ballcount += 1;

                        }

                    } else if (pattern == 2) {

                        if (pickupnum == 1) {

                            onerotation(ballcases(pickupnum, true));

                            ballcount += 1;

                        } else if (pickupnum == 2) {

                            onerotation(ballcases(pickupnum, true));

                            ballcount += 1;

                        } else if (pickupnum == 3) {

                            onerotation(ballcases(pickupnum, true));

                            ballcount += 1;

                        }

                    } else if (pattern == 3) {

                        if (pickupnum == 1) {

                            onerotation(ballcases(pickupnum, true));

                            ballcount += 1;

                        } else if (pickupnum == 2) {

                            onerotation(ballcases(pickupnum, true));

                            ballcount += 1;

                        } else if (pickupnum == 3) {

                            onerotation(ballcases(pickupnum, true));

                            ballcount += 1;

                        }

                    }

                }

            }
        }
    }

    public static class froggyeat extends CommandBase {
        private final intakesubsys intake;
        private int pickupnum;

        public froggyeat(intakesubsys intake, int pickupnum) {
            this.intake = intake;
            this.pickupnum = pickupnum;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            intake.intakeon();
        }

        @Override
        public void execute() {
            intake.sort(pickupnum);
        }

        @Override
        public void end(boolean interrupted) {
            intake.intakeoff();
        }
    }



    ///////////////////////////////////////////////



    public class outtakesubsys extends SubsystemBase {

        private Motor launcher1, launcher2, revolver;

        private final ElapsedTime timer = new ElapsedTime();

        private final ElapsedTime timerservo = new ElapsedTime();

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
            launcherdist = hardwareMap.get(DistanceSensor.class, "distanceSensor");

            lastPosition = launcher1.getCurrentPosition();

            revolver = new Motor(hardwareMap, "revolver", 28, 1150);

            revolver.setRunMode(Motor.RunMode.RawPower);

            revolver.resetEncoder();

            turretPIDF = new PIDFController(Globals.turret.turretKP, Globals.turret.turretKI, Globals.turret.turretKD, Globals.turret.turretKF);

            turretPIDF.setTolerance(Globals.turret.turretTol);

            eject = new SimpleServo(hardwareMap, "eject", 0, 70);

            eject.setInverted(true);

            eject.turnToAngle(Globals.pushServo.defualt);

            turrot1 = new SimpleServo(hardwareMap, "t1", 90, 270, AngleUnit.DEGREES);
            turrot1.turnToAngle(turretTarget);
            turrot2 = new SimpleServo(hardwareMap, "t2", 90, 270, AngleUnit.DEGREES);
            turrot2.turnToAngle(turretTarget);

            gate = new SimpleServo(hardwareMap,"gate", 0, 300, AngleUnit.DEGREES);
            gate.turnToAngle(Globals.openGate);

            ejectAnalog = hardwareMap.get(AnalogInput.class, "ejectAnalog");

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

        private void aiming() {
            ff.setPIDF(Globals.launcher.flykP, Globals.launcher.flykI, Globals.launcher.flykD, Globals.launcher.flykF);
            turretPIDF.setPIDF(Globals.turret.turretKP, Globals.turret.turretKI, Globals.turret.turretKD, Globals.turret.turretKF);
            visionPortal.setProcessorEnabled(tagProcessor, true);


            List<AprilTagDetection> detections = tagProcessor.getDetections();

                if (detections != null && !detections.isEmpty()) {
                    for (AprilTagDetection d : detections) {
                        if (d.ftcPose != null && d.id == 20) {//blue IS 20 red IS 24 TODO READD ID==20
                            power = (2547.5 * pow(2.718281828459045, (0.0078 * d.ftcPose.range))) / Globals.launcher.launcherTransformation; // here

                            aligned = Math.abs(d.ftcPose.bearing) <= Globals.turret.turretTol;
                            double delta = aligned ? 0.0 : turretPIDF.calculate(d.ftcPose.bearing, 0.0);
                            turretTarget += delta; //THIS IS POSITIVE
                        }
                    }
                } else {
                    power = 0;
                }

            turrot1.turnToAngle(turretTarget);
            turrot2.turnToAngle(turretTarget);
        }

        private void timerreset() {

            timer.reset();

            ballsshot = 0;

            ejecting = false;

            launching = false;

        }

        private void endmotor() {

            launcher1.set(0);

            launcher2.set(0);

            ballsshot = 0;

        }

        private void shooting(int shootnum){
            if (!launchfinished) {
                if (!ejecting) {
                    eject.turnToAngle(Globals.pushServo.eject);
                    ejecting = true;
                }

                if (aligned && Math.abs(power - RPM) < Globals.launcher.launcherTol && power > 0 && ejecting && launcherdist.getDistance(DistanceUnit.CM) < 6 && !cumdown) {
                    set.turnToAngle(Globals.launcher.upset);
                    ejectreturn = true;
                    cumdown = true;//dist is inconsistent
                }

                if (Math.abs(previousRPM - RPM) > Globals.launcher.RPMDipThreshold) {
                    set.turnToAngle(Globals.launcher.downset);
                }

                if (ejectreturn) {
                    eject.turnToAngle(Globals.pushServo.defualt);
                }

                if (ang < 165 && ejectreturn) {//fix
                    onerotation(ballcases(shootnum, false));
                    launchfinished = true;
                    ejectreturn = false;
                }
            }
        }



        private void launch(int shootnum) {
            telemetry.addData("time", timer.seconds());
            telemetry.addData("balls", ballsshot);
            telemetry.addData("revready", revolverReady);
            telemetry.update();
            ang = (ejectAnalog.getVoltage()/3.3) * 360;
            launcher1.set(feedforwardPower);
            launcher2.set(feedforwardPower);

            if (ballsshot < 3) {
                if (revolverReady) {
                    if (pattern == 1) {
                        if (shootnum == 0) {
                            shooting(shootnum);
                            if (launchfinished) {
                                ejecting = false;
                                launchfinished = false;
                                launching = false;
                                ejectreturn = true;
                                cumdown = false;
                                ballsshot += 1;
                            }
                        }


                    }
                }
            }
        }



        @Override

        public void periodic() {
            calculateRPM();
            aiming();

            feedforwardPower = ff.calculate(RPM, power);

        }

    }//todo

    public static class froggyspit extends CommandBase {

        private final outtakesubsys outtake;

        private int shootnum;

        public froggyspit(outtakesubsys outtake, int shootnum) {

            this.outtake = outtake;

            this.shootnum = shootnum;

            addRequirements(outtake);

        }



        @Override

        public void initialize() {

            outtake.timerreset();

        }



        @Override

        public void execute() {

            outtake.launch(shootnum);

        }



        @Override

        public void end(boolean interrupted) {

            outtake.endmotor();

        }

    }//todo















    @Override

    public void initialize() {

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



        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(18.790, 119.941, Math.toRadians(-126)));//todo

        telemetry.update();



        outtakesubsys loopedfunctionsout = new outtakesubsys(hardwareMap);

        register(loopedfunctionsout);



        outtakesubsys loopedfunctionsin = new outtakesubsys(hardwareMap);

        register(loopedfunctionsin);



        buildPaths();





        SequentialCommandGroup froggyroute = new SequentialCommandGroup(

//              new FollowPathCommand(follower, shoot3),
//
//                new ParallelDeadlineGroup(
//
//                        new WaitCommand(100),
//
//                        new InstantCommand(this::getPattern)
//
//                ),
//
                new ParallelDeadlineGroup(

                        new WaitCommand(20000),

                        new froggyspit(new outtakesubsys(hardwareMap), 0)

                ),
//
//                new FollowPathCommand(follower, eat3rotate),
//
                new ParallelDeadlineGroup(
                        new WaitCommand(10000),

                        //new FollowPathCommand(follower, eat3),

                        new froggyeat(new intakesubsys(hardwareMap), 1)

                )
//
//                new FollowPathCommand(follower, shoot6),
//
//                new ParallelDeadlineGroup(
//
//                        new WaitCommand(4300),
//
//                        new froggyspit(new outtakesubsys(hardwareMap), 1)
//
//                ),
//
//                new FollowPathCommand(follower, eat6rotate),
//
//                new ParallelDeadlineGroup(
//
//                        new FollowPathCommand(follower, eat6),
//
//                        new froggyeat(new intakesubsys(hardwareMap), 2)
//
//                ),
//
//                new FollowPathCommand(follower, shoot9),
//
//                new ParallelDeadlineGroup(
//
//                        new WaitCommand(4300),
//
//                        new froggyspit(new outtakesubsys(hardwareMap), 2)
//
//                ),
//                new FollowPathCommand(follower, escape)

        );

        schedule(froggyroute);

    }



    @Override

    public void run() {

        super.run();

        follower.update();



        telemetryData.addData("X", follower.getPose().getX());

        telemetryData.addData("Y", follower.getPose().getY());

        telemetryData.addData("Heading", follower.getPose().getHeading());

        telemetryData.addData("pattern", pattern);

        telemetryData.update();

    }

}

