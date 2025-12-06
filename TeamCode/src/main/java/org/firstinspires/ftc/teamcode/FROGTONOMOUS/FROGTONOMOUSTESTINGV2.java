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

@Autonomous
@Configurable
public class FROGTONOMOUSTESTINGV2 extends CommandOpMode {
    private Follower follower;
    TelemetryData telemetryData = new TelemetryData(telemetry);
    private PathChain shoot3, eat3, shoot6, eat6setup, eat6, shoot9, eat9setup, eat9, shoot12;
    private boolean aligned = false;
    private boolean launchfinished = false;
    private boolean cumdown = false;
    private double revolverposition;
    private boolean ejected = false;
    private double ang;
    private double previousRPM = 0;
    private boolean rotated = false;
    private boolean camedown = false;
    private boolean revolverReady = true;
    private boolean ejecting = false;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    private double feedforwardPower = 0;
    private static int pattern = 1;
    private double distance;
    private int revolverindex = 0;
    private int revolvertarget = 0;
    private static final int revolvertol = 5;
    private double power;
    private AnalogInput ejectAnalog;
    private double previousRevolverPosition;
    private PIDFController turretPIDF, ff, revolverPID;
    private SimpleServo set, turrot1, turrot2, eject, gate;
    private DistanceSensor intakedistone, intakedisttwo, launcherdist;
    private double RPM;
    private double lastTime;
    private int lastPosition;
    private double bearing = 0.0;
    double turretTarget = 222F; // inital turret angle red
    private boolean launching = true;
    private double revolverPower;
    private int ballcount = 0;
    private int ballsshot = 0;
    private boolean pattern3turned = false;
    private intakesubsys froggyintake;
    private outtakesubsys froggyouttake;
    private visionsubsystem froggyvision;

    private enum launchseq {
        NOTREADY,
        READY,
        SHOOTING,
        RESET,
        COOLDOWN,
        FINISHED
    } private launchseq froggylaunch = launchseq.READY;



    ////////////////////////////////////////////////


    public void buildPaths() {
        shoot3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.738, 122.019), new Pose(45.308, 86.131))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-126), Math.toRadians(-180))
                .build();

        eat3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45.308, 86.131), new Pose(20.020, 85.698))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shoot6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.020, 85.698), new Pose(45.308, 86.131))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        eat6setup = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45.308, 86.131), new Pose(43.795, 61.639))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        eat6 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(43.795, 61.639), new Pose(19.317, 61.463))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shoot9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(19.317, 61.463), new Pose(45.308, 86.131))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        eat9setup = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(45.308, 86.131), new Pose(43.944, 37.234))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        eat9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(43.944, 37.234), new Pose(18.841, 37.234))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        shoot12 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(18.841, 37.234), new Pose(45.308, 86.131))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
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
        public intakesubsys(HardwareMap hardwareMap) {
            intake = new Motor(hardwareMap, "intake");
            intake.setRunMode(Motor.RunMode.RawPower);
            intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            intake.set(0.0);

            revolver = new Motor(hardwareMap, "revolver", 28, 1150);
            revolver.setRunMode(Motor.RunMode.RawPower);
            revolver.resetEncoder();

            revolverPID = new PIDFController(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD, Globals.revolver.revolverKF);
            revolverPID.setTolerance(revolvertol);

            intakedistone = hardwareMap.get(DistanceSensor.class, "colour1");
            intakedisttwo = hardwareMap.get(DistanceSensor.class, "colour2");

            gate = new SimpleServo(hardwareMap,"gate", 0, 300, AngleUnit.DEGREES);
            gate.turnToAngle(Globals.openGate);

            revolverindex = 0;
            revolvertarget = 0;

            revolverReady = true;
        }

        public void intakeon() {
            intake.set(Globals.intakePower);
            gate.turnToAngle(Globals.closeGate);
            ballcount = 0;
        }

        public void intake() {
            intake.set(Globals.intakePower);
        }

        public void intakeoff() {
            intake.set(0);
            gate.turnToAngle(Globals.openGate);
            ballcount = 0;
        }

        public void outtake() {
            intake.set(0);
        }

        public void sort(int pickupnum){//1 = ppg 2 pgp 3 gpp,
            if (ballcount == 2 && pattern == 3 && pickupnum == 1 && revolverReady) {
                onerotation(ballcases(pickupnum, true));
                ballcount ++;
            }

            if (ballcount < 2) {
                if ((intakedistone.getDistance(DistanceUnit.CM) < 2 || intakedisttwo.getDistance(DistanceUnit.CM) < 2) && revolverReady) {
                    onerotation(ballcases(pickupnum, true));
                    ballcount ++;
                }
            }
        }

        public void onerotation(boolean left) {
            revolverReady = false;
            revolverindex += left ? 1 : -1;
            revolvertarget = revolverindex * Globals.revolver.oneRotation;
        }

        @Override
        public void periodic () {
            revolverPower = revolverPID.calculate(revolver.getCurrentPosition(), revolvertarget);
            revolver.set(revolverPower);

            if (!revolverReady &&
                    Math.abs(revolver.getCurrentPosition() - revolvertarget) < revolvertol) {
                revolverReady = true;
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

    public static class froggyintakeon extends CommandBase {
        private final intakesubsys intake;

        public froggyintakeon(intakesubsys intake) {
            this.intake = intake;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            intake.intake();
        }

        @Override
        public void end(boolean interrupted) {
            intake.outtake();
        }
    }



    ///////////////////////////////////////////////



    public class outtakesubsys extends SubsystemBase {
        private Motor launcher1, launcher2;
        private final intakesubsys intake;
        private final ElapsedTime timer = new ElapsedTime();
        public outtakesubsys(HardwareMap hardwareMap, intakesubsys intake) {
            this.intake = intake;

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
            turretPIDF = new PIDFController(Globals.turret.turretKP, Globals.turret.turretKI, Globals.turret.turretKD, Globals.turret.turretKF);
            turretPIDF.setTolerance(Globals.turret.turretTol);
            eject = new SimpleServo(hardwareMap, "eject", 0, 70);
            eject.setInverted(true);
            eject.turnToAngle(Globals.pushServo.defualt);
            turrot1 = new SimpleServo(hardwareMap, "t1", 90, 270, AngleUnit.DEGREES);
            turrot1.turnToAngle(turretTarget);
            turrot2 = new SimpleServo(hardwareMap, "t2", 90, 270, AngleUnit.DEGREES);
            turrot2.turnToAngle(turretTarget);

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

        private void outtakeon() {
            timer.reset();
            ballsshot = 0;

            froggylaunch = launchseq.NOTREADY;
        }

        private void outtakeoff() {
            launcher1.set(0);
            launcher2.set(0);
            ballsshot = 0;
        }

        private void shooting(int shootnum){
            switch (froggylaunch) {
                case NOTREADY:
                    if (revolverReady){
                        rotated = false;
                        camedown = false;
                        froggylaunch = launchseq.READY;
                        break;
                    }
                    break;
                case READY:
                    if (!ejected){
                        timer.reset();
                        ejected = true;
                    }
                    eject.turnToAngle(Globals.pushServo.eject);
                    if (timer.seconds() > 0.1) {
                        if ((ang > 175 && ang < 195) || launcherdist.getDistance(DistanceUnit.CM) < 5) {
                            froggylaunch = launchseq.SHOOTING;
                            break;
                        } else if (timer.seconds() > 0.3){
                            froggylaunch = launchseq.SHOOTING;
                            break;
                        }
                    }
                    break;
                case SHOOTING:
                    eject.turnToAngle(Globals.pushServo.defualt);
                    set.turnToAngle(Globals.launcher.upset);
                    froggylaunch = launchseq.RESET;
                    break;
                case RESET:
                    if (!camedown && (previousRPM - RPM) > Globals.launcher.RPMDipThreshold){
                        set.turnToAngle(Globals.launcher.downset);
                        timer.reset();
                        camedown = true;
                    }
                    if (!rotated && ang < 165){
                        intake.onerotation(ballcases(shootnum, false));
                        rotated = true;
                    }
                    if (camedown && rotated) {
                        froggylaunch = launchseq.COOLDOWN;
                        break;
                    }
                    break;
                case COOLDOWN:
                    if (timer.seconds() > 0.5){//can be optimized by moving timer.reset to rpm dip part
                        ballsshot++;
                        if (ballsshot < 3){
                            froggylaunch = launchseq.NOTREADY;
                            break;
                        } else {
                            froggylaunch = launchseq.FINISHED;
                            break;
                        }
                    }
                    break;
                case FINISHED:
                    break;
            }
        }

        private void launch(int shootnum) {
            calculateRPM();
            //feedforwardPower = ff.calculate(RPM, power);
            feedforwardPower = ff.calculate(RPM, 3500);
            ang = (ejectAnalog.getVoltage()/3.3) * 360;
            launcher1.set(feedforwardPower);
            launcher2.set(feedforwardPower);
            if (pattern == 3 && shootnum == 0 && !pattern3turned){
                intake.onerotation(true);
                pattern3turned = true;
            }
            shooting(shootnum);
        }


        @Override
        public void periodic() {
            //aiming();
        }
    }

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
            outtake.outtakeon();
        }

        @Override
        public void execute() {
            outtake.launch(shootnum);
        }

        @Override
        public void end(boolean interrupted) {
            outtake.outtakeoff();
        }
    }


    //////////////////////////////////////


    public class visionsubsystem extends SubsystemBase {
        private Limelight3A limelight;
        public visionsubsystem(HardwareMap map) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(20);
            limelight.pipelineSwitch(0);
            limelight.start();
        }

        public void getpattern(){
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

        public void visionend(){
            limelight.stop();
        }

    }

    public static class froggyvision extends CommandBase{
        private final visionsubsystem visionsubsystem;

        public froggyvision(visionsubsystem visionsubsystem){
            this.visionsubsystem = visionsubsystem;
            addRequirements(visionsubsystem);
        }

        @Override
        public void execute() {
            visionsubsystem.getpattern();
        }

        @Override
        public void end(boolean interrupted) {
            visionsubsystem.visionend();
        }
    }














    @Override
    public void initialize() {
        froggyintake = new intakesubsys(hardwareMap);
        froggyouttake = new outtakesubsys(hardwareMap, froggyintake);
        froggyvision = new visionsubsystem(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(19.738, 122.019, Math.toRadians(-126)));//todo
        telemetry.update();

        register(froggyintake);
        register(froggyouttake);

        buildPaths();





        SequentialCommandGroup froggyroute = new SequentialCommandGroup(
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, shoot3),
                        new froggyvision(froggyvision)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(3300),
                        new froggyspit(froggyouttake, 0)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, eat3),
                        new froggyeat(froggyintake, 1)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, shoot6),
                        new froggyintakeon(froggyintake)
                ),

                new ParallelDeadlineGroup(
                        new WaitCommand(3300),
                        new froggyspit(froggyouttake, 1)
                ),
                new FollowPathCommand(follower, eat6setup),
//
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, eat6),
                        new froggyeat(froggyintake, 2)
                ),
//
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, shoot9),
                        new froggyintakeon(froggyintake)
                ),

                new ParallelDeadlineGroup(
                        new WaitCommand(3300),
                        new froggyspit(froggyouttake, 2)
                ),
                new FollowPathCommand(follower, eat9setup),//TODO ADD COMMA FOR 12BALL

                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, eat9),
                        new froggyeat(froggyintake, 1)
                ),
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, shoot12),
                        new froggyintakeon(froggyintake)
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(2500),
                        new froggyspit(froggyouttake, 1)
                )

        );

        schedule(froggyroute);

    }



    @Override

    public void run() {

        super.run();

        follower.update();



//        telemetryData.addData("X", follower.getPose().getX());
//        telemetryData.addData("Y", follower.getPose().getY());
//        telemetryData.addData("Heading", follower.getPose().getHeading());
        telemetryData.addData("pattern", pattern);
        telemetryData.addData("state", froggylaunch);
        telemetryData.addData("ballsshot", ballsshot);
        telemetryData.addData("revolstate", revolverReady);
        telemetryData.update();

    }

}



