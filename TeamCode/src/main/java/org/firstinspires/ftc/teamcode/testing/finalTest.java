package org.firstinspires.ftc.teamcode.testing;

import static java.lang.Math.pow;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.SimpleServoExtKt;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.hardware.Globals;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import java.util.Collections;



@TeleOp(name = "Final")
public class finalTest extends OpMode {

    // ----- booleans/toggles
    private boolean aligned = false;
    private boolean patternDetected = false;
    private boolean autoAimEnabled = true;
    private boolean prevTri = false;

    // ----- doubles -----
    private double distance;
    private double power;

    // ----- pid's -----
    private PIDFController turretPIDF, ff;
    private PIDController revolverPID;

    // ----- Motors, servos, sensors -----
    private Motor launcher1, launcher2, revolver,fl,bl,fr,br, intake;
    private SimpleServo set, rotate, eject;
    private NormalizedColorSensor colourSensor;
    private DistanceSensor distanceSensor;

    // ----- launcher -----
    private double RPM;
    private double lastTime;
    private int lastPosition;

    // ----- turret -----
    private double bearing = 0.0;
    double turretTarget = 150F; // inital turret angle

    // ----- revolver -----
    private final float[] hsv = new float[3];


    //index 0 is the top, 1 is the left, 2 is the right when looking from the front view
    private List<String> revolverState = new ArrayList<>(Arrays.asList("EMPTY", "EMPTY", "EMPTY"));
    private int revolverTarget = 0;
    private double revolverPower;
    public enum pattern {
        PPG,
        PGP,
        GPP
    } pattern currentPattern = pattern.PPG;

    // ----- misc -----
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private GamepadEx gamepadEx;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    @Override
    public void init() {
        // ----- drive -----
        fl = new Motor(hardwareMap,"fl");
        fl.setRunMode(Motor.RunMode.RawPower);
        fl.setInverted(true);
        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        bl = new Motor(hardwareMap,"bl");
        bl.setRunMode(Motor.RunMode.RawPower);
        bl.setInverted(true);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        fr = new Motor(hardwareMap,"fr");
        fr.setRunMode(Motor.RunMode.RawPower);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        br = new Motor(hardwareMap,"br");
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
        set = new SimpleServo(hardwareMap, "set", 0, 180, AngleUnit.DEGREES);
        // ----- launcher helpers -----
        lastTime = getRuntime();
        lastPosition = launcher1.getCurrentPosition();

        // ----- revolver -----
        revolver = new Motor(hardwareMap, "revolver", 28, 1150);
        revolver.setRunMode(Motor.RunMode.RawPower);
        revolver.resetEncoder();
        revolverPID = new PIDController(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD);
        // ----- colour sensor -----
        colourSensor = hardwareMap.get(NormalizedColorSensor.class,"colour1");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colour1");
        colourSensor.setGain(2.0f); //CAMERA SENSITIVITY, increase for darker environemnts

        // ----- turret -----
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

        gamepadEx = new GamepadEx(gamepad1);

        intake = new Motor(hardwareMap, "intake");
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        eject = new SimpleServo(hardwareMap, "eject", 0, 70);
        eject.setInverted(true);
        eject.turnToAngle(44);

    }

    @Override
    public void loop() { // TODO add revolver sequence logic
        calculateRPM();
        launcherawe();
        autoAimServoMode();      // now only reads/controls; does NOT rebuild vision
        doTelemetry();
        intake();
        drive();
        ejection();
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

    private void froggystomach(int pattern1234) {
        // 0 is intake bottom, 1 is top, 2 is non intake side.
        // pattern 0 = ppg, 1 = pgp, 2 = gpp
        ArrayList<String> balls = new ArrayList<String>(3);
        balls.add("empty");
        balls.add("empty");
        balls.add("empty");

        // COLLECTIONS.ROTATE EXAMPLE LIST1 = [1,2,3]
        // Collections.rotate(list1, 1) => [2,3,1]

        //intake function needed
        balls.set(0, senseColour());
        Collections.rotate(balls, 1);//rotates so intake side is now at top
        balls.set(0, senseColour());;
        Collections.rotate(balls, 1);//rotates so intake side is now at top
        balls.set(0, senseColour());
        Collections.rotate(balls, 1);

        //outttake
        if (pattern1234 == 0){
            if (balls.get(1) == "p"){
                if (balls.get(0) == "g"){
                    //shoot
                    //rotate clockwise
                    //shoot
                    //rotate clockwise
                    //shoot
                } else {
                    //shoot
                    //rotate clockwise
                    //shoot
                    //rotate clockwise
                    //shoot
                }
            } else {
                //rotateclockwise
                //shoot
                //rotate clockwise
                //shoot
                //rotate clockwise
                //shoot
            }
        }
    }


    private String senseColour() {
        if (distanceSensor.getDistance(DistanceUnit.CM) > 2.5) return "EMPTY";

        NormalizedRGBA rgba = colourSensor.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), hsv); // hsv[0]=H, hsv[1]=S, hsv[2]=V

        if (hsv[0] >= 150 && hsv[0] <= 180 &&
                hsv[1] >= 0.75 && hsv[1] <= 1.00 &&
                hsv[2] > 0.00 && hsv[2] < 0.16) {
            return "G";
        }

        if (hsv[0] >= 220 && hsv[0] <= 250 &&
                hsv[1] >= 0.40 && hsv[1] <= 0.60 &&
                hsv[2] > 0.00 && hsv[2] < 0.16) {
            return "P";
        }

        return "EMPTY";
    }


    public void ejection() {
        if (gamepad1.right_stick_x > 0.5) {
            eject.turnToAngle(28);
        } else if (gamepad1.right_stick_x < -0.5) {
            eject.turnToAngle(51);
        } else {
            eject.turnToAngle(44);

        }// eject is 30, default is 44, push is 51

    }

    public void intake() {
        if (gamepadEx.getButton(GamepadKeys.Button.TRIANGLE)) {
            int filled = revolverState.size() - Collections.frequency(revolverState, "EMPTY");
            String color = senseColour(); // "G", "P", or "EMPTY"
            intake.set(Globals.intakePower);

            if (!"EMPTY".equals(color)) {
                switch (filled) {
                    case 3:
                        intake.set(0);
                        break;

                    case 2:
                        revolverState.set(2, color);
                        intake.set(0);
                        break;

                    case 1:
                        revolverState.set(2, color);
                        if (revolverState.get(1).equals("EMPTY")) {
                            rotate(false);
                        } else {
                            rotate(true);
                        }
                        intake.set(0);
                        break;

                    case 0:
                    default:
                        revolverState.set(2,color);
                        String[] desired = desiredByPattern();
                        String wantTop  = desired[0];
                        String wantLeft = desired[1];
                        String wantRight= desired[2];

                        if (color.equals(wantTop)) {
                            // Move right -> top
                            rotate(false); // right rotation
                        } else if (color.equals(wantLeft)) {
                            // Move right -> left
                            rotate(true);  // left rotation
                        } else {
                            // color already belongs on right; no rotation
                        }

                        intake.set(0);
                        break;
                }
            }
        } else {
            intake.set(0);
        }
    }





    public void rotate(boolean left) {
        // PID setup
        revolverPID.setTolerance(0);
        revolverPID.setPID(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD);

        if (eject.getAngle() != 51) {
            String t = revolverState.get(0);
            String l = revolverState.get(1);
            String r = revolverState.get(2);

            if (left) {
                revolverTarget += Globals.revolver.oneRotation;
                //t,l,r -> l,r,t
                revolverState.set(0, l);
                revolverState.set(1, r);
                revolverState.set(2, t);
            } else {
                revolverTarget -= Globals.revolver.oneRotation;
                //t,l,r -> r,t,l
                revolverState.set(0, r);
                revolverState.set(1, t);
                revolverState.set(2, l);
            }
        }

        revolverPower = revolverPID.calculate(revolver.getCurrentPosition(), revolverTarget);
        revolver.set(revolverPower);
    }


    private void findPattern() {
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.OPTIONS)) {
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
        }
    }


    private void calculateRPM() {
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

    private void autoAimServoMode() {
        turretPIDF.setPIDF(Globals.turret.turretKP, Globals.turret.turretKI, Globals.turret.turretKD, Globals.turret.turretKF);

//        boolean tri = gamepadEx.getButton(GamepadKeys.Button.TRIANGLE);
//        if (tri && !prevTri) {
//            autoAimEnabled = !autoAimEnabled;
//            turretPIDF.reset();
//        }
//        prevTri = tri;

        boolean lb = gamepadEx.getButton(GamepadKeys.Button.LEFT_BUMPER);
        boolean rb = gamepadEx.getButton(GamepadKeys.Button.RIGHT_BUMPER);


        List<AprilTagDetection> detections = tagProcessor.getDetections();
        if (autoAimEnabled && !lb ^ rb) {
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

        } else if (!autoAimEnabled && !patternDetected) {
            visionPortal.setProcessorEnabled(tagProcessor, false);
        }

        if (lb ^ rb) {
            aligned = false;
            turretTarget += lb ? +Globals.turret.nudge : -Globals.turret.nudge;

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
                distance = d.ftcPose.range;

                power = (2547.5 * pow(2.718281828459045, (0.0078 * distance)))/Globals.launcher.launcherTransformation; // here

            }
        } else {
            power = 0;

        }

        double feedforwardPower = ff.calculate(RPM, power);


        if (gamepadEx.getButton(GamepadKeys.Button.CROSS)) {
            launcher1.set(feedforwardPower);
            launcher2.set(feedforwardPower);
            if (Math.abs(power - RPM) < Globals.launcher.launcherTol && aligned) { // there
                set.turnToAngle(Globals.launcher.upset);
            }
        } else {
            launcher1.set(0);
            launcher2.set(0);
            set.turnToAngle(Globals.launcher.downset);
        }

    }

    public void launch3() {


    }


    private void doTelemetry() {
        telemetry.addData("aligned? ", aligned);
        telemetry.addData("pattern?: ", currentPattern);
        telemetry.update();

        TelemetryPacket rpmPacket = new TelemetryPacket();
        rpmPacket.put("RPM", RPM);

        TelemetryPacket powerPacket = new TelemetryPacket();
        powerPacket.put("targetRPM", power);

        FtcDashboard.getInstance().sendTelemetryPacket(powerPacket);
        FtcDashboard.getInstance().sendTelemetryPacket(rpmPacket);
    }


    private void drive(){
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_trigger - gamepad1.left_trigger;
        double slowdown = 0.6;
        if (Math.abs(y)<0.2){
            y=0.0;
        }
        if (Math.abs(x)<0.2){
            x=0.0;
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
