package org.firstinspires.ftc.teamcode.chuckFull.subsystems;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.Globals;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
/**
 * Launcher: contains two flywheels and the gate servo. Exposes periodic() which maintains feedforward.
 */
public class LauncherSubsystem extends SubsystemBase {
    private final Motor l1, l2;
    private final SimpleServo gate;
    private final PIDFController ff;


    // runtime state
    private double RPM = 0;
    private double lastTime = 0;
    private int lastPosition = 0;


    public LauncherSubsystem(HardwareMap hw) {
        l1 = new Motor(hw, "l1", 28, 6000);
        l2 = new Motor(hw, "l2", 28, 6000);
        l1.setRunMode(Motor.RunMode.RawPower);
        l2.setRunMode(Motor.RunMode.RawPower);
        l1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        l2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);


        gate = new SimpleServo(hw, "set", 0, 180);
        gate.turnToAngle(Globals.launcher.downset);


        ff = new PIDFController(Globals.launcher.flykP, Globals.launcher.flykI, Globals.launcher.flykD, Globals.launcher.flykF);


        lastTime = System.nanoTime() / 1e9;
        lastPosition = l1.getCurrentPosition();
    }


    public void periodic(GamepadEx g, TurretSubsystem turret, PatternSubsystem pattern) {
// update RPM
        double currentTime = System.nanoTime() / 1e9;
        int currentPosition = l1.getCurrentPosition();
        double deltaTime = currentTime - lastTime;
        int deltaTicks = currentPosition - lastPosition;
        if (deltaTime > 0.05) {
            double revs = (double) deltaTicks / 28.0;
            RPM = (revs / deltaTime) * 60.0;
            lastTime = currentTime;
            lastPosition = currentPosition;
        }


// compute power target from vision (distance)
        double powerTarget = 0;
        if (turret != null && turret.getLastDetectedDistance() > 0) {
            double d = turret.getLastDetectedDistance();
            powerTarget = (2547.5 * Math.exp(0.0078 * d)) / Globals.launcher.launcherTransformation;
        }


        ff.setP(Globals.launcher.flykP);
        ff.setI(Globals.launcher.flykI);
        ff.setD(Globals.launcher.flykD);
        ff.setF(Globals.launcher.flykF);


        double feed = ff.calculate(RPM, powerTarget);
        l1.set(feed);
        l2.set(feed);
    }


    public void setGateUp() { gate.turnToAngle(Globals.launcher.upset); }
    public void setGateDown() { gate.turnToAngle(Globals.launcher.downset); }
    public void stop() { l1.set(0); l2.set(0); }
}