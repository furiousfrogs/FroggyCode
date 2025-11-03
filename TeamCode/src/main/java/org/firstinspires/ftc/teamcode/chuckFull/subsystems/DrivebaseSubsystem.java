package org.firstinspires.ftc.teamcode.chuckFull.subsystems;


import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DrivebaseSubsystem extends SubsystemBase {
    private final Motor fl, fr, bl, br;


    public DrivebaseSubsystem(HardwareMap hw) {
        fl = new Motor(hw, "fl");
        fl.setRunMode(Motor.RunMode.RawPower);
        fl.setInverted(true);
        fl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        bl = new Motor(hw, "bl");
        bl.setRunMode(Motor.RunMode.RawPower);
        bl.setInverted(true);
        bl.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        fr = new Motor(hw, "fr");
        fr.setRunMode(Motor.RunMode.RawPower);
        fr.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        br = new Motor(hw, "br");
        br.setRunMode(Motor.RunMode.RawPower);
        br.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }


    public void periodic() {
// no-op default; driven directly from GamepadEx by RobotContainer in this design
    }


    public void drive(GamepadEx g) {
        double y = -g.getLeftY();
        double x = g.getLeftX() * 1.1;
        double rx= g.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)-g.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double slowdown = 0.6;


        if (Math.abs(y) < 0.2) y = 0.0;
        if (Math.abs(x) < 0.2) x = 0.0;


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