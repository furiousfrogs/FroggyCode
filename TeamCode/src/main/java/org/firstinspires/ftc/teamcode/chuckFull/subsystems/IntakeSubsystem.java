package org.firstinspires.ftc.teamcode.chuckFull.subsystems;


import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.Globals;


public class IntakeSubsystem extends SubsystemBase {
    private final Motor intake;
    private final SimpleServo eject;


    public IntakeSubsystem(HardwareMap hw) {
        intake = new Motor(hw, "intake");
        intake.setRunMode(Motor.RunMode.RawPower);
        intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        eject = new SimpleServo(hw, "eject", 0, 70);
        eject.setInverted(true);
        eject.turnToAngle(Globals.pushServo.defualt);
    }


    public void periodic(GamepadEx g) {
        intake.set(g.getButton(com.seattlesolvers.solverslib.gamepad.GamepadKeys.Button.TRIANGLE) ? Globals.intakePower : 0);


        boolean rightXPos = g.getRightX() > 0.5;
        boolean rightXNeg = g.getRightX() < -0.5;
        if (rightXPos) eject.turnToAngle(28);
        else if (rightXNeg) eject.turnToAngle(51);
        else eject.turnToAngle(44);
    }


    public void stop() { intake.set(0); }
}