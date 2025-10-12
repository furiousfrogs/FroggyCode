package org.firstinspires.ftc.teamcode.commandbase;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.hardware.Globals;

@TeleOp(name = "xx", group = "Testing")
public class xx extends OpMode {

    private MotorEx revolver;
    private GamepadEx gamepadEx;

    // GoBILDA 5203 Yellow Jacket constants
    private static final double TICKS_PER_REV = 537.7; // encoder ticks per full revolution
    private static final double TARGET_DEGREES = 270.0;
    private static final double KP_DEFAULT = 0.01; // fallback if Globals not initialized
    private static final double POWER = 0.5;

    @Override
    public void init() {
        // Motor setup (ticks per rev, RPM)
        revolver = new MotorEx(hardwareMap, "revolver", 1538, 435);
        revolver.setRunMode(MotorEx.RunMode.PositionControl);
        revolver.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        // Apply your PID constants from Globals, with safety fallback
        revolver.setPositionCoefficient(Globals.revolverKP != 0 ? Globals.revolverKP : KP_DEFAULT);
        revolver.setPositionTolerance(Globals.revolverTol);

        gamepadEx = new GamepadEx(gamepad1);

        telemetry.addLine("RotateTest Initialized");
        telemetry.addData("Target Δ", TARGET_DEGREES + "°");
        telemetry.update();
    }

    @Override
    public void loop() {
        gamepadEx.readButtons(); // ensure GamepadEx updates state

        // Refresh tuning in real time
        revolver.setPositionCoefficient(Globals.revolverKP != 0 ? Globals.revolverKP : KP_DEFAULT);
        revolver.setPositionTolerance(Globals.revolverTol);

        double ticksPerDegree = TICKS_PER_REV / 360.0;
        double deltaTicks = TARGET_DEGREES * ticksPerDegree;

        if (gamepadEx.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            revolver.setTargetPosition((int)(revolver.getCurrentPosition() + deltaTicks));
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.CROSS)) {
            revolver.setTargetPosition((int)(revolver.getCurrentPosition() - deltaTicks));
        }

        // Drive motor toward target
        if (!revolver.atTargetPosition()) {
            revolver.set(POWER);
        } else {
            revolver.stopMotor();
        }

        telemetry.addData("Current Pos (ticks)", revolver.getCurrentPosition());
        //telemetry.addData("Target Pos (ticks)", revolver.getTargetPosition());
        telemetry.addData("At Target?", revolver.atTargetPosition());
        telemetry.addData("Power", POWER);
        telemetry.update();
    }
}
