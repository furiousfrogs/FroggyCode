package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.hardware.Globals;
@TeleOp(name = "rotate test")
public class rotatetest extends OpMode {
    private Motor revolver;
    private GamepadEx gamepadEx;

    private boolean indexInProgress = false;
    private int revolverTarget = 0;

    // tuning
    private static final double DRIVE_POWER = 0.1; // while PositionControl does its job

    @Override
    public void init() {
        revolver = new Motor(hardwareMap, "revolver", Globals.cpr, Globals.rpm);
        revolver.setRunMode(Motor.RunMode.PositionControl);
        revolver.setPositionCoefficient(Globals.revolverKP);
        revolver.setPositionTolerance(Globals.revolverTol);

        revolver.resetEncoder();                     // zero now (or after homing)
        revolverTarget = revolver.getCurrentPosition(); // anchor target to current
        revolver.setTargetPosition(revolverTarget);
        revolver.stopMotor();

        gamepadEx = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        gamepadEx.readButtons();

        // keep tunables live if you use Dashboard
        revolver.setPositionCoefficient(Globals.revolverKP);
        revolver.setPositionTolerance(Globals.revolverTol);

        // Service an in-flight index first
        if (indexInProgress) {
            if (revolver.atTargetPosition()) {
                revolver.stopMotor();
                indexInProgress = false;
            } else {
                revolver.set(DRIVE_POWER);
            }
            return;
        }

        // No move in progress: check for a new step
        int step = 0;  // <-- reset every loop
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            step =  (int)Math.round(Globals.oneRotation);   // CW
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.CROSS)) {
            step = -(int)Math.round(Globals.oneRotation);   // CCW
        }

        if (step != 0) {
            revolverTarget += step;                 // accumulate EXACT target
            revolver.setTargetPosition(revolverTarget);
            revolver.set(DRIVE_POWER);
            indexInProgress = true;
        }

        // (optional) telemetry
        telemetry.addData("target", revolverTarget);
        telemetry.addData("pos", revolver.getCurrentPosition());
        telemetry.addData("busy", indexInProgress && !revolver.atTargetPosition());
        telemetry.update();
    }
}
