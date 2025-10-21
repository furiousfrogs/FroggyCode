package org.firstinspires.ftc.teamcode.testing;

import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.controller.PIDController;
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

    private PIDController revolverPID;

    private double revolverPower;
    // tuning
    private static final double DRIVE_POWER = 0.1; // while PositionControl does its job

    @Override
    public void init() {
        revolver = new Motor(hardwareMap, "revolver", 384.5, 435);
        revolver.setRunMode(Motor.RunMode.RawPower);


        revolver.resetEncoder();                     // zero now (or after homing)


        gamepadEx = new GamepadEx(gamepad1);

        revolverPID = new PIDController(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD);
    }

    @Override
    public void loop() {
        rotate();
        // (optional) telemetry
        telemetry.addData("currently: ", revolver.getCurrentPosition());
        telemetry.addData("target: ", revolverTarget);
        telemetry.update();
    }
    public void rotate() {
        revolverPID.setTolerance(0);
        gamepadEx.readButtons();
        revolverPID.setPID(Globals.revolver.revolverKP, Globals.revolver.revolverKI, Globals.revolver.revolverKD);

        if (gamepadEx.wasJustPressed(GamepadKeys.Button.SQUARE)) {
            revolverTarget += Globals.revolver.oneRotation;  // CW
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.CROSS)) {
            revolverTarget -= Globals.revolver.oneRotation;
        }

        revolverPower = revolverPID.calculate(revolver.getCurrentPosition(), revolverTarget);
        revolver.set(revolverPower);

    }
}
