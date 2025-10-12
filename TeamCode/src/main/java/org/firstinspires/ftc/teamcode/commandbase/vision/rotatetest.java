package org.firstinspires.ftc.teamcode.commandbase.vision;

import android.provider.Settings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.hardware.Globals;

@TeleOp(name = "rotate test")
public class rotatetest extends OpMode {
 private Motor revolver;
    private GamepadEx gamepadEx;
    @Override
    public void init() {


        //revolver = hardwareMap.get(DcMotor.class, "revolver");
        //revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //revolver.setTargetPosition(0);
        //revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        revolver = new Motor(hardwareMap, "revolver", 1538.0, 435);
        revolver.setRunMode(Motor.RunMode.PositionControl);
        revolver.setPositionCoefficient(Globals.revolverKP);
        gamepadEx = new GamepadEx(gamepad1);
    }

    @Override
    public void loop(){
        revolver.setPositionCoefficient(Globals.revolverKP);
        revolver.setPositionTolerance(Globals.revolverTol);
        if (gamepadEx.getButton(GamepadKeys.Button.SQUARE)) {
            revolver.setTargetPosition(revolver.getCurrentPosition() + Globals.oneRotation); // 2 rotations
        } else if (gamepadEx.getButton(GamepadKeys.Button.CROSS)) {
            revolver.setTargetPosition(revolver.getCurrentPosition() - Globals.oneRotation);
        }
        if (!revolver.atTargetPosition()) {
            revolver.set(0.5);
        } else {
            revolver.set(0);
        }
    }
}