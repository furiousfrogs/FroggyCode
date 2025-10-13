package org.firstinspires.ftc.teamcode.commandbase.vision;

import android.provider.Settings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.gamepad.ButtonReader; //SIX SEVENNNNNN SIX OR SEVEN
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.hardware.Globals;
@TeleOp(name = "rotate test")
public class rotatetest extends OpMode {
    private Motor revolver;
    private GamepadEx gamepadEx;

    ButtonReader reader;
    private double logicalTarget = 0;



    @Override
    public void init() {


        //revolver = hardwareMap.get(DcMotor.class, "revolver");
        //revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //revolver.setTargetPosition(0);
        //revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        revolver = new Motor(hardwareMap, "revolver", Globals.cpr, Globals.rpm);
        revolver.setRunMode(Motor.RunMode.PositionControl);
        revolver.setPositionCoefficient(Globals.revolverKP);
        gamepadEx = new GamepadEx(gamepad1);
        reader = new ButtonReader(
                gamepadEx, GamepadKeys.Button.SQUARE
        );
        revolver.resetEncoder();

    }

    @Override
    public void loop() {
        gamepadEx.readButtons();
        revolver.setPositionCoefficient(Globals.revolverKP);
        revolver.setPositionTolerance(Globals.revolverTol);
        if (reader.wasJustPressed()) {
            logicalTarget += Globals.oneRotation * 0.75;  // 270 degrees
        } else if (gamepadEx.wasJustPressed(GamepadKeys.Button.CROSS)) {
            logicalTarget -= Globals.oneRotation * 0.75;
        }

        revolver.setTargetPosition((int)logicalTarget);

        if (!revolver.atTargetPosition()) {
            revolver.set(0.3);
        } else {
            revolver.set(0);
        }


    }
}
