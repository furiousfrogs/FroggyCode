package org.firstinspires.ftc.teamcode.commandbase;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.hardware.Globals;

@TeleOp(name = "outtake")
public class outtaketests extends OpMode {
    private Motor revolver;
    private GamepadEx gamepadEx;
    @Override
    public void init() {


        //revolver = hardwareMap.get(DcMotor.class, "revolver");
        //revolver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //revolver.setTargetPosition(0);
        //revolver.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        revolver = new Motor(hardwareMap, "revolver", 1538.0, 435);
        revolver.setRunMode(Motor.RunMode.RawPower);

        gamepadEx = new GamepadEx(gamepad1);
    }

    @Override
    public void loop(){

        if (gamepadEx.getButton(GamepadKeys.Button.SQUARE)) {
          revolver.set(1);
        } else if (gamepadEx.getButton(GamepadKeys.Button.CROSS)) {
           revolver.set(-1);
        } else {
            revolver.set(0);
        }
    }
}