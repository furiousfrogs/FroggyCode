package org.firstinspires.ftc.teamcode.deprsTest;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;



@TeleOp(name = "buttons")
public class buttonTest extends OpMode {
    private GamepadEx gamepadEx;

    @Override
    public void init() {

        gamepadEx = new GamepadEx(gamepad1);


    }

    @Override
    public void loop() { // TODO add revolver sequence logic
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
            telemetry.addLine("curcle ressed");
        }
        if (gamepadEx.wasJustReleased(GamepadKeys.Button.CROSS)) {
            telemetry.addLine("cross release");
        }
        if (gamepadEx.getButton(GamepadKeys.Button.TRIANGLE)) {
            telemetry.addLine("trkajbrelease");
        }
        telemetry.update();
    }
}


