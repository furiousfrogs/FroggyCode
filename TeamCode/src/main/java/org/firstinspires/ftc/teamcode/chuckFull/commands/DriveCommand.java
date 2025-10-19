package org.firstinspires.ftc.teamcode.chuckFull.commands;

import org.firstinspires.ftc.teamcode.chuckFull.subsystems.Drivebase;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;


public class DriveCommand extends CommandBase {
    private final Drivebase drivebase;
    private final GamepadEx gamepad;

    public DriveCommand(Drivebase drivebase, GamepadEx gamepad) {
        this.drivebase = drivebase;
        this.gamepad = gamepad;
        addRequirements(drivebase);
    }

    @Override
    public void execute() {
        // Left stick drives, triggers turn
        double y  = -gamepad.getLeftY();
        double x  = gamepad.getLeftX();
        double rx = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) - gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        drivebase.drive(y, x, rx);
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.stop();
    }
}