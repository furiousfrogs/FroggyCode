package org.firstinspires.ftc.teamcode.chuckFull.commands;


import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.chuckFull.subsystems.TurretSubsystem;


public class TurretAutoAimCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final GamepadEx driver;


    public TurretAutoAimCommand(TurretSubsystem turret, GamepadEx driver) {
        this.turret = turret;
        this.driver = driver;
        addRequirements(turret);
    }


    @Override
    public void execute() {
        turret.periodic(driver);
    }
}