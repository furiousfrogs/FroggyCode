package org.firstinspires.ftc.teamcode.chuckFull.commands;


import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.chuckFull.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.chuckFull.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.chuckFull.subsystems.PatternSubsystem;


public class LauncherCommand extends CommandBase {
    private final LauncherSubsystem launcher;
    private final TurretSubsystem turret;
    private final PatternSubsystem pattern;
    private final GamepadEx driver;


    public LauncherCommand(LauncherSubsystem launcher, TurretSubsystem turret, PatternSubsystem pattern, GamepadEx driver) {
        this.launcher = launcher;
        this.turret = turret;
        this.pattern = pattern;
        this.driver = driver;
        addRequirements(launcher);
    }


    @Override
    public void execute() {
        launcher.periodic(driver, turret, pattern);
    }
}

