package org.firstinspires.ftc.teamcode.chuckFull.commands;


import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.chuckFull.subsystems.PatternSubsystem;


public class PatternDetectCommand extends CommandBase {
    private final PatternSubsystem pattern;
    private final GamepadEx driver;


    public PatternDetectCommand(PatternSubsystem pattern, GamepadEx driver) {
        this.pattern = pattern;
        this.driver = driver;
        addRequirements(pattern);
    }


    @Override
    public void execute() {
        pattern.periodic(driver);
    }
}