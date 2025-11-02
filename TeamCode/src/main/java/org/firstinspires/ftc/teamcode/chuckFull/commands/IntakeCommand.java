package org.firstinspires.ftc.teamcode.chuckFull.commands;


import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.chuckFull.subsystems.IntakeSubsystem;


public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final GamepadEx driver;


    public IntakeCommand(IntakeSubsystem intake, GamepadEx driver) {
        this.intake = intake;
        this.driver = driver;
        addRequirements(intake);
    }


    @Override
    public void execute() {
        intake.periodic(driver);
    }
}