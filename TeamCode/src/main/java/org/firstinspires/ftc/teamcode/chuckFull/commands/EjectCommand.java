package org.firstinspires.ftc.teamcode.chuckFull.commands;


import com.seattlesolvers.solverslib.command.CommandBase;
import org.firstinspires.ftc.teamcode.chuckFull.subsystems.IntakeSubsystem;


public class EjectCommand extends CommandBase {
    private final IntakeSubsystem intake;


    public EjectCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }


    @Override
    public void execute() {
        intake.stop();
    }
}