package org.firstinspires.ftc.teamcode.chuckFull.commands;


import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.chuckFull.subsystems.RevolverSubsystem;


public class RevolverCommand extends CommandBase {
    private final RevolverSubsystem revolver;
    private final GamepadEx driver;


    public RevolverCommand(RevolverSubsystem revolver, GamepadEx driver) {
        this.revolver = revolver;
        this.driver = driver;
        addRequirements(revolver);
    }


    @Override
    public void execute() {
        revolver.periodic(driver);
    }
}