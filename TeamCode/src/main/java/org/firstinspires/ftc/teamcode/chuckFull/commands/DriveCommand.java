package org.firstinspires.ftc.teamcode.chuckFull.commands;


import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.chuckFull.subsystems.DrivebaseSubsystem;


public class DriveCommand extends CommandBase {
    private final DrivebaseSubsystem drive;
    private final GamepadEx driver;


    public DriveCommand(DrivebaseSubsystem drive, GamepadEx driver) {
        this.drive = drive;
        this.driver = driver;
        addRequirements(drive);
    }


    @Override
    public void execute() {
        drive.drive(driver);
    }
}