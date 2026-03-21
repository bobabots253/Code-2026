package frc.robot.util.autoUtil;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class AutoCommandPicker {
    enum CommandName {
        DO_NOTHING,
        TRENCH_ENTRY
    }

    private static SendableChooser<CommandName> mModeChooser = new SendableChooser<>();

    public AutoCommandPicker(){
        mModeChooser.addOption("shoot", CommandName.TRENCH_ENTRY);
        mModeChooser.setDefaultOption("Nothing", CommandName.DO_NOTHING);
    }

    public Command getCommand(){
        CommandName choice = mModeChooser.getSelected();
        choice = choice == null ? CommandName.DO_NOTHING:choice;
        switch (choice) {
            case DO_NOTHING:
                return new InstantCommand();
        
            default:
                return new InstantCommand();
        }
    }
}