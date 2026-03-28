package frc.robot.util.autoUtil;

import java.security.PublicKey;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class AutoCommandPicker {
  private RobotContainer container;
  enum CommandName {
    DO_NOTHING,
    TRENCH_ENTRY,
    LEFT_SWIPE,
    RIGHT_SWIPE;
  }

  private static SendableChooser<CommandName> mModeChooser = new SendableChooser<>();

  public AutoCommandPicker(RobotContainer container) {
    this.container = container;
    mModeChooser.addOption("shoot", CommandName.TRENCH_ENTRY);
    mModeChooser.setDefaultOption("Nothing", CommandName.DO_NOTHING);
  }

  public Command getCommand() {
    CommandName choice = mModeChooser.getSelected();
    choice = choice == null ? CommandName.DO_NOTHING : choice;
    switch (choice) {
      case DO_NOTHING:
        return new InstantCommand();

      default:
        return new InstantCommand();
    }
  }
}
