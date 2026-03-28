package frc.robot.util.autoUtil;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class AutoCommandPicker {
  private RobotContainer container;

  enum CommandName {
    DO_NOTHING,
    LEFT_SWIPE,
    RIGHT_SWIPE;
  }

  private static SendableChooser<CommandName> mModeChooser = new SendableChooser<>();

  public AutoCommandPicker(RobotContainer container) {
    this.container = container;
    mModeChooser.addOption("shoot - left side", CommandName.LEFT_SWIPE);
    mModeChooser.addOption("shoot - right side", CommandName.RIGHT_SWIPE);
    mModeChooser.setDefaultOption("Nothing", CommandName.DO_NOTHING);
  }

  public Command getCommand() {
    CommandName choice = mModeChooser.getSelected();
    choice = choice == null ? CommandName.DO_NOTHING : choice;
    switch (choice) {
      case DO_NOTHING:
        return new InstantCommand();
      case LEFT_SWIPE:
        return container.getSwipeAutoCommand(false);
      case RIGHT_SWIPE:
        return container.getSwipeAutoCommand(true);

      default:
        return new InstantCommand();
    }
  }
}
