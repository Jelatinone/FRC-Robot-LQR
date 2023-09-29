// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import frc.robot.Constants.Profiles;
import frc.robot.Constants.Profiles.KeybindingNames;
import frc.robot.Constants.Profiles.PreferenceNames;
import frc.lib.PilotProfile;

// ----------------------------------------------------------[Robot Container Class]--------------------------------------------------------//
public final class RobotContainer {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static RobotContainer INSTANCE = (null);
  public static PilotProfile DRIVEBASE_PILOT = Profiles.Cody.PROFILE;
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//

  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private RobotContainer() {

  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Apply subsystem default commands
   */
  public static void applySubsystemDefaults() {
    Constants.Subsystems.DRIVEBASE_SUBSYSTEM.setDefaultCommand(
      new InstantCommand(() ->
        DrivebaseSubsystem.set(
          applyInputSquare(applyInputDeadzone(-(Double) DRIVEBASE_PILOT.getPreference(PreferenceNames.TRANSLATIONAL_X_INPUT),
          (Double) DRIVEBASE_PILOT.getPreference(PreferenceNames.TRANSLATIONAL_X_DEADZONE))),
          applyInputSquare(applyInputDeadzone((Double) DRIVEBASE_PILOT.getPreference(PreferenceNames.TRANSLATIONAL_Y_INPUT),
          (Double) DRIVEBASE_PILOT.getPreference(PreferenceNames.TRANSLATIONAL_Y_DEADZONE))),
          applyInputSquare(applyInputDeadzone((Double) DRIVEBASE_PILOT.getPreference(PreferenceNames.ORIENTATION_INPUT),
          (Double) DRIVEBASE_PILOT.getPreference(PreferenceNames.ORIENTATION_DEADZONE))),
          () -> (false)), 
          DrivebaseSubsystem.getInstance()
    ));

  }

  /**
   * Remove subsystem default commands
   */
  public static void removeSubsystemDefaults() {
    Constants.Subsystems.SUBSYSTEMS.forEach(SubsystemBase::removeDefaultCommand);
  }

  /**
   * Configure all pilot keybindings to proper event triggers
   */
  public static void configureKeybindings() {
    Constants.Profiles.PILOT_PROFILES.forEach((Profile) -> {
      Profile.getKeybinding(KeybindingNames.FIELD_ORIENTED_TOGGLE).onTrue(new InstantCommand(DrivebaseSubsystem::toggleFieldOriented, DrivebaseSubsystem.getInstance()));
      Profile.getKeybinding(KeybindingNames.LOCKING_TOGGLE_TRIGGER).onTrue(new InstantCommand(DrivebaseSubsystem::toggleLockingEnabled, DrivebaseSubsystem.getInstance()));
    });
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  public static Double applyInputDeadzone(final Double Input, final Double Deadzone) {
    return (Math.abs(Input) > Deadzone)? (Input): (0.0);
  }

  public static Double applyInputSquare(final Double Input) {
    return (Input >= 0)? (Math.pow(Input,2)): (-Math.pow(Input,2));
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  public static synchronized RobotContainer getInstance() {
    if(java.util.Objects.isNull(INSTANCE)) {
      //noinspection InstantiationOfUtilityClass
      INSTANCE = new RobotContainer();
    }
    return INSTANCE;
  }
}
