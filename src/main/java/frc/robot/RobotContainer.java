// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Profiles;

// ----------------------------------------------------------[Robot Container Class]--------------------------------------------------------//
public final class RobotContainer {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static RobotContainer INSTANCE = (null);
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//

  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private RobotContainer() {

  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  public void activateSubsystemDefaults() {
    Constants.Subsystems.DRIVEBASE_SUBSYSTEM.setDefaultCommand(
            new InstantCommand(() ->
                    DrivebaseSubsystem.set(
                            (Double)Profiles.Test.PROFILE.getPreference(Profiles.PreferenceNames.TRANSLATIONAL_X_INPUT),
                            (Double)Profiles.Test.PROFILE.getPreference(Profiles.PreferenceNames.TRANSLATIONAL_Y_INPUT),
                            (Double)Profiles.Test.PROFILE.getPreference(Profiles.PreferenceNames.ORIENTATION_INPUT),
                            () -> true),DrivebaseSubsystem.getInstance()));

  }

  public void deactivateSubsystemDefaults() {
    Constants.Subsystems.DRIVEBASE_SUBSYSTEM.removeDefaultCommand();
  }

  public void configureKeybindings() {

  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//

  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  public static synchronized RobotContainer getInstance() {
    if(java.util.Objects.isNull(INSTANCE)) {
      //noinspection InstantiationOfUtilityClass
      INSTANCE = new RobotContainer();
    }
    return INSTANCE;
  }
}
