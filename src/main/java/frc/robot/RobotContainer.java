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
    Constants.Subsystems.DRIVEBASE_SUBSYSTEM.setDefaultCommand(
      new InstantCommand(() -> {
      var Profile = Profiles.JOHN_DOE.PROFILE;
      DrivebaseSubsystem.set(
        Profile.getController().getRawAxis((Integer)Profile.getField(Profiles.FIELDS.TRANSLATION_HORIZONTAL_INPUT)),
        Profile.getController().getRawAxis((Integer)Profile.getField(Profiles.FIELDS.TRANSLATION_VERTICAL_INPUT)),
        Profile.getController().getRawAxis((Integer)Profile.getField(Profiles.FIELDS.ORIENTATION_INPUT)),
        () -> false);
      }));
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//

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
