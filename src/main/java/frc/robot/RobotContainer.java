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
        (Double)Profile.getController().getRawAxis((Integer)Profile.getField(Profiles.FIELDS.TRANSLATION_HORIZONTAL_INPUT)),
        (Double)Profile.getController().getRawAxis((Integer)Profile.getField(Profiles.FIELDS.TRANSLATION_VERTICAL_INPUT)),
        (Double)Profile.getController().getRawAxis((Integer)Profile.getField(Profiles.FIELDS.ORIENTATION_INPUT)),
        () -> false);
      }));
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//

  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//

  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  public static synchronized RobotContainer getInstance() {
    if(java.util.Objects.isNull(INSTANCE)) {
      INSTANCE = new RobotContainer();
    }
    return INSTANCE;
  }
}
