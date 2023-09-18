// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import static frc.robot.subsystems.drivebase.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.server.PathPlannerServer;
import frc.lib.SwerveModule;
import java.util.List;

// -------------------------------------------------------[Drivebase Subsystem Class]-------------------------------------------------------//
public final class DrivebaseSubsystem extends SubsystemBase {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  public static transient DrivebaseSubsystem INSTANCE; 
  public static final List<SwerveModule> MODULES = List.of(FL_Module.Hardware.MODULE,FR_Module.Hardware.MODULE,RL_Module.Hardware.MODULE,RR_Module.Hardware.MODULE);
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private DrivebaseSubsystem() {
    PathPlannerServer.startServer(PATHPLANNER_SERVER_PORT);
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//

  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//

  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  public static synchronized DrivebaseSubsystem getInstance() {
    if(java.util.Objects.equals(INSTANCE, null)) {
      INSTANCE = new DrivebaseSubsystem();
    }
    return INSTANCE;
  }
}
