// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot;
import org.littletonrobotics.junction.Logger;

// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;

// ------------------------------------------------------------[Constants Class]------------------------------------------------------------//
public final class Constants {
    
    public static final class Subsystems {

        public static final DrivebaseSubsystem DRIVEBASE_SUBSYSTEM = DrivebaseSubsystem.getInstance();

    }

    public static final Logger LOGGER = Logger.getInstance();
    public static final Boolean TURBO_MODE = (false);
}
