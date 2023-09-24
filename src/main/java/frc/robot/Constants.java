// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.drivebase.DrivebaseSubsystem;
import frc.lib.PilotProfile;

// ------------------------------------------------------------[Constants Class]------------------------------------------------------------//
public final class Constants {
    
    public static final class Subsystems {

        public static final DrivebaseSubsystem DRIVEBASE_SUBSYSTEM = DrivebaseSubsystem.getInstance();

    }
    
    public static final class AdvantageKit {
        public static final Boolean REPLAY_FROM_LOG = (false);
    }


    public static final class Profiles { 

        public static final class JOHN_DOE {
            public static final Integer CONTROLLER_PORT = (0);
            public static final PilotProfile PROFILE = new PilotProfile(new CommandXboxController(CONTROLLER_PORT));        
        }

        public static final class FIELDS {
            public static final String TRANSLATION_HORIZONTAL_INPUT = "TRANSLATION_HORIZONTAL_INPUT";
            public static final String TRANSLATION_VERTICAL_INPUT = "TRANSLATION_VERTICAL_INPUT";
            public static final String ORIENTATION_INPUT = "ORIENTATION_INPUT";
        }

    }

    public static final Logger LOGGER = Logger.getInstance();
    public static final Boolean TURBO_MODE = (false);
}
