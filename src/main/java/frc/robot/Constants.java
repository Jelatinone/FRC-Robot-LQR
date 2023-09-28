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

        public static final Boolean LOGGING_ENABLED = (false);
    }


    public static final class Profiles { 

        public static final class PreferenceNames {
            public static final String TRANSLATIONAL_X_INPUT = ("TRANSLATION_X_INPUT");
            public static final String TRANSLATIONAL_Y_INPUT = ("TRANSLATION_Y_INPUT");
            public static final String ORIENTATION_INPUT = ("ORIENTATION_X_INPUT");
        }

        public static final class Test {
            public static final Integer CONTROLLER_PORT = (0);
            public static final CommandXboxController CONTROLLER = new CommandXboxController(CONTROLLER_PORT);
            public static final PilotProfile PROFILE = new PilotProfile("JOHN-DOE")
                .addPreference(PreferenceNames.TRANSLATIONAL_X_INPUT, () -> CONTROLLER.getLeftX())
                .addPreference(PreferenceNames.TRANSLATIONAL_Y_INPUT, () -> CONTROLLER.getLeftY())
                .addPreference(PreferenceNames.ORIENTATION_INPUT, () -> CONTROLLER.getRightX());
        }

    }

    public static final Integer POWER_DISTRIBUTION_ID = (0);

    public static final Logger LOGGER = Logger.getInstance();

    public static final String LOGGING_FOLDER = ("src\\main\\java\\frc\\deploy\\logs");
    public static final Boolean TURBO_MODE = (false);
}
