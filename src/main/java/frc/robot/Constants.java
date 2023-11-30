// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.PilotProfile;

import org.littletonrobotics.junction.Logger;

import java.util.List;

// ------------------------------------------------------------[Constants Class]------------------------------------------------------------//
public final class Constants {
    
    public static final class SubsystemManagement {

        public static final List<SubsystemBase> SUBSYSTEMS = List.of();
    }
    
    public static final class AdvantageKit {
        public static final Boolean IS_REAL_ROBOT = RobotBase.isReal();     
        public static final Boolean REPLAY_FROM_LOG = (false);
        public static final Boolean LOGGING_ENABLED = (false);

    }


    public static final class Profiles { 

        public static final List<PilotProfile> PILOT_PROFILES = List.of(TestPilot.PROFILE);


        public static final class PreferenceNames {
            public static final String TRANSLATIONAL_X_INPUT = ("TRANSLATION_X_INPUT");
            public static final String TRANSLATIONAL_Y_INPUT = ("TRANSLATION_Y_INPUT");
            public static final String ORIENTATION_INPUT = ("ORIENTATION_X_INPUT");
            public static final String TRANSLATIONAL_X_DEADZONE = ("TRANSLATIONAL_X_DEADZONE");
            public static final String TRANSLATIONAL_Y_DEADZONE = ("TRANSLATIONAL_Y_DEADZONE");
            public static final String ORIENTATION_DEADZONE = ("ORIENTATION_DEADZONE");
        }

        public static final class KeybindingNames {
            public static final String LOCKING_TOGGLE_TRIGGER = ("LOCKING_ENABLED_TRIGGER");
            public static final String FIELD_ORIENTED_TOGGLE = ("FIELD_ORIENTED_TOGGLE");
        }

        public static final class TestPilot {
            public static final Integer CONTROLLER_PORT = (0);
            public static final CommandXboxController CONTROLLER = new CommandXboxController(CONTROLLER_PORT);
            public static final PilotProfile PROFILE = new PilotProfile(("TestPilot"))
                .addPreference(PreferenceNames.TRANSLATIONAL_X_INPUT, () -> CONTROLLER.getLeftX())
                .addPreference(PreferenceNames.TRANSLATIONAL_Y_INPUT, () -> CONTROLLER.getLeftY())
                .addPreference(PreferenceNames.ORIENTATION_INPUT, () -> CONTROLLER.getRightX())
                .addPreference(PreferenceNames.TRANSLATIONAL_X_DEADZONE, () -> (0.1))
                .addPreference(PreferenceNames.TRANSLATIONAL_Y_DEADZONE, () -> (0.1))
                .addPreference(PreferenceNames.ORIENTATION_DEADZONE, () -> (0.1))
                .addKeybinding(KeybindingNames.LOCKING_TOGGLE_TRIGGER, CONTROLLER.a())
                .addKeybinding(KeybindingNames.FIELD_ORIENTED_TOGGLE, CONTROLLER.b());
        }
    }

    public static final Integer POWER_DISTRIBUTION_ID = (2);

    public static final Logger LOGGER = Logger.getInstance();

    public static final String LOGGING_FOLDER = ("src\\main\\java\\frc\\deploy\\logs");
    public static final Boolean TURBO_MODE = (false);
}
