// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.lib;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import java.util.function.Supplier;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.button.Trigger;
// -----------------------------------------------------------[Pilot Profile Class]---------------------------------------------------------//
/**
 * 
 * 
 * <h1> PilotProfile </h1>
 * 
 * <p> Represents a robot driver's personalized preferences and keybindings that can be used to control the robot to their desires. </p>
 * 
 * @author Cody Washington (@Jelatinone) 
 */
public final class PilotProfile {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private final Map<String,Supplier<Object>> PREFERENCES = new HashMap<>();  
  private final Map<String,Trigger> KEYBINDINGS = new HashMap<>();
  private final String PILOT_NAME;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Constructor.
   * @param Name Pilot's name to reference against other pilot profiles
   */
  public PilotProfile(final String Name) {
    PILOT_NAME = Name;
  }  
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  /**
   * Adds a new keybinding to the pilot's keybinding mapping
   * @param KeybindingName      Name of keybinding
   * @param KeybindingRetriever Keybinding Trigger
   */
  public PilotProfile addKeybinding(final String KeybindingName, final Trigger KeybindingRetriever) {
    KEYBINDINGS.put(KeybindingName, KeybindingRetriever);
    return this;
  }
  /**
   * Add a new preference to the pilot's preference mapping
   * @param ValueName Name of preference
   * @param Value Value of preference, can be any object
   */
  public PilotProfile addPreference(final String ValueName, final Supplier<Object> Value) {
    PREFERENCES.put(ValueName, Value);
    return this;
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Retrieves a keybinding from the pilot's keybinding mapping
   * @param KeybindingName Name of keybinding
   * @return A trigger from the pilot's keybinding map
   */
  public Trigger getKeybinding(final String KeybindingName) {
    return KEYBINDINGS.get(KeybindingName);
  }
  /**
   * Retrieves a preference from the pilot's preference mapping
   * @param ValueName Name of preference
   * @return An object from the pilot's keybinding map
   */
  public Object getPreference(final String ValueName) {
    return PREFERENCES.get(ValueName).get();
  }

  /**
   * Retrieves the pilot's name
   * @return Name of pilot
   */
  public String getName() {
    return PILOT_NAME;
  }
}
