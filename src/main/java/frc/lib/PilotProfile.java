// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.lib;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Function;
import java.util.HashMap;
import java.util.Map;

// -----------------------------------------------------------[Pilot Profile Class]---------------------------------------------------------//

public final class PilotProfile {
    // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
    private final CommandGenericHID CONTROLLER;
    private final Map<String,Function<CommandGenericHID,Trigger>> BINDINGS = new HashMap<>();
    private final Map<String,Object> FIELDS = new HashMap<>();
    // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  
    // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
    public PilotProfile(final CommandGenericHID Controller) {
      CONTROLLER = Controller;
    }
    // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
    public void addBinding(final String BindingName, final Function<CommandGenericHID, Trigger> ControllerBinding) {
      BINDINGS.put(BindingName, ControllerBinding);
    }

    public void addField(final String FieldName, final Object Value) {
      FIELDS.put(FieldName, Value);
    }
    // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
    public Trigger getBinding(final String BindingName) {
      return BINDINGS.get(BindingName).apply(CONTROLLER);
    }

    public Object getField(final String FieldName) {
      return FIELDS.get(FieldName);
    }

    public CommandGenericHID getController() {
      return CONTROLLER;
    }
  

}
