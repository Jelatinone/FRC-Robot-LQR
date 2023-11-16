// ------------------------------------------------------------------[Package]------------------------------------------------------------------//
package frc.lib.motion.control;
// -----------------------------------------------------------------[Libraries]-----------------------------------------------------------------//
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;

// -----------------------------------------------------------------[Component]-----------------------------------------------------------------//
/**
 *
 *
 * <h2>Component</h2>
 *
 * <p>Represents a device which can control some motion and take in some double matrix inputs, which
 * control the underlying hardware to produce some output, which can be measured to provide feedback
 * to an optional controller.
 */
public interface Component {
  // ---------------------------------------------------------------[Abstract]------------------------------------------------------------------//
  /**
   * Mutates current control mode while additionally setting the control reference.
   * @param Reference                            Controller setpoint, or goal state matrix to reach
   * @param ControlMode                          Control movement mode
   * @throws java.lang.IndexOutOfBoundsException When the dimensions of the reference do not match that expected by the controller
   */
  public void set(final Matrix<Num,N1> Demand, final ControlMode Mode);

  /**
   * Provides controller's current reference state matrix
   * @return A matrix of system states
   */
  public Matrix<Num,N1> getControllerReference();

    /**
   * Provides controller's current calculate output state matrix
   * @return A matrix of system states
   */
  public Matrix<Num,N1> getControllerOutputs();

    /**
   * Provides controller's current real (device) output state matrix
   * @return A matrix of system states
   */
  public Matrix<Num,N1> getDeviceOutput();

  // ---------------------------------------------------------------[Internal]------------------------------------------------------------------//
  /**
   * Represents a {@link Component}'s current mode of motion, with each mode having different
   * underlying behaviour and resultant motions.
   */
  public enum ControlMode {
    /**
     * Represents a components in state control mode with a closed loop
     */
    STATE_CONTROL_CLOSED,    
    /**
     * Represents a component in state control mode with an open loop
     */
    STATE_CONTROL_OPEN,    
    /**
     * Represents a component in characterization mode
     */
    CHARACTERIZATION,    
    /**
     * Represents a component in disabled mode
     */
    DISABLED, 
  }

  /**
   * Represents a {@link Component}'s current mode of calculation, with each mode having different
   * underlying behaviour and resultant motions.
   */
  public enum CalculationMode {
    /**
     * Represnets calculating expecting deamdn angle to be continous
     */
    CONTINOUS,

    /**
     * Represents calculating expecting demand to be wrapped
     */
    WRAPPED,
  }
}
