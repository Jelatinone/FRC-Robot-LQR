// ------------------------------------------------------------------[Package]------------------------------------------------------------------//
package frc.lib.motion.control;
// -----------------------------------------------------------------[Libraries]-----------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

// ----------------------------------------------------------[Linear System Component]----------------------------------------------------------//
/**
 * 
 * 
 * <h2>LinearSystemComponent</h2>
 * 
 * <p>Shape-safe wrapper for the {@link edu.wpi.first.math.system.LinearSystemLoop LinearSystemLoop} class, with additional support for angle wrapped, or unwrapped
 * when output is calculated, and support for characterizatiom.</p>
 * 
 * @author Cody Washington (Team 5411)
 * @see Component
 */
public class LinearSystemComponent implements Component {
  // ----------------------------------------------------------------[Shared]-------------------------------------------------------------------//
  private static final Logger LOGGER_INSTANCE = Logger.getInstance();
  // ---------------------------------------------------------------[Constants]-----------------------------------------------------------------//
  private final LinearSystemLoop<Num,Num,Num> CONTROL_SYSTEM;
  private final Supplier<Matrix<Num,N1>> OUTPUT_DEVICES;  
  private final Consumer<Matrix<Num,N1>> INPUT_DEVICES;
  private final CalculationMode CALCULATION_MODE;
  private final String REFERENCE_NAME;
  // ----------------------------------------------------------------[Fields]-------------------------------------------------------------------//
  private volatile Matrix<Num,N1> Control_Output;  
  private Matrix<Num,N1> Control_Characterization;  
  private Matrix<Num,N1> Control_Reference;
  private Double Characterization_Standard_Deviation;
  private Double Characterization_Standard_Increment;  
  private Double Control_Time_Reference;    
  private Integer Characterization_Starting_Row;
  private Integer Characterization_Ending_Row;
  private Integer Control_Subjective_Row;  
  private RepeatCommand Control_Command;    
  private ControlMode Control_Mode;  

  // ------------------------------------------------------------[Constructor(s)]---------------------------------------------------------------//
  /**
   * Linear System Component Constructor.
   * @param Controller      Linear-model based control system, which calculates the control outputs, {@code u}, based on a set reference {@code r}
   * @param OutputDevices   Device, or set of that produce some output motion based on a matrix of states which represent the controller output states, {@code u}
   * @param InputDevices    Device, or set of that produce some feedback data based on a matrix of states which represent the real system output states, {@code x}
   * @param CalculationType Set mode of calculting the outputs, whether it is utilizing wrapped or unwrapped angles
   * @param ReferenceName   A name for easy referencing of controller outputs and references with {@link org.littletonrobotics.junction.Logger logging}
   */
  public LinearSystemComponent(final LinearSystemLoop<Num,Num,Num> Controller, final Supplier<Matrix<Num,N1>> OutputDevices,
                            final Consumer<Matrix<Num,N1>> InputDevices,final CalculationMode CalculationType, final String ReferenceName) {
    CONTROL_SYSTEM = Controller;
    OUTPUT_DEVICES = OutputDevices;
    INPUT_DEVICES = InputDevices;
    CALCULATION_MODE = CalculationType;
    REFERENCE_NAME = ReferenceName;
    Control_Mode = ControlMode.DISABLED;
    Control_Reference = CONTROL_SYSTEM.getNextR();
    Control_Output = CONTROL_SYSTEM.getU();
    Control_Characterization = new MatBuilder<>(() -> Control_Reference.getNumRows(), Nat.N1()).fill((0.0));
    Control_Time_Reference = (0.0);
    Control_Command = new RepeatCommand(new InstantCommand((() -> {
      switch(Control_Mode) {
        case STATE_CONTROL_CLOSED:
          calculate();
          break;
        case STATE_CONTROL_OPEN:
          Control_Output = Control_Reference;
          break;
        case CHARACTERIZATION:
          characterize();
          break;
        case DISABLED:
          Control_Output.fill((0.0));
          break;
      }
      INPUT_DEVICES.accept(Control_Output);
    })));
    Control_Command.schedule();
   }
  // ----------------------------------------------------------------[Methods]------------------------------------------------------------------//
  /**
   * Shape-safe calculation of the error between the controller reference {@code r}, based on the predicted system states, {@code X-hat}
   * @param Reference Matrix of controller references, {@code R}, which must match the size of the expected controller reference
   * @param Predicted Matrix of the controller predictions, {@code X-hat}, which may be any of any height.
   * @return A calculation of the error between existing references and the predicted controller system states.
   */
  private Matrix<Num,N1> conform(final Matrix<Num, N1> Reference, final Matrix<Num, N1> Predicted) {
    var ControlReferenceIterator = new AtomicInteger();
    var ComponentOutputIterator = Arrays.stream(Predicted.getData()).iterator();
    return new MatBuilder<>(Reference::getNumRows, Nat.N1())
      .fill(Arrays.stream(Reference.getData()).boxed().mapToDouble(
        (ReferenceElement) -> {
          var ConformedElement = (ComponentOutputIterator.hasNext())? (ReferenceElement - ComponentOutputIterator.next()): (ReferenceElement);
          LOGGER_INSTANCE.recordOutput((REFERENCE_NAME + "/REFERENCE(" + ControlReferenceIterator.getAndIncrement() + ")"),ConformedElement);
          return ConformedElement;
        })
    .toArray());    
  }

  /**
   * Shape safe calculation of the controller's next control output, {@code U}, based on the mode of calculation (Wrapped or Contious)
   * and the controller reference, {@code R}, only computing values within range of the controller's expected reference.
   */
  private synchronized void calculate() {
    CONTROL_SYSTEM.setNextR(conform(
      Control_Reference.block(() -> CONTROL_SYSTEM.getNextR().getNumRows(),Nat.N1(),Control_Subjective_Row, (0)),
      CONTROL_SYSTEM.getXHat()));
    CONTROL_SYSTEM.correct(OUTPUT_DEVICES.get());
    var ControlOuputsCalculated = (CALCULATION_MODE.equals(CalculationMode.CONTINOUS))? 
      (CONTROL_SYSTEM.clampInput(
        CONTROL_SYSTEM
          .getController()
          .getK()
          .times(conform(CONTROL_SYSTEM.getNextR(), CONTROL_SYSTEM.getXHat())))):
      (CONTROL_SYSTEM.clampInput(
        CONTROL_SYSTEM
          .getController()
          .getK()
          .times(CONTROL_SYSTEM.getNextR())
          .plus(CONTROL_SYSTEM.getFeedforward().calculate(CONTROL_SYSTEM.getNextR()))));
    var DiscretizationTimestep = (0.0);
    if (Control_Time_Reference.equals((0.0))) {
      DiscretizationTimestep = ((1.0) / (50.0));
    } else {
      var MeasuredTime = Timer.getFPGATimestamp();
      DiscretizationTimestep = MeasuredTime - Control_Time_Reference;
      Control_Time_Reference = MeasuredTime;      
    }
    CONTROL_SYSTEM.getObserver().predict(ControlOuputsCalculated, DiscretizationTimestep);
    Control_Output = Control_Reference;
    Control_Output.assignBlock(Control_Subjective_Row, (0), ControlOuputsCalculated);
    var ControlReferenceIterator = new AtomicInteger();
    Arrays.stream(Control_Output.getData()).boxed().toList().forEach(
      (OutputElement) -> LOGGER_INSTANCE.recordOutput((REFERENCE_NAME + "/OUTPUT(" + ControlReferenceIterator.getAndIncrement() + ")"),OutputElement));
  }

  /**
   * Shape safe calculation of the controller's characterization constants, based on an initial state reference, {@code R},
   * to produce some motion output through iteration.
   */
  private synchronized void characterize() {
    Arrays.stream(OUTPUT_DEVICES.get().getData()).forEach((DataComponent) -> {
      if(Control_Mode.equals(ControlMode.CHARACTERIZATION)) { 
        if(Math.abs(DataComponent) > (Characterization_Standard_Deviation)) {
          Control_Mode = ControlMode.DISABLED;
    }}});
    if(Control_Mode.equals(ControlMode.CHARACTERIZATION)) {
      var LocalIterator = new AtomicInteger(Characterization_Starting_Row);
      Arrays.stream(Control_Characterization.getData()).forEach((DataComponent) -> {
        if(LocalIterator.get() >= Characterization_Ending_Row) {
          Control_Characterization.set(LocalIterator.getAndIncrement(),(0), DataComponent + (Characterization_Standard_Increment));
        }
      });    
      Control_Output.assignBlock(Control_Subjective_Row, (0), Control_Characterization);
    } else {
      Control_Output.fill((0.0));
    }
  }
  // ---------------------------------------------------------------[Mutators]------------------------------------------------------------------//
  /**
   * Mutates current control mode to another mode, additionally resets all elements of the reference matrix to zero
   * @param ControlMode Control movement mode
   */
  public synchronized void set(final ControlMode ControlMode) {
    Control_Reference.fill((0.0));
    Control_Mode = ControlMode;      
  }

  /**
   * Mutates current control reference into another reference.
   * @param Reference                            Controller setpoint, or goal state matrix to reach
   * @throws java.lang.IndexOutOfBoundsException When the dimensions of the reference do not match that expected by the controller
   */
  public synchronized void set(final Matrix<Num,N1> Reference) {
    if(Reference.getNumRows() != CONTROL_SYSTEM.getNextR().getNumRows()) {
      throw new IndexOutOfBoundsException("Next Reference exceeds controller reference dimensions");      
    } else {
      Control_Reference = Reference;
    }
  }

  /**
   * Mutates current control mode while additionally setting the control reference.
   * @param Reference                            Controller setpoint, or goal state matrix to reach
   * @param ControlMode                          Control movement mode
   * @throws java.lang.IndexOutOfBoundsException When the dimensions of the reference do not match that expected by the controller
   */
  public synchronized void set(final Matrix<Num,N1> Reference, final ControlMode ControlMode) {
    Control_Mode = ControlMode;
    if(Reference.getNumRows() != CONTROL_SYSTEM.getNextR().getNumRows()) {
      throw new IndexOutOfBoundsException("Next Reference exceeds controller reference dimensions");      
    } else {
      Control_Reference = Reference;
    }
  }

  /**
   * Mutates standard deviation to achieve for characterization
   * @param StandardDeviation Standard deviation to test against
   */
  public void setStandardDeviation(final Double StandardDeviation) {
    Characterization_Standard_Deviation = StandardDeviation;
  }

  /**
   * Mutates standard increment with each step for characterization
   * @param StandardIncrement Standard increment for each constant within characterization values
   */
  public void setStandardIncrement(final Double StandardIncrement) {
    Characterization_Standard_Increment = StandardIncrement;
  }

  /**
   * Mutates starting row for characterization
   * @param StartingRow Starting reference matrix row to extract characterization constants from
   */
  public void setStartingRow(final Integer StartingRow) {
    Characterization_Starting_Row = StartingRow;
  }

  /**
   * Mutates starting for for closed state control
   * @param SubjectRow Starting reference matrix row to extract controller reference constants from
   */
  public void setSubjectRow(final Integer SubjectRow) {
    Control_Subjective_Row = SubjectRow;
  }

  /**
   * Mutates ending row for characterization
   * @param EndingRow Ending reference matrix row to extract characterization constants from 
   */
  public void setEndingRow(final Integer EndingRow) {
    Characterization_Ending_Row = EndingRow;
  }
  // ---------------------------------------------------------------[Accessors]-----------------------------------------------------------------//
  /**
   * Provides controller's current reference state matrix
   * @return A matrix of system states
   */
  public Matrix<Num,N1> getControllerReference() {
    return Control_Reference;
  }

  /**
   * Provides controller's current calculate output state matrix
   * @return A matrix of system states
   */
  public Matrix<Num,N1> getControllerOutputs() {
    return Control_Output;
  }

  /**
   * Provides controller's current real (device) output state matrix
   * @return A matrix of system states
   */
  public Matrix<Num,N1> getDeviceOutput() {
    return OUTPUT_DEVICES.get();
  }

  /**
   * Provides the reference name for logging
   * @return A string reference name
   */
  public String getName() {
    return REFERENCE_NAME;
  }

}