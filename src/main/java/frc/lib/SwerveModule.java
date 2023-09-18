// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.lib;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

// -----------------------------------------------------------[Motion Module Class]---------------------------------------------------------//
/**
 * 
 * 
 * <h1> SwerveModule <h1>
 * 
 * 
 * 
 */
public final class SwerveModule implements AutoCloseable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private final Supplier<TrapezoidProfile.Constraints> ROTATIONAL_MOTION_CONSTRAINTS;
  private final LinearSystemLoop<N2,N1,N1> MOTION_CONTROL_LOOP;  
  private final MotorControllerGroup TRANSLATION_CONTROLLER;
  private final MotorControllerGroup ROTATION_CONTROLLER;   
  private final Supplier<SwerveModuleState> STATE_SENSOR; 
  private final Supplier<Double> MAXIMUM_LINEAR_VELOCITY;

  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private transient TrapezoidProfile.State TargetPositionStateReference = new TrapezoidProfile.State();   
  private transient TrapezoidProfile.State TargetPositionState = (null);   
  private transient ParallelCommandGroup TargetStateCommand = new ParallelCommandGroup();  
  private transient Double TimeReference = (0.0);

  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//

  public SwerveModule(final MotorControllerGroup TranslationController, final MotorControllerGroup RotationController, final Supplier<SwerveModuleState> StateSensor,
     final Supplier<TrapezoidProfile.Constraints> RotationMotionConstraints, LinearSystemLoop<N2,N1,N1> MotionControlLoop, Supplier<Double> MaximumLinearVelocity) {
    TRANSLATION_CONTROLLER = TranslationController;
    ROTATION_CONTROLLER = RotationController;
    ROTATIONAL_MOTION_CONSTRAINTS = RotationMotionConstraints;
    MOTION_CONTROL_LOOP = MotionControlLoop;
    STATE_SENSOR = StateSensor;
    MAXIMUM_LINEAR_VELOCITY = MaximumLinearVelocity;
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  public synchronized void setVelocity(final Supplier<Translation2d> Demand, final Supplier<Boolean> ControlType) {
    var TranslationDemand = Demand.get();
    if(TranslationDemand != null & TranslationDemand.getNorm() != Double.NaN) {
      if(ControlType.get()) {
        TRANSLATION_CONTROLLER.set(TranslationDemand.getNorm() / MAXIMUM_LINEAR_VELOCITY.get());
      } else {
        TRANSLATION_CONTROLLER.set(0);
      }      
    }
  }

  public synchronized void setPosition(final Supplier<Rotation2d> Demand) {
    Rotation2d RotationDemand = Demand.get();
    if(RotationDemand != null & RotationDemand.getRadians() != Double.NaN) {
      Double IntervalTime;
      if(TimeReference != (0)) {
        var RealTime = Timer.getFPGATimestamp();
        IntervalTime = RealTime - TimeReference;
        TimeReference = RealTime;
      } else {
        IntervalTime = (0.02);
      }
      TrapezoidProfile.Constraints SystemConstraints = ROTATIONAL_MOTION_CONSTRAINTS.get();
      TargetPositionState = new TrapezoidProfile.State(RotationDemand.getRadians(), (0));
      TargetPositionStateReference = new TrapezoidProfile(SystemConstraints,TargetPositionState,TargetPositionStateReference).calculate(IntervalTime);
      MOTION_CONTROL_LOOP.setNextR(TargetPositionStateReference.position,TargetPositionStateReference.velocity);
      MOTION_CONTROL_LOOP.correct(VecBuilder.fill(STATE_SENSOR.get().angle.getRadians()));
      MOTION_CONTROL_LOOP.predict(IntervalTime);
      ROTATION_CONTROLLER.setVoltage(MOTION_CONTROL_LOOP.getU(0));
    }
  }

  public synchronized void set(final Supplier<MotionState> Demand, final Supplier<Boolean> ControlType) {
    var DemandState = Demand.get();
    set(() -> new SwerveModuleState(new Translation2d(DemandState.TRANSLATION.getX(), DemandState.TRANSLATION.getY()).getNorm(),
              new Rotation2d(Demand.get().ROTATION.getX(), Demand.get().ROTATION.getY())), () -> ControlType.get());
  } 

  public synchronized void set(final Supplier<SwerveModuleState> Demand, final BooleanSupplier ControlType) {
    TargetStateCommand.cancel();
    TargetStateCommand = new ParallelCommandGroup(
      new InstantCommand(() -> 
        setPosition(() -> new Rotation2d(Demand.get().angle.getRadians()))
      ),
      new InstantCommand(() -> {
        SwerveModuleState DemandState = Demand.get();
        setVelocity(() -> new Translation2d(DemandState.speedMetersPerSecond, DemandState.angle), () ->  ControlType.getAsBoolean());
      }));
    TargetStateCommand.repeatedly().schedule();
  }
  // --------------------------------------------------------------[Abstract]---------------------------------------------------------------//
  public void post(NetworkTableInstance Table) {

  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//

  public synchronized void reset(final Double EncoderPositionRadian, final Double EncoderVelocityRadiansPerSecond) {
    MOTION_CONTROL_LOOP.reset(VecBuilder.fill(EncoderPositionRadian,EncoderVelocityRadiansPerSecond));
    TargetPositionStateReference = new TrapezoidProfile.State(EncoderPositionRadian, EncoderVelocityRadiansPerSecond);

  }

  public synchronized void close() {
    TRANSLATION_CONTROLLER.close();
    ROTATION_CONTROLLER.close();
    TargetStateCommand.cancel();
  }

  public void disable() {
    TargetStateCommand.cancel();
    TRANSLATION_CONTROLLER.setVoltage((0));
    ROTATION_CONTROLLER.setVoltage((0));
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  public SwerveModuleState getModuleState() {
    return STATE_SENSOR.get();
  }

  public Double getVelocity() {
    return STATE_SENSOR.get().speedMetersPerSecond;
  }

  public Rotation2d getPosition() {
    return STATE_SENSOR.get().angle;
  }
}
