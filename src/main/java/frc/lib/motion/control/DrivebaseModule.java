// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.lib.motion.control;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.io.Closeable;

// ------------------------------------------------------------[Drive Module Class]---------------------------------------------------------//
/**
 * 
 * 
 * <h1> DrivebaseModule </h1>
 * 
 * <p> An common interface which represents any kind of drivebase subunit module, this module can <i>optionally</i> implement 
 * methods for translation and/or rotation as needed, meaning a drivebase module can represent any type of drivebase's
 * modules, whether it is a pair of tank drive motors, a mecanum wheel, or a swerve module. </p>
 * 
 * <p> For a drivebase module, it is reccomended that mutator methods such as {@link #accept(SwerveModuleState)} should
 * be synchronized when implemented for thread safety. </p>
 * 
 * <p> This class provides team standardization for drivetrains. </p>
 * 
 * @see {@link edu.wpi.first.networktables.NetworkTable NetworkTable}
 * 
 * @author Cody Washington (@Jelatinone)
 */
@FunctionalInterface
public interface DrivebaseModule extends Sendable, Closeable, Supplier<SwerveModuleState>, Consumer<SwerveModuleState> {

  /**
   * Shorthand for receiving the measured (encoder read, actual) motion of the module; must be implemented
   * 
   * @return The measured motion state
   */
  public SwerveModuleState get();

  /**
   * Stops all motion within this module, can be moved again with a call to {@link #accept(SwerveModuleState)}
   */
  public default void stop() {}

  /**
   * Post all telemtry updates from the current time to the previous call to this method
   */
  public default void post() {}

  /**
   * Close open sockets to all resources within the module, {@link #accept(SwerveModuleState)} can no longer be called
   */
  public default void close() {}

  /**
   * Reset the control loop system of the module, sets reference r and output u to zero
   */
  public default void reset() {}

  /**
   * Shorthand without suppliers for specifying the desired motion that the module should achieve
   * 
   * @param Demand The desired motion state
   */
  public default void accept(final SwerveModuleState Demand) {}

  /**
   * Initializes this object as a {@link edu.wpi.first.util.sendable.Sendable} which is sendable over the network
   * @param Builder The builder to apply values to
   */
  public default void initSendable(final SendableBuilder Builder) {}

    /**
     * Set the controllers to meet rotation, and translation demands packaged as a {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState}
     *
     * @param StateDemand The specified demand as a {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState}, or translation (velocity) as a {@link java.lang.Double Double}},
     *                    and Rotation as radians in a {@link edu.wpi.first.math.geometry.Rotation2d Rotation2d}
     * @param ControlType The type of control of meeting the velocity demand, whether it is open loop.
     */
  public default void set(final SwerveModuleState Demand, Supplier<Boolean> ControlType) {
    Supplier<SwerveModuleState> OptimizedDemand = () -> SwerveModuleState.optimize(Demand, getMeasuredPosition());
    new ParallelCommandGroup(
      new InstantCommand(() -> 
          setPosition(() -> new Rotation2d(OptimizedDemand.get().angle.getRadians()))),
       new InstantCommand(() -> 
          setVelocity(() -> OptimizedDemand.get().speedMetersPerSecond, ControlType))
      ).schedule();
  }

  public default void setVelocity(final Supplier<Double> Demand, final Supplier<Boolean> ControlType) {}

    /**
     * Set the rotation (position) controller to meet a specified demand using a control type
     *
     * @param Demand The specified demand as a rotation in two-dimensional space as a {@link edu.wpi.first.math.geometry.Rotation2d Rotation2d}
     */
  public default void setPosition(final Supplier<Rotation2d> Demand) {}


  
  /**
   * Optimize the swerve module state to reduce unesscessary rotations.
   * @param Desired  Generated demand state
   * @param Measured Measured anngle
   * @return An optimized state with reduced rotations
   */
  public static SwerveModuleState optimize(SwerveModuleState Desired, Rotation2d Measured) {
    double TargetPosition = scope(Measured.getDegrees(), Desired.angle.getDegrees());
    double TargetVelocity = Desired.speedMetersPerSecond;
    double DeltaPosition = TargetPosition - Measured.getDegrees();
    if (Math.abs(DeltaPosition) > 90){
        TargetVelocity = -TargetVelocity;
        TargetPosition = DeltaPosition > 90 ? (TargetPosition -= 180) : (TargetPosition += 180);
    }        
    return new SwerveModuleState(TargetVelocity, Rotation2d.fromDegrees(TargetPosition));
  }

  /**
   * Places the angles within the correct reference scope
   * @param Reference Referemce scope
   * @param Target    Target scope
   * @return Angle placed within the correct scope
   */
  private static double scope(double Reference, double Target) {
    double LowerBound;
    double UpperBound;
    double LowerOffset = Reference % 360;
    if (LowerOffset >= 0) {
      LowerBound = Reference - LowerOffset;
      UpperBound = Reference + (360 - LowerOffset);
    } else {
      UpperBound = Reference - LowerOffset;
      LowerBound = Reference - (360 + LowerOffset);
    }
    while (Target < LowerBound) {
      Target += 360;
    }
    while (Target > UpperBound) {
      Target -= 360;
    }
    if (Target - Reference > 180) {
      Target -= 360;
    } else if (Target - Reference < -180) {
      Target += 360;
    }
    return Target;
  }

  
    /**
     * Get the current state of the module as a {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState}.
     *
     * @return A {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState} representation of the module's motion
     */
    public default SwerveModuleState getMeasuredModuleState() {
      return new SwerveModuleState(Double.NaN, new Rotation2d(Double.NaN));
  }

  /**
   * Get the current position (rotation) in radians as a {@link edu.wpi.first.math.geometry.Rotation2d Rotation2d}
   *
   * @return A quantitative representation of the angle of the module
   */
  public default Rotation2d getMeasuredPosition() {
      return new Rotation2d(Double.NaN);
  }

  /**
   * Get the current velocity (translation) in m/s as a {@link java.lang.Double Double}
   *
   * @return A quantitative representation of the angle of the module
   */
  public default Double getMeasuredVelocity() {
      return Double.NaN;
  }

  /**
   * Get the current state of the module as a {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState}.
   *
   * @return A {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState} representation of the module's motion
   */
  public default SwerveModuleState getDemandModuleState() {
      return new SwerveModuleState(Double.NaN, new Rotation2d(Double.NaN));
  }

  /**
   * Get the current position (rotation) in radians as a {@link edu.wpi.first.math.geometry.Rotation2d Rotation2d}
   *
   * @return A quantitative representation of the angle of the module
   */
  public default Rotation2d getDemandPosition() {
      return new Rotation2d(Double.NaN);
  }

  /**
   * Get the current velocity (translation) in m/s as a {@link java.lang.Double Double}
   *
   * @return A quantitative representation of the angle of the module
   */
  public default Double getDemandVelocity() {
      return Double.NaN;
  }

  /**
   * Get the current percent output [-1,1] of the position(Rotation of Rotational controller)
   * 
   * @return A quantitative representation of the module's rotation percent output
   */
  public default Double getRotationalOutput() {
    return Double.NaN;
  }

  /**
   * Get the current percent output [-1,1] of the translation(Velocity of Translation controller)
   * 
   * @return A quantitative representation of the module's translation percent output
   */
  public default Double getTranslationalOutput() {
      return Double.NaN;
  }
  
}