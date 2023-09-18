// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.lib;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.Optional;

// ----------------------------------------------------------[Motion State Class]-----------------------------------------------------------//
/**
 * 
 * 
 * <h1> MotionState </h1>
 * 
 * <p> Describes a mechanism's real motion in three dimensional space. </p>
 * 
 * <p> A state is made up of a three dimensional {@link edu.wpi.first.math.geometry.Translation3d Translation}, and a three dimensional 
 * {@link edu.wpi.first.math.geometry.Rotation3d Rotation}, both of which are {@link java.util.Optional Optional} values that may or may
 * not exist upon creation of a state. </p>
 * 
 * @author Cody Washington
 * 
 */
public final class MotionState implements Interpolatable<MotionState>, Cloneable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  /**
   * The mechanism's real velocity or translation in three-dimensional space; may or may not exist upon creation of this state.
   * @see {@link java.util.Optional Optional} 
   */
  public final Translation3d TRANSLATION;  
  /**
   * The mechanism's real position or rotation in three-dimensional space;  may or may not exist upon creation of this state.
   * @see {@link java.util.Optional Optional} 
   */
  public final Rotation3d ROTATION;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Constructs a new MotionState
   * @param Translation - An optional value which may or may not exist descriibing three dimensional translation in space.
   * @param Rotation - An optional value which may or may not exist descriibing three dimensional rotation in space.
   */
  public MotionState(final Optional<Translation3d> Translation, final Optional<Rotation3d> Rotation) {
    this.TRANSLATION = (Translation.isPresent())? (new Translation3d(Double.NaN, Double.NaN, Double.NaN)): (Translation.get());
    this.ROTATION = (Rotation.isPresent())? (new Rotation3d(Double.NaN, Double.NaN, Double.NaN)): (Rotation.get());
  }
  /**
   * Constructs a new MotionState
   * @param Translation - An optional value which does exist descriibing three dimensional translation in space.
   * @param Rotation - An optional value which does exist descriibing three dimensional rotation in space.
   */
  public MotionState(final Translation3d Translation, final Rotation3d Rotation) {
    this.TRANSLATION = Translation;
    this.ROTATION = Rotation;
  }

  /**
   * Construct a new value-less MotionState
   */
  public MotionState() {
    this.TRANSLATION = new Translation3d(Double.NaN, Double.NaN, Double.NaN);
    this.ROTATION = new Rotation3d(Double.NaN, Double.NaN, Double.NaN);
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//

  /**
   * Returns the interpolated value, with the translation and rotation of this object assumed to be at the starting position
   * @param EndValue - The upper bound, or ending state
   * @param Distance - How far the upper and lower bounds are, [0,1]
   * @return The interpolated value
   */
  @Override
  public MotionState interpolate(final MotionState EndValue, final double Distance) {
    return new MotionState(
      TRANSLATION.interpolate(EndValue.TRANSLATION, Distance), 
      ROTATION.interpolate(EndValue.ROTATION, Distance));
  }

  /**
   * Subtract by another state
   * @param Subtractive - The state to subtract by
   * @return The state subtracted by the subtractive value
   */
  public MotionState subtract(final MotionState Subtractive) {
    return new MotionState(
      Optional.of(new Translation3d(
        TRANSLATION.getX() - Subtractive.TRANSLATION.getX(),
        TRANSLATION.getY() - Subtractive.TRANSLATION.getY(),
        TRANSLATION.getZ() - Subtractive.TRANSLATION.getZ())),
      Optional.of(new Rotation3d(
        ROTATION.getX() - Subtractive.ROTATION.getX(),
        ROTATION.getY() - Subtractive.ROTATION.getY(),
        ROTATION.getZ() - Subtractive.ROTATION.getZ())));
  }

  /**
   * Multiply by another state
   * @param Factor - The state to multiply by
   * @return The state multiplied by the factor
   */
  public MotionState multiply(final MotionState Factor) {
      return new MotionState(
        Optional.of(new Translation3d(
          TRANSLATION.getX() * Factor.TRANSLATION.getX(),
          TRANSLATION.getY() * Factor.TRANSLATION.getY(),
          TRANSLATION.getZ() * Factor.TRANSLATION.getZ())),
        Optional.of(new Rotation3d(
          ROTATION.getX() * Factor.ROTATION.getX(),
          ROTATION.getY() * Factor.ROTATION.getY(),
          ROTATION.getZ() * Factor.ROTATION.getZ())));
  }

  /**
   * Divide by another state
   * @param Divisor - The state to divide by
   * @return The state divided by the divisor
   */
  public MotionState divide(final MotionState Divisor) {
    return new MotionState(
      Optional.of(new Translation3d(
        TRANSLATION.getX() / Divisor.TRANSLATION.getX(),
        TRANSLATION.getY() / Divisor.TRANSLATION.getY(),
        TRANSLATION.getZ() / Divisor.TRANSLATION.getZ())),
      Optional.of(new Rotation3d(
        ROTATION.getX() / Divisor.ROTATION.getX(),
        ROTATION.getY() / Divisor.ROTATION.getY(),
        ROTATION.getZ() / Divisor.ROTATION.getZ())));
  }

  /**
   * Add by another state
   * @param Additive - The state to add by
   * @return The state subtracted by the additive value
   */
  public MotionState add(final MotionState Additive) {
    return new MotionState(
      Optional.of(new Translation3d(
        TRANSLATION.getX() + Additive.TRANSLATION.getX(),
        TRANSLATION.getY() + Additive.TRANSLATION.getY(),
        TRANSLATION.getZ() + Additive.TRANSLATION.getZ())),
      Optional.of(new Rotation3d(
        ROTATION.getX() + Additive.ROTATION.getX(),
        ROTATION.getY() + Additive.ROTATION.getY(),
        ROTATION.getZ() + Additive.ROTATION.getZ())));
  }

  /**
   * Creates a copy of this state.
   */
  @Override
  public MotionState clone() {
    return new MotionState(TRANSLATION, ROTATION);
  }  
}