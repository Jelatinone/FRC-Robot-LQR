// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.lib.motion.control;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.util.sendable.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;
import frc.lib.MotionState;
// -------------------------------------------------------[Linear System Module Class]------------------------------------------------------//
/**
 * 
 * 
 * <h1> LinearSystemModule </h1>
 *
 * <p> Represents an {@link  edu.wpi.first.math.system.LinearSystemLoop LinearSystemLoop} based approach to a individual swerve
 * module; a module that has full control over both the translation (velocity), and rotation(position) of it's wheel. Meaning that it
 * has the capability to move in any direction. </p>
 *
 * <p> SwerveModules can consume a {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState} as a demand, and computes
 * the necessary voltage using a {@link  edu.wpi.first.math.system.LinearSystemLoop LinearSystemLoop} control loop. <p>
 *
 * @author Cody Washington (@Jelatinone)
 */
public final class LinearSystemModule implements DrivebaseModule  {
    // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
    private final TrapezoidProfile.Constraints ROTATIONAL_MOTION_CONSTRAINTS;
    private final Double MAXIMUM_TRANSLATIONAL_VELOCITY;
    private final LinearSystemLoop<N2, N1, N1> MOTION_CONTROL_LOOP;
    private final MotorControllerGroup TRANSLATION_CONTROLLER;
    private final MotorControllerGroup ROTATION_CONTROLLER;
    private final Supplier<SwerveModuleState> STATE_SENSOR;
    private final Integer REFERENCE_NUMBER;
    private static final Logger LOGGER = Logger.getInstance();
    private static Integer INSTANCE_COUNT = 0;    
    // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
    private TrapezoidProfile.State TargetPositionStateReference = new TrapezoidProfile.State();
    private RepeatCommand TargetStateCommand = new RepeatCommand(new InstantCommand());
    private SwerveModuleState DemandState = new SwerveModuleState();
    private Double TimeReference = (0.0);
    // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//

    /**
     * Constructor.
     *
     * @param TranslationController      The motor controller(s) responsible for controlling linear translation (velocity); queried with {@link #setVelocity(Supplier, Supplier)}
     * @param RotationController         The motor controller(s) responsible for controlling azimuth rotation (position); queried with {@link #setPosition(Supplier)}
     * @param StateSensor                The sum all collected sensor(s) data into a single {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState} supplier
     * @param MaximumTranslationVelocity The constraints placed on the Translation Controller which determine maximum velocity output
     * @param RotationMotionConstraints  The constraint placed on the RotationController which determine maximum velocity output, and maximum change in velocity in an instant time
     * @param MotionControlLoop          The control loop responsible for controller azimuth output
     */
    public LinearSystemModule(final MotorControllerGroup TranslationController, final MotorControllerGroup RotationController, final Supplier<SwerveModuleState> StateSensor, final Double MaximumTranslationVelocity, final TrapezoidProfile.Constraints RotationMotionConstraints, LinearSystemLoop<N2, N1, N1> MotionControlLoop) {
        MAXIMUM_TRANSLATIONAL_VELOCITY = MaximumTranslationVelocity;
        ROTATIONAL_MOTION_CONSTRAINTS = RotationMotionConstraints;
        TRANSLATION_CONTROLLER = TranslationController;
        ROTATION_CONTROLLER = RotationController;
        MOTION_CONTROL_LOOP = MotionControlLoop;
        STATE_SENSOR = StateSensor;
        REFERENCE_NUMBER = INSTANCE_COUNT;
        INSTANCE_COUNT++;
        SendableRegistry.addLW(this, "LinearSystemModule", REFERENCE_NUMBER);
    }
    // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//

    /**
     * Set the translation (linear velocity) controller to meet a specified demand using a control type
     *
     * @param Demand      The specified demand as a translation in two-dimensional space in meters / second
     * @param ControlType The type of control of meeting the demand, whether it is open loop.
     */
    @Override
    public synchronized void setVelocity(final Supplier<Double> Demand, final Supplier<Boolean> ControlType) {
        var TranslationDemand = Demand.get();
        TranslationDemand = Objects.isNull(TranslationDemand)? (0.0): (TranslationDemand);
        if (!Double.isNaN(TranslationDemand) && Math.abs(TranslationDemand - getTranslationalOutput()) > (2e-2)) {
            if (ControlType.get()) {
                TRANSLATION_CONTROLLER.set(TranslationDemand * (MAXIMUM_TRANSLATIONAL_VELOCITY));
            } else {
                TRANSLATION_CONTROLLER.set(TranslationDemand);
            }
        }
    }

    /**
     * Set the rotation (position) controller to meet a specified demand using a control type
     *
     * @param Demand The specified demand as a rotation in two-dimensional space as a {@link edu.wpi.first.math.geometry.Rotation2d Rotation2d}
     */
    @Override
    public synchronized void setPosition(final Supplier<Rotation2d> Demand) {
        Rotation2d RotationDemand = Demand.get();
        if (RotationDemand != null & !Double.isNaN(RotationDemand.getRadians())) {
            double IntervalTime;
            if (TimeReference != (0)) {
                var RealTime = Timer.getFPGATimestamp();
                IntervalTime = RealTime - TimeReference;
                TimeReference = RealTime;
            } else {
                IntervalTime = (0.02);
            }
            TrapezoidProfile.State targetPositionState = new TrapezoidProfile.State(RotationDemand.getRadians(), (0));
            TargetPositionStateReference = new TrapezoidProfile(ROTATIONAL_MOTION_CONSTRAINTS, targetPositionState, TargetPositionStateReference).calculate(IntervalTime);
            MOTION_CONTROL_LOOP.setNextR(TargetPositionStateReference.position, TargetPositionStateReference.velocity);
            MOTION_CONTROL_LOOP.correct(VecBuilder.fill(STATE_SENSOR.get().angle.getRadians()));
            MOTION_CONTROL_LOOP.predict(IntervalTime);
            ROTATION_CONTROLLER.setVoltage(MOTION_CONTROL_LOOP.getU((0)));
        }
    }

    /**
     * Set the controllers to meet rotation, and translation demands packaged as a {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState}
     *
     * @param Demand      The specified demand as a {@link frc.lib.MotionState MotionState} that has translation and rotation in three-dimensional space
     * @param ControlType The type of control of meeting the velocity demand, whether it is open loop.
     */
    @Deprecated
    public synchronized void set(final Supplier<MotionState> Demand, final Supplier<Boolean> ControlType) {
        var DemandMotion = Demand.get();
        DemandState = new SwerveModuleState(
            new Translation2d(DemandMotion.TRANSLATION.getX(),
            DemandMotion.TRANSLATION.getY()).getNorm(),
            new Rotation2d(Demand.get().ROTATION.getX(),
              Demand.get().ROTATION.getY()));
        set(DemandState, ControlType);              
    }

    /**
     * Set the controllers to meet rotation, and translation demands packaged as a {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState}
     *
     * @param StateDemand The specified demand as a {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState}, or translation (velocity) as a {@link java.lang.Double Double}},
     *                    and Rotation as radians in a {@link edu.wpi.first.math.geometry.Rotation2d Rotation2d}
     * @param ControlType The type of control of meeting the velocity demand, whether it is open loop.
     */
    @Override
    public synchronized void set(final SwerveModuleState StateDemand, final Supplier<Boolean> ControlType) {
        Supplier<SwerveModuleState> OptimizedDemand = () -> SwerveModuleState.optimize(StateDemand, getMeasuredPosition());
        TargetStateCommand.cancel();
        TargetStateCommand = new ParallelCommandGroup(
        new InstantCommand(() -> 
            setPosition(() -> new Rotation2d(OptimizedDemand.get().angle.getRadians()))),
         new InstantCommand(() -> 
            setVelocity(() -> OptimizedDemand.get().speedMetersPerSecond, ControlType))
        ).repeatedly();
        TargetStateCommand.schedule();
        DemandState = OptimizedDemand.get();
    }
    // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//

    /**
     * Reset the control loop system of the module, sets reference r and output u to zero
     */
    public synchronized void reset() {
        var State = STATE_SENSOR.get();
        var EncoderPositionRadian = State.angle.getRadians();
        var EncoderPositionRadiansPerSecond = State.angle.getRadians() / (TimeReference - Timer.getFPGATimestamp());
        MOTION_CONTROL_LOOP.reset(VecBuilder.fill(EncoderPositionRadian, EncoderPositionRadiansPerSecond));
        TargetPositionStateReference = new TrapezoidProfile.State(EncoderPositionRadian, EncoderPositionRadiansPerSecond);

    }

    /**
     * Close the held resources within the module, module is no longer usable after this operation.
     */
    @Override
    public synchronized void close() {
        SendableRegistry.remove(this);
        TRANSLATION_CONTROLLER.stopMotor();
        ROTATION_CONTROLLER.stopMotor();
        TRANSLATION_CONTROLLER.close();
        ROTATION_CONTROLLER.close();
        TargetStateCommand.cancel();
    }

    /**
     * Stop all the controllers within the module immediately, and cancel all subsequent motions. make another call to {@link #set(SwerveModuleState, Supplier)} to call again.
     */
    @Override
    public void stop() {
        DemandState = new SwerveModuleState((0.0),getDemandPosition());
        TargetStateCommand.cancel();
        TRANSLATION_CONTROLLER.stopMotor();
        ROTATION_CONTROLLER.stopMotor();
    }

    /**
     * Consume a SwerveModuleState as shorthand for calling {@link #set(SwerveModuleState, Supplier), with false as a ControlType}
     *
     * @param Demand The desired state for the module to achieve
     */
    @Override
    public void accept(final SwerveModuleState Demand) {
        set(Demand, () -> false);
    }

    @Override
    public void initSendable(SendableBuilder Builder) {
        Builder.setSmartDashboardType("LinearSystemModule");
        Builder.addDoubleProperty(("Translation Velocity"), this::getMeasuredVelocity, (Demand) -> setVelocity(() -> Demand, () -> false));
        Builder.addDoubleProperty(("Translation Output"), this::getTranslationalOutput, null);
        Builder.addDoubleProperty(("Rotation Position"), () -> getMeasuredPosition().getRadians(), (Demand) -> setPosition(() -> new Rotation2d(Demand)));
        Builder.addDoubleProperty(("Rotation Velocity"), () -> TargetPositionStateReference.velocity, (null));
        Builder.addDoubleProperty(("Rotation Output"), this::getRotationalOutput, null);
    }

    /**
     * Post measurement, output, and demand data to shuffleboard static  instance
     */
    public void post() {
        var Prefix = ("Module [") + REFERENCE_NUMBER + ("]/");
        SmartDashboard.putNumber(Prefix + "Demand Rotation", DemandState.angle.getDegrees());
        SmartDashboard.putNumber(Prefix + "Demand Rotation Velocity",TargetPositionStateReference.velocity);        
        SmartDashboard.putNumber(Prefix + "Demand Velocity", DemandState.speedMetersPerSecond);
        SmartDashboard.putNumber(Prefix + "Measured Rotation", getMeasuredPosition().getDegrees());
        SmartDashboard.putNumber(Prefix + "Measured Velocity", getMeasuredVelocity());
        SmartDashboard.putNumber(Prefix + "Output Rotation", ROTATION_CONTROLLER.get());
        SmartDashboard.putNumber(Prefix + "Output Velocity", TRANSLATION_CONTROLLER.get());
        LOGGER.recordOutput((Prefix + "DemandAzimuthPosition"), DemandState.angle.getDegrees());
        LOGGER.recordOutput((Prefix + "DemandAzimuthPositionVelocity"), TargetPositionStateReference.velocity);
        LOGGER.recordOutput((Prefix + "DemandTranslationVelocity"), DemandState.speedMetersPerSecond);
        LOGGER.recordOutput((Prefix + "MeasuredAzimuthRotation"), getMeasuredPosition().getDegrees());
        LOGGER.recordOutput((Prefix + "MeasuredTranslationVelocity"), getMeasuredVelocity());
        LOGGER.recordOutput((Prefix + "OutputAzimuthPercent"), ROTATION_CONTROLLER.get());
        LOGGER.recordOutput((Prefix + "OutputTranslationPercent"), TRANSLATION_CONTROLLER.get());
    }

    /**
     * Configure a {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonFX WPI_TalonFX} rotational controller to swerve module specifications
     *
     * @param Controller     Controller that is to be configured
     * @param AzimuthEncoder Azimuth CANCoder instance acting as a feedback filter reference point, and an azimuth sensor position source
     * @param CurrentLimit   The current limit configuration of the motor's stator.
     * @param Deadband       Desired percent deadband of controller inputs
     * @return A copy of the configured controller
     */
    public static WPI_TalonFX configureRotationController(final WPI_TalonFX Controller, final WPI_CANCoder AzimuthEncoder, final StatorCurrentLimitConfiguration CurrentLimit, final Double Deadband) {
        Controller.configFactoryDefault();
        Controller.setInverted(TalonFXInvertType.CounterClockwise);
        Controller.setNeutralMode(NeutralMode.Brake);
        Controller.configRemoteFeedbackFilter(AzimuthEncoder, (0));
        Controller.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        Controller.configStatorCurrentLimit(CurrentLimit);
        Controller.setSelectedSensorPosition(AzimuthEncoder.getAbsolutePosition());
        Controller.configNeutralDeadband(Deadband);
        Controller.configVoltageCompSaturation((12));
        Controller.setInverted(TalonFXInvertType.CounterClockwise);
        return Controller;
    }

    /**
     * Configure a {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonFX WPI_TalonFX} translation controller to swerve module specifications
     *
     * @param Controller   Controller that is to be configured
     * @param CurrentLimit The current limit configuration of the motor's stator.
     * @return A copy of the configured controller
     */
    public static WPI_TalonFX configureTranslationController(final WPI_TalonFX Controller, final StatorCurrentLimitConfiguration CurrentLimit) {
        Controller.configFactoryDefault();
        Controller.setInverted(TalonFXInvertType.CounterClockwise);
        Controller.setNeutralMode(NeutralMode.Brake);
        Controller.configStatorCurrentLimit(CurrentLimit);
        Controller.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        Controller.setSelectedSensorPosition((0.0));
        Controller.enableVoltageCompensation((true));
        return Controller;
    }

    /**
     * Configure a {@link com.ctre.phoenix.sensors.CANCoder CANCoder} azimuth encoder to swerve module specifications
     *
     * @param AzimuthEncoder Encoder that is to be configured
     * @param Offset         The starting encoder's offset
     * @return A copy of the configured encoder
     */
    public static WPI_CANCoder configureRotationEncoder(final WPI_CANCoder AzimuthEncoder, final Double Offset) {
        AzimuthEncoder.configFactoryDefault();
        AzimuthEncoder.configMagnetOffset(Offset);
        AzimuthEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        AzimuthEncoder.setPositionToAbsolute();
        return AzimuthEncoder;
    }

    /**
     * Configure a {@link com.revrobotics.CANSparkMax CANSparkMax} translation controller to swerve module specifications
     *
     * @param Controller     Controller that is to be configured
     * @param AmpLimit       The current limit of the controller measured in amps
     * @param NominalVoltage The nominal voltage to compensate output of the controller to compensate voltage for
     * @return A copy of the configured controller
     */
    public static CANSparkMax configureTranslationController(final CANSparkMax Controller, final Integer AmpLimit, Double NominalVoltage) {
        Controller.restoreFactoryDefaults();
        Controller.setSmartCurrentLimit(AmpLimit);
        Controller.enableVoltageCompensation(NominalVoltage);
        Controller.setInverted((true));
        Controller.setIdleMode(IdleMode.kBrake);
        Controller.burnFlash();
        return Controller;
    }

    /**
     * Configure a {@link com.revrobotics.CANSparkMax CANSparkMax} rotation controller to swerve module specifications
     *
     * @param Controller     Controller that is to be configured
     * @param AmpLimit       The current limit of the controller measured in amps
     * @param NominalVoltage The nominal voltage to compensate output of the controller to compensate voltage for
     * @return A copy of the configured controller
     */
    public static CANSparkMax configureRotationController(final CANSparkMax Controller, final Integer AmpLimit, final Double NominalVoltage) {
        Controller.restoreFactoryDefaults();
        Controller.setSmartCurrentLimit(AmpLimit);
        Controller.enableVoltageCompensation(NominalVoltage);
        Controller.setInverted((true));
        Controller.setIdleMode(IdleMode.kBrake);
        Controller.burnFlash();
        return Controller;
    }
    // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//

    /**
     * Get the current state of the module as a {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState}.
     *
     * @return A {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState} representation of the module's motion
     */
    @Override
    public SwerveModuleState getMeasuredModuleState() {
        return STATE_SENSOR.get();
    }

    /**
     * Get the current position (rotation) in radians as a {@link edu.wpi.first.math.geometry.Rotation2d Rotation2d}
     *
     * @return A quantitative representation of the angle of the module
     */
    @Override
    public Rotation2d getMeasuredPosition() {
        return STATE_SENSOR.get().angle;
    }

    /**
     * Get the current velocity (translation) in m/s as a {@link java.lang.Double Double}
     *
     * @return A quantitative representation of the angle of the module
     */
    @Override
    public Double getMeasuredVelocity() {
        return STATE_SENSOR.get().speedMetersPerSecond;
    }

    /**
     * Get the current state of the module as a {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState}.
     *
     * @return A {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState} representation of the module's motion
     */
    @Override
    public SwerveModuleState getDemandModuleState() {
        return DemandState;
    }

    /**
     * Get the current position (rotation) in radians as a {@link edu.wpi.first.math.geometry.Rotation2d Rotation2d}
     *
     * @return A quantitative representation of the angle of the module
     */
    @Override
    public Rotation2d getDemandPosition() {
        return DemandState.angle;
    }

    /**
     * Get the current velocity (translation) in m/s as a {@link java.lang.Double Double}
     *
     * @return A quantitative representation of the angle of the module
     */
    @Override
    public Double getDemandVelocity() {
        return DemandState.speedMetersPerSecond;
    }

    /**
     * Get the current percent output [-1,1] of the position(Rotation of Rotational controller)
     * 
     * @return A quantitative representation of the module's rotation percent output
     */
    @Override
    public Double getRotationalOutput() {
        return ROTATION_CONTROLLER.get();
    }

    /**
     * Get the current percent output [-1,1] of the translation(Velocity of Translation controller)
     * 
     * @return A quantitative representation of the module's translation percent output
     */
    @Override
    public Double getTranslationalOutput() {
        return TRANSLATION_CONTROLLER.get();
    }

  /**
   * Shorthand for receiving the measured (encoder read, actual) motion of the module; must be implemented
   * 
   * @return The measured motion state
   */
    @Override
    public SwerveModuleState get() {
        return STATE_SENSOR.get();
    }
}