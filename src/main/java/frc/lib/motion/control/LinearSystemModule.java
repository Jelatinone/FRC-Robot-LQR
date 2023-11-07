// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.lib.motion.control;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.util.sendable.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
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
 * @see frc.lib.motion.control.DrivebaseModule DrivebaseModule
 */
public final class LinearSystemModule implements DrivebaseModule  {
    // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
    private static final Logger LOGGER = Logger.getInstance();    
    private final TrapezoidProfile.Constraints ROTATIONAL_MOTION_CONSTRAINTS;
    private final LinearSystemLoop<N2, N1, N1> MOTION_CONTROL_LOOP;
    private final Supplier<SwerveModuleState> STATE_SENSOR;    
    private final MotorController TRANSLATION_CONTROLLER;
    private final MotorController ROTATION_CONTROLLER;
    private final Double TRANSLATIONAL_MOTION_MAXIMUM_VELOCITY;  
    private final Double ROTATIONAL_FLYWHEEL_INITIAL_POSITION;        
    private final Integer REFERENCE_NUMBER;   
    private final Boolean IS_SIMULATED;   
    // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
    private TrapezoidProfile.State TargetPositionStateReference = new TrapezoidProfile.State();
    private RepeatCommand TargetStateCommand = new RepeatCommand(new InstantCommand());
    private SwerveModuleState DemandState = new SwerveModuleState();
    private FlywheelSim TRANSLATIONAL_CONTROLLER_WHEEL = (null);
    private FlywheelSim ROTATIONAL_CONTROLLER_WHEEL = (null);        
    private SimDouble MEASURED_POSITION_AZIMUTH = (null);    
    private SimDouble MEASURED_VELOCITY_LINEAR = (null);    
    private SimDouble DEMAND_POSITION_AZIMUTH = (null);   
    private SimDouble OUTPUT_VELOCITY_AZIMUTH = (null);
    private SimDouble DEMAND_VELOCITY_LINEAR = (null);    
    private SimDouble OUTPUT_VELOCITY_LINEAR = (null);    
    private SimDevice SIMULATED_MODULE = (null);
    private Double RotationalFlywheelPosition = (0.0);    
    private Double TimeReferencePosition = (0.0);
    private Double TimeReferenceVelocity = (0.0);
    private static Integer INSTANCE_COUNT = (0);    
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
    public LinearSystemModule(final MotorController TranslationController, final MotorController RotationController, final Supplier<SwerveModuleState> StateSensor,
                              final Double MaximumTranslationVelocity, final TrapezoidProfile.Constraints RotationMotionConstraints, final LinearSystemLoop<N2, N1, N1> MotionControlLoop) {
        TRANSLATIONAL_MOTION_MAXIMUM_VELOCITY = MaximumTranslationVelocity;
        ROTATIONAL_MOTION_CONSTRAINTS = RotationMotionConstraints;
        TRANSLATION_CONTROLLER = TranslationController;
        ROTATION_CONTROLLER = RotationController;
        MOTION_CONTROL_LOOP = MotionControlLoop;
        ROTATIONAL_FLYWHEEL_INITIAL_POSITION = (null);
        STATE_SENSOR = StateSensor;
        REFERENCE_NUMBER = INSTANCE_COUNT;
        INSTANCE_COUNT++;
        SendableRegistry.addLW((this), ("LQR-Module"), REFERENCE_NUMBER);
        IS_SIMULATED = (false);   
        stop();
    }

    /**
     * Constructor.
     *
     * @param TranslationFlywheel        The flywheel(s) responsible for controlling linear translation (velocity); queried with {@link #setVelocity(Supplier, Supplier)}
     * @param RotationFlywheel           The flywheel(s) responsible for controlling azimuth rotation (position); queried with {@link #setPosition(Supplier)}
     * @param StateSensor                The sum all collected sensor(s) data into a single {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleState} supplier
     * @param MaximumTranslationVelocity The constraints placed on the Translation Controller which determine maximum velocity output
     * @param RotationMotionConstraints  The constraint placed on the RotationController which determine maximum velocity output, and maximum change in velocity in an instant time
     * @param MotionControlLoop          The control loop responsible for controller azimuth output
     */
    public LinearSystemModule(final FlywheelSim TranslationFlywheel, final FlywheelSim RotationFlywheel, final Supplier<SwerveModuleState> StateSensor,
                              final Double MaximumTranslationVelocity, final TrapezoidProfile.Constraints RotationMotionConstraints, final LinearSystemLoop<N2, N1, N1> MotionControlLoop,
                               final Double RotationInitialPosition) {
        TRANSLATIONAL_MOTION_MAXIMUM_VELOCITY = MaximumTranslationVelocity;
        ROTATIONAL_MOTION_CONSTRAINTS = RotationMotionConstraints;
        TRANSLATION_CONTROLLER = (null);
        ROTATION_CONTROLLER = (null);
        MOTION_CONTROL_LOOP = MotionControlLoop;
        STATE_SENSOR = StateSensor;
        REFERENCE_NUMBER = INSTANCE_COUNT;
        TRANSLATIONAL_CONTROLLER_WHEEL = TranslationFlywheel;
        ROTATIONAL_CONTROLLER_WHEEL = RotationFlywheel;
        ROTATIONAL_FLYWHEEL_INITIAL_POSITION = RotationInitialPosition;
        INSTANCE_COUNT++;
        IS_SIMULATED = (true);
        SendableRegistry.addLW((this), ("LQR-Module"), REFERENCE_NUMBER);
        SIMULATED_MODULE = SimDevice.create(("LQR-Module"), REFERENCE_NUMBER);
        DEMAND_POSITION_AZIMUTH = SIMULATED_MODULE.createDouble(("DEMAND-POSITION-AZIMUTH"),SimDevice.Direction.kOutput, (DemandState.angle.getDegrees()));
        OUTPUT_VELOCITY_AZIMUTH = SIMULATED_MODULE.createDouble(("OUTPUT-VELOCITY-AZIMUTH"), SimDevice.Direction.kOutput, (0.0));
        MEASURED_POSITION_AZIMUTH =  SIMULATED_MODULE.createDouble(("MEASURED-POSITION-AZIMUTH"), SimDevice.Direction.kOutput, (STATE_SENSOR.get().angle.getDegrees()));
        DEMAND_VELOCITY_LINEAR = SIMULATED_MODULE.createDouble(("DEMAND-VELOCITY-LINEAR"), SimDevice.Direction.kOutput,(DemandState.speedMetersPerSecond));
        OUTPUT_VELOCITY_LINEAR = SIMULATED_MODULE.createDouble(("OUTPUT-VELOCITY-LINEAR"), SimDevice.Direction.kOutput, (0.0));
        MEASURED_VELOCITY_LINEAR  = SIMULATED_MODULE.createDouble(("MEASURED-VELOCITY-LINEAR"), SimDevice.Direction.kOutput, (StateSensor.get().speedMetersPerSecond));
        stop();
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
        TranslationDemand = (Objects.isNull(TranslationDemand)) ? (0.0): (TranslationDemand);
        double DiscretizationTimestep;                                                                                                                   
        if (TimeReferenceVelocity != (0)) {                                                                                                                                   
            var MeasuredTime = Timer.getFPGATimestamp();                                                                                                                         
            DiscretizationTimestep = MeasuredTime - TimeReferenceVelocity;                                                                                                              
            TimeReferenceVelocity = MeasuredTime;                                                                                                                                        
        } else {                                                                                                                                                                 
            DiscretizationTimestep = (0.02);                                                                                                                                     
        }   
        if (!Double.isNaN(TranslationDemand) && Math.abs(TranslationDemand - getTranslationalOutput()) > (2e-2)) {
            if (ControlType.get()) {
                var TranslationalControllerOutput = (TranslationDemand / (TRANSLATIONAL_MOTION_MAXIMUM_VELOCITY));
                var TranslationalControllerVoltage = TranslationalControllerOutput * RobotController.getBatteryVoltage();
                if(IS_SIMULATED) {
                    DEMAND_VELOCITY_LINEAR.set(TranslationDemand);
                    TRANSLATIONAL_CONTROLLER_WHEEL.update(DiscretizationTimestep);
                    TRANSLATIONAL_CONTROLLER_WHEEL.setInputVoltage(TranslationalControllerVoltage * (100));
                    OUTPUT_VELOCITY_LINEAR.set(getTranslationalOutput());
                } else {
                    TRANSLATION_CONTROLLER.setVoltage(TranslationalControllerVoltage);                    
                }
            } else {
                var TranslationalControllerVoltage = (TranslationDemand * RobotController.getBatteryVoltage());
                if(IS_SIMULATED) {
                    TRANSLATIONAL_CONTROLLER_WHEEL.update(DiscretizationTimestep);
                    TRANSLATIONAL_CONTROLLER_WHEEL.setInputVoltage(TranslationalControllerVoltage * (100));
                    OUTPUT_VELOCITY_LINEAR.set(getTranslationalOutput());
                } else {
                    TRANSLATION_CONTROLLER.setVoltage(TranslationalControllerVoltage);                    
                }

            }
        }
    }

    /**
     * Set the rotation (position) controller to meet a specified demand using a control type
     *
     * @param Demand The specified demand as a rotation in two-dimensional space as a {@link edu.wpi.first.math.geometry.Rotation2d Rotation2d}
     */
    @Override                                                                                                                                                                        // TODO: Read Documentation Below
    public synchronized void setPosition(final Supplier<Rotation2d> Demand) {
        var RotationDemand = Demand.get();                                                                                                                                           // Obtain the actual rotation from the supplier, this is a supplier for parity with #setVelocity(Supplier,Supplier)
        if (RotationDemand != null & !Double.isNaN(Objects.requireNonNull(RotationDemand).getRadians())) {                                                                           // Check for null or non-existent values, discard
            double DiscretizationTimestep;                                                                                                                                           // Calculate the discretization Timestep
            if (TimeReferencePosition != (0)) {                                                                                                                                      // ...
                var MeasuredTime = Timer.getFPGATimestamp();                                                                                                                         // ...
                DiscretizationTimestep = MeasuredTime - TimeReferencePosition;                                                                                                       // ...
                TimeReferencePosition = MeasuredTime;                                                                                                                                // ...
            } else {                                                                                                                                                                 // ...
                DiscretizationTimestep = (0.02);                                                                                                                                     // Set discretization timestep to twenty milliseconds by default
            }                                                                                                                                                                        // ...
            var TargetPositionState = new TrapezoidProfile.State(RotationDemand.getRadians(), (0.0));                                                                                // Construct a target position, with a position of the demand and no velocity.
            TargetPositionStateReference = new TrapezoidProfile(ROTATIONAL_MOTION_CONSTRAINTS, TargetPositionState, TargetPositionStateReference).calculate(DiscretizationTimestep); // Calculate for the acual position and velocity
            MOTION_CONTROL_LOOP.setNextR(TargetPositionStateReference.position, TargetPositionStateReference.velocity);                                                              // Given our new reference, set the Linear System's reference to be the calculated reference
            MOTION_CONTROL_LOOP.correct(VecBuilder.fill((IS_SIMULATED)? (RotationalFlywheelPosition): (STATE_SENSOR.get().angle.getRadians())));                                     // Correct the filter's state vector estimate with real encoder data
            MOTION_CONTROL_LOOP.predict(DiscretizationTimestep);                                                                                                                     // Update our Optimizer (LQR) to generate control voltage to predict the next state without filter
            var RotationalControllerVoltage = MOTION_CONTROL_LOOP.getU((0));                                                                                                         // Retrieve the calculated voltage in row 0 of the output matrix U                                                                          
            if(IS_SIMULATED) {                                                                                                                                                       // ...       
                ROTATIONAL_CONTROLLER_WHEEL.update(DiscretizationTimestep);                                                                                                          // Update the flywheel with respect to time
                ROTATIONAL_CONTROLLER_WHEEL.setInputVoltage(RotationalControllerVoltage * (100));                                                                                    // Set flywheel controller input voltage   
                DEMAND_POSITION_AZIMUTH.set(RotationDemand.getDegrees());                                                                                                            // ...       
                RotationalFlywheelPosition += ROTATIONAL_CONTROLLER_WHEEL.getAngularVelocityRadPerSec() * (DiscretizationTimestep);                                                  // Integrate flywheel velocity into position
                OUTPUT_VELOCITY_AZIMUTH.set(getRotationalOutput());                                                                                                                  // ...       
            } else {                                                                                                                                                                 // ...       
                ROTATION_CONTROLLER.setVoltage(RotationalControllerVoltage);                                                                                                         // Set motor controller input voltage  
            }                                                                                                                                                                        // ...       
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
            setPosition(() -> OptimizedDemand.get().angle)),
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
        var EncoderPositionRadiansPerSecond = State.angle.getRadians() / (TimeReferencePosition - Timer.getFPGATimestamp());
        MOTION_CONTROL_LOOP.reset(VecBuilder.fill(EncoderPositionRadian, EncoderPositionRadiansPerSecond));
        TargetPositionStateReference = new TrapezoidProfile.State(EncoderPositionRadian, EncoderPositionRadiansPerSecond);

    }

    /**
     * Close the held resources within the module, module is no longer usable after this operation.
     */
    @Override
    public synchronized void close() {
        stop();        
        SendableRegistry.remove(this);
        if(IS_SIMULATED)  {
            SIMULATED_MODULE.close();
        }
        TargetStateCommand.cancel();
    }

    /**
     * Stop all the controllers within the module immediately, and cancel all subsequent motions. make another call to {@link #set(SwerveModuleState, Supplier)} to call again.
     */
    @Override
    public void stop() {
        TargetPositionStateReference = new TrapezoidProfile.State((0.0), (0.0));        
        DemandState = new SwerveModuleState((0.0),new Rotation2d((0.0)));
        TargetStateCommand.cancel(); 
        if(IS_SIMULATED) {
            double DiscretizationTimestep;                                                                                                                                     
            if (TimeReferencePosition != (0)) {                                                                                                                                              
                var MeasuredTime = Timer.getFPGATimestamp();                                                                                                                         
                DiscretizationTimestep = MeasuredTime - TimeReferencePosition;                                                                                                               
                TimeReferencePosition = MeasuredTime;                                                                                                                                        
            } else {                                                                                                                                                                 
                DiscretizationTimestep = (0.02);                                                                                                                                     
            }       
            TRANSLATIONAL_CONTROLLER_WHEEL.update(DiscretizationTimestep);                     
            TRANSLATIONAL_CONTROLLER_WHEEL.setState(VecBuilder.fill((0)));
            ROTATIONAL_CONTROLLER_WHEEL.update(DiscretizationTimestep);           
            ROTATIONAL_CONTROLLER_WHEEL.setState(VecBuilder.fill((0)));
            OUTPUT_VELOCITY_AZIMUTH.set((0));
            OUTPUT_VELOCITY_LINEAR.set((0));
            DEMAND_POSITION_AZIMUTH.set((0));
            DEMAND_VELOCITY_LINEAR.set((0));
            RotationalFlywheelPosition = (ROTATIONAL_FLYWHEEL_INITIAL_POSITION);
            var MeasuredState = STATE_SENSOR.get();              
            MEASURED_POSITION_AZIMUTH.set(MeasuredState.angle.getDegrees());
            MEASURED_VELOCITY_LINEAR.set(MeasuredState.speedMetersPerSecond);              
        } else {
            TRANSLATION_CONTROLLER.setVoltage((0.0));
            ROTATION_CONTROLLER.setVoltage((0.0));            
        }
        post();        
    }

    /**
     * Consume a SwerveModuleState as shorthand for calling {@link #set(SwerveModuleState, Supplier), with false as a ControlType}
     *
     * @param Demand The desired state for the module to achieve
     */
    @Override
    public void accept(final SwerveModuleState Demand) {
        set(Demand, () -> (false));
    }

    @Override
    public void initSendable(SendableBuilder Builder) {
        Builder.setSmartDashboardType(("LinearSystemModule"));
        Builder.addDoubleProperty(("Translation Velocity"), this::getMeasuredVelocity, (Demand) -> setVelocity(() -> Demand, () -> false));
        Builder.addDoubleProperty(("Translation Output"), this::getTranslationalOutput, (null));
        Builder.addDoubleProperty(("Rotation Position"), () -> getMeasuredPosition().getRadians(), (Demand) -> setPosition(() -> new Rotation2d(Demand)));
        Builder.addDoubleProperty(("Rotation Velocity"), () -> TargetPositionStateReference.velocity, (null));
        Builder.addDoubleProperty(("Rotation Output"), this::getRotationalOutput, (null));
    }

    /**
     * Post measurement, output, and demand data to shuffleboard static  instance
     */
    @Override
    public void post() {
        var Prefix = ("Module [") + REFERENCE_NUMBER + ("]/");
        SmartDashboard.putNumber(Prefix + "Demand Azimuth Rotation", DemandState.angle.getDegrees());
        SmartDashboard.putNumber(Prefix + "Demand Azimuth Velocity",TargetPositionStateReference.velocity);        
        SmartDashboard.putNumber(Prefix + "Demand Linear Velocity", DemandState.speedMetersPerSecond);
        SmartDashboard.putNumber(Prefix + "Measured Azimuth Rotation", getMeasuredPosition().getDegrees() % (360));
        SmartDashboard.putNumber(Prefix + "Measured Linear Velocity", getMeasuredVelocity());
        SmartDashboard.putNumber(Prefix + "Output Azimuth Rotation", getRotationalOutput());
        SmartDashboard.putNumber(Prefix + "Output Linear Velocity",getTranslationalOutput());
        LOGGER.recordOutput((Prefix + "DemandAzimuthPosition"), DemandState.angle.getDegrees());
        LOGGER.recordOutput((Prefix + "DemandAzimuthPositionVelocity"), TargetPositionStateReference.velocity);
        LOGGER.recordOutput((Prefix + "DemandTranslationVelocity"), DemandState.speedMetersPerSecond);
        LOGGER.recordOutput((Prefix + "MeasuredAzimuthRotation"), getMeasuredPosition().getDegrees() % (360));
        LOGGER.recordOutput((Prefix + "MeasuredTranslationVelocity"), getMeasuredVelocity());
        LOGGER.recordOutput((Prefix + "OutputAzimuthPercent"), getRotationalOutput());
        LOGGER.recordOutput((Prefix + "OutputTranslationPercent"), getTranslationalOutput());
        if(IS_SIMULATED) {
            MEASURED_VELOCITY_LINEAR.set(STATE_SENSOR.get().speedMetersPerSecond);
            MEASURED_POSITION_AZIMUTH.set(STATE_SENSOR.get().angle.getDegrees());            
        }
    }

    /**
     * Configure a {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonFX WPI_TalonFX} rotational controller to swerve module specifications
     *
     * @param Controller     Controller that is to be configured
     * @param AzimuthEncoder Azimuth CANCoder instance acting as a feedback filter reference point, and an azimuth sensor position source
     * @param CurrentLimit   The current limit configuration of the motor's stator.
     * @param Deadband       Desired percent deadband of controller inputs
     * @param Inverted       If the controller's output should be mirrored
     * @return A copy of the configured controller
     */
    public static WPI_TalonFX configureController(final WPI_TalonFX Controller, final WPI_CANCoder AzimuthEncoder, final StatorCurrentLimitConfiguration CurrentLimit, final Double Deadband, final Boolean Inverted) {
        Controller.configFactoryDefault();
        Controller.clearStickyFaults();
        Controller.setInverted(Inverted);
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
     * @param Inverted     If the controller's output should be mirrored
     * @return A copy of the configured controller
     */
    public static WPI_TalonFX configureTranslationController(final WPI_TalonFX Controller, final StatorCurrentLimitConfiguration CurrentLimit, final Boolean Inverted) {
        Controller.configFactoryDefault();
        Controller.clearStickyFaults();
        Controller.setInverted(Inverted);
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
     * @param Inverted       If the encoder output should be mirrored
     * @return A copy of the configured encoder
     */
    public static WPI_CANCoder configureRotationEncoder(final WPI_CANCoder AzimuthEncoder, final Double Offset, final Boolean Inverted) {
        AzimuthEncoder.configFactoryDefault();
        AzimuthEncoder.clearStickyFaults();
        AzimuthEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        AzimuthEncoder.configSensorDirection(Inverted);        
        AzimuthEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);        
        AzimuthEncoder.configGetFeedbackTimeBase(SensorTimeBase.PerSecond.value);        
        AzimuthEncoder.configMagnetOffset(Offset);
        AzimuthEncoder.setPositionToAbsolute();     
        return AzimuthEncoder;
    }

    /**
     * Configure a {@link com.revrobotics.CANSparkMax CANSparkMax} translation or rotation controller to swerve module specifications
     *
     * @param Controller     Controller that is to be configured
     * @param AmpLimit       The current limit of the controller measured in amps
     * @param NominalVoltage The nominal voltage to compensate output of the controller to compensate voltage for
     * @param Inverted       If the controller's output should be mirrored
     * @return A copy of the configured controller
     */
    public static CANSparkMax configureController(final CANSparkMax Controller, final Integer AmpLimit, final Double NominalVoltage, final Boolean Inverted) {
        Controller.restoreFactoryDefaults();
        Controller.clearFaults();        
        Controller.setInverted(Inverted);
        Controller.setSmartCurrentLimit(AmpLimit);
        Controller.setIdleMode(IdleMode.kBrake);
        Controller.enableVoltageCompensation(NominalVoltage);
        Controller.burnFlash();
        return Controller;
    }

    /**
     * Configure a {@link com.revrobotics.RelativeEncoder RelativeEncoder} of a translation or rotation controller to swerve module specifications
     * 
     * @param Encoder                  Encoder that is to be configured
     * @param VelocityConversionFactor Factor of conversion when outputting velocity
     * @param PositionConversionFactor Factor of conversion when outputting position 
     * @return
     */
    public static RelativeEncoder configureEncoder(final RelativeEncoder Encoder, final Double VelocityConversionFactor, final Double PositionConversionFactor) {
        Encoder.setPosition((0.0));
        Encoder.setVelocityConversionFactor(VelocityConversionFactor);
        Encoder.setPositionConversionFactor(PositionConversionFactor);
        return Encoder;
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
     * Get the current angular velocity in RPM of the rotational controller or flywheel
     * 
     * @return A quantitative representation of the module's rotation 
     */
    @Override
    public Double getRotationalOutput() {
        return IS_SIMULATED? (RotationalFlywheelPosition): (ROTATION_CONTROLLER.get());
    }

    /**
     * Get the current angular velocity in rotations of the translational controller or flywheel
     * 
     * @return A quantitative representation of the module's translation
     */
    @Override
    public Double getTranslationalOutput() {
        return IS_SIMULATED? (TRANSLATIONAL_CONTROLLER_WHEEL.getAngularVelocityRPM() * (0.02)): (TRANSLATION_CONTROLLER.get());
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