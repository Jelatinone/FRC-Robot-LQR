// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.DrivebaseModule;

import java.io.Closeable;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Stream;

import static frc.robot.subsystems.drivebase.Constants.Hardware;
import static frc.robot.subsystems.drivebase.Constants.Values;

// -------------------------------------------------------[Drivebase Subsystem Class]-------------------------------------------------------//
//TODO CompletePathplanner Integration, Limelight Measurement Pose Estimation
/**
 * 
 * 
 * <h1> DrivebaseSubsystem </h1>
 * 
 * 
 * <p> {@link frc.robot.subsystems.drivebase.Constants Constants} based implementation of swerve drivebase which utilizes 
 * {@link frc.lib.DrivebaseModule modules} based on {@link edu.wpi.first.math.system.LinearSystemLoop LinearSystemLoop} to achieve more
 * accurate azimuth positioning. This subsystem is entirely static, meaning only one instance will exist and this Instance is obtained
 * by calling {@link #getInstance()} from a static context. </p>
 * 
 * @author Cody Washington (@Jelatinone)
 * 
 */
public final class DrivebaseSubsystem extends SubsystemBase implements Closeable, Consumer<SwerveModuleState[]>, Supplier<Pose2d> {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static DrivebaseSubsystem INSTANCE;

  private static final Stream<DrivebaseModule> MODULES = Stream.of(
    Hardware.Modules.FL_Module.Components.MODULE,
    Hardware.Modules.FR_Module.Components.MODULE,
    Hardware.Modules.RL_Module.Components.MODULE,
    Hardware.Modules.RR_Module.Components.MODULE).parallel();

  private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
    new Translation2d((Values.Chassis.ROBOT_WIDTH)/(2), (Values.Chassis.ROBOT_WIDTH)/(2)),
    new Translation2d((Values.Chassis.ROBOT_WIDTH)/(2), -(Values.Chassis.ROBOT_WIDTH)/(2)),
    new Translation2d(-(Values.Chassis.ROBOT_WIDTH)/(2), (Values.Chassis.ROBOT_WIDTH)/(2)),
    new Translation2d(-(Values.Chassis.ROBOT_WIDTH)/(2), -(Values.Chassis.ROBOT_WIDTH)/(2)));

  private static final SwerveDrivePoseEstimator POSE_ESTIMATOR = new SwerveDrivePoseEstimator(
    KINEMATICS,
    Hardware.GYROSCOPE.getRotation2d(),
    getModulePositions(),
    new Pose2d()
  );

  private static final Field2d FIELD = new Field2d();

  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static Boolean FieldOriented = (false);  
  private static Double TimeInterval = (0.0);
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Constructor.
   */
  private DrivebaseSubsystem() {
    PathPlannerServer.startServer(Values.Port.PATHPLANNER_SERVER_PORT);
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  /**
   * Periodically updates the robot's estimated posotion, and Update the module and drivebase telemetry for all modules within the system.
   */
  @Override
  public void periodic() {
    double IntervalTime;
    if(TimeInterval != (0)) {
      var RealTime = Timer.getFPGATimestamp();
      IntervalTime = RealTime - TimeInterval;
      TimeInterval = RealTime;
    } else {
      IntervalTime = (0.02);
    }
    FIELD.setRobotPose(POSE_ESTIMATOR.updateWithTime((IntervalTime), Hardware.GYROSCOPE.getRotation2d(),getModulePositions()));
    AtomicReference<Integer> ModuleNumber = new AtomicReference<>(1);
    MODULES.forEach(
    (Module) -> {
      Module.post(ModuleNumber.get()); 
      ModuleNumber.set(ModuleNumber.get() + (1));
    });
    SmartDashboard.putNumber("DRIVEBASE HEADING", Hardware.GYROSCOPE.getAngle());
  }

  /**
   * Consume an array of demand states for each module.
   * @param Demand - The states to demand for each module, paired in order with each module.
   */
  public void accept(final SwerveModuleState[] Demand) {
    set(Arrays.asList(Demand),() -> (false));
  }

  /**
   * Reset odometry with a new {@link edu.wpi.first.math.geometry.Pose2d Pose2d} relative to the field
   * @param FieldRelativePose - The actual position the robot should be placed at
   */
  public synchronized void reset(Pose2d FieldRelativePose) {
    POSE_ESTIMATOR.resetPosition(
    Hardware.GYROSCOPE.getRotation2d(),    
    getModulePositions(),
    FieldRelativePose);
  }

  /**
   * Ceases all drivebase motion immediately, can be called again immediately with {@link #set(List, BooleanSupplier)}
   */
  @SuppressWarnings("unused, null")
  public static synchronized void stop() {
    MODULES.forEach(DrivebaseModule::stop);
  }

  /**
   * Closes all resources held within the susbsytem, makes subsystem unusable
   */
  public void close() {
    MODULES.close();
      FIELD.close();
  }
  // --------------------------------------------------------------[Mutators]-----------------------------i9u87----------------------------------//
  /**
   * Set and  demand states based on translation and orientation imputs to generate module states 
   * @param Translation_X - Demand on the X-axis
   * @param Translation_Y - Demand on the Y-axis
   * @param Orientation - The desired rotation of the front face
   * @param ControlType - Whether to use OpenLoop control
   */
  @SuppressWarnings("unused, null")
  public static synchronized void set(final Double Translation_X, final Double Translation_Y, final Double Orientation, BooleanSupplier ControlType) {
    var Demand = (List.of(KINEMATICS.toSwerveModuleStates((FieldOriented) ?
      (ChassisSpeeds.fromFieldRelativeSpeeds(Translation_X, Translation_Y, Orientation, Hardware.GYROSCOPE.getRotation2d())) :
      (new ChassisSpeeds(Translation_X, Translation_Y, Orientation)))));
    Demand.forEach(
      (State) ->
        State.speedMetersPerSecond = (((State.speedMetersPerSecond * (60)) / Values.Chassis.WHEEL_DIAMETER) * Values.Chassis.DRIVETRAIN_GEAR_RATIO) * (Values.ComponentData.ENCODER_SENSITIVITY / (600)));
    set(Demand,ControlType);
  }

  /**
   * Set Module states given a list of swerve module states and a control type
   * @param Demand - Module state demands
   * @param ControlType - Whether to use OpenLoop control
   */
  @SuppressWarnings("unchecked, unused")
  public static synchronized void set(final List<SwerveModuleState> Demand, BooleanSupplier ControlType) {
    var Iterator = Demand.iterator();
    SwerveDriveKinematics.desaturateWheelSpeeds((SwerveModuleState[])Demand.toArray(),Values.Limit.ROBOT_MAXIMUM_VELOCITY);
    MODULES.forEach((Module) -> Module.set(() -> Iterator.next(), ControlType));
  }

  /**
   * Set module states given a list of swerve module states
   * @param Demand - Module state demands
   */
  @SuppressWarnings("unchecked, unused")
  public static synchronized void set(final List<SwerveModuleState> Demand) {
    set(Demand,() ->  false);
  }
  
  /**
   * Set module's into a default, "x-locking" position that prevents drivebase movement when idle
   */
  @SuppressWarnings("unused, null")
  public static synchronized void set() {
    set(List.of(
      new SwerveModuleState((0.0),new Rotation2d(Units.degreesToRadians((315)))),
      new SwerveModuleState((0.0),new Rotation2d(Units.degreesToRadians((45)))),
      new SwerveModuleState((0.0),new Rotation2d(Units.degreesToRadians((225)))),
      new SwerveModuleState((0.0),new Rotation2d(Units.degreesToRadians((135))))),
      () -> (false));
  }

  /**
   * Set whether the robot is using field oriented control
   * @param isFieldOriented - Whether or not field oriented control is enabled
   */
  @SuppressWarnings("unused, null")
  public static void setFieldOriented(final Boolean isFieldOriented) {
    FieldOriented = isFieldOriented;
  }

  /**
   * Toggles (switches) the current state of Field Orientation
   */
  @SuppressWarnings("unused, null")
  public static void toggleFieldOriented() {
    FieldOriented = !FieldOriented;
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Obtain an autonomous command which follows a trajectory
   * @param Demand - The demand pathplanner trajectory
   * @param EventMap - An event map to pair to execute at given markers along the path
   * @param IsBlue - Whether or not the robot is on the blue alliance or not, flips the side of the robot
   * @return Autonomous trajectory following command
   */
  @SuppressWarnings("unused, null")
  public static synchronized Command getAutonomousCommand(final PathPlannerTrajectory Demand, final HashMap<String,Command> EventMap, final Boolean IsBlue) {
    MODULES.forEach(DrivebaseModule::reset);
    return new SwerveAutoBuilder(
      INSTANCE,
      INSTANCE::reset,
      KINEMATICS,
      new PIDConstants(Constants.Values.PathPlanner.TRANSLATION_KP, Constants.Values.PathPlanner.TRANSLATION_KI, Constants.Values.PathPlanner.TRANSLATION_KD),
      new PIDConstants(Constants.Values.PathPlanner.ROTATION_KP, Constants.Values.PathPlanner.ROTATION_KI, Constants.Values.PathPlanner.ROTATION_KD), 
      INSTANCE, 
      (EventMap),
      (IsBlue),
      (INSTANCE)
    ).fullAuto(Demand);
  }

  /**
   * Get an array representing the measured {@link edu.wpi.first.math.kinematics.SwerveModulePosition position} of each module in order
   * @return An array of {@link edu.wpi.first.math.kinematics.SwerveModulePosition SwerveModulePosition}
   */
  public static SwerveModulePosition[] getModulePositions() {
    return (SwerveModulePosition[]) MODULES.map(
      (Module) -> new SwerveModulePosition(
        Values.ComponentData.SCALE_FACTOR * (Module.getVelocity()) * Values.Chassis.DRIVETRAIN_GEAR_RATIO * Values.Chassis.WHEEL_PERIMETER,
        Module.getPosition())
    ).toArray();
  }

  /**
   * Get an array representing the measured {@link edu.wpi.first.math.kinematics.SwerveModuleState state} of each module in order
   * @return An array of {@link edu.wpi.first.math.kinematics.SwerveModuleState SwerveModuleStates}
   */
  @SuppressWarnings("unused, null")
  public static SwerveModuleState[] getModuleStates() {
    return (SwerveModuleState[]) MODULES.map(DrivebaseModule::getModuleState).toArray();
  }

  /**
   * Get the static instance of the Drivebase, if not already created this will also create a new instance.
   * @return Static {@link frc.robot.subsystems.drivebase.DrivebaseSubsystem DrivebaseSubsystem} instance
   */
  public static synchronized DrivebaseSubsystem getInstance() {
    if(java.util.Objects.equals(INSTANCE, (null))) {
      INSTANCE = new DrivebaseSubsystem();
    }
    return INSTANCE;
  }

  /**
   * Get the {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator pose estimator}'s estimated {@link edu.wpi.first.math.geometry.Pose2d Pose2d} based on measurement data
   * @return estimated {@link edu.wpi.first.math.geometry.Pose2d Pose2d} based on measurement data
   */
  public Pose2d get() {
    return POSE_ESTIMATOR.getEstimatedPosition();
  }
}