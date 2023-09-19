// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import static frc.robot.subsystems.drivebase.Constants.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.SwerveModule;
import java.util.stream.*;
import java.util.Arrays;
import java.util.List;

// -------------------------------------------------------[Drivebase Subsystem Class]-------------------------------------------------------//
//TODO Pathplanner Integeration, Limelight Measurement Pose Estimation
public final class DrivebaseSubsystem extends SubsystemBase {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static transient DrivebaseSubsystem INSTANCE;

  private static final Stream<SwerveModule> MODULES = List.of(
    Hardware.Modules.FL_Module.Components.MODULE,
    Hardware.Modules.FR_Module.Components.MODULE,
    Hardware.Modules.RL_Module.Components.MODULE,
    Hardware.Modules.RR_Module.Components.MODULE)
      .stream().parallel();

  private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
    new Translation2d(Values.ROBOT_WIDTH/2, Values.ROBOT_WIDTH/2),
    new Translation2d(Values.ROBOT_WIDTH/2, -Values.ROBOT_WIDTH/2),
    new Translation2d(-Values.ROBOT_WIDTH/2, Values.ROBOT_WIDTH/2),
    new Translation2d(-Values.ROBOT_WIDTH/2, -Values.ROBOT_WIDTH/2)
  );

  private static final SwerveDrivePoseEstimator POSE_ESTIMATOR = new SwerveDrivePoseEstimator(
    KINEMATICS,
    Hardware.GYROSCOPE.getRotation2d(),
    getModulePositions(),
    new Pose2d()
  );

  private static final Field2d FIELD = new Field2d();

  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static Boolean FieldOriented = false;  
  private static Double TimeInterval = 0.0;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  private DrivebaseSubsystem() {
    PathPlannerServer.startServer(Values.PATHPLANNER_SERVER_PORT);
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  public static synchronized void set(final Double Translation_X, final Double Translation_Y, final Double Orientation, BooleanSupplier ControlType) {
    var Demand = KINEMATICS.toSwerveModuleStates((FieldOriented)?
      (ChassisSpeeds.fromFieldRelativeSpeeds(
        Translation_X,
        Translation_Y,
        Orientation,
        Hardware.GYROSCOPE.getRotation2d())):
      (new ChassisSpeeds(
        Translation_X,
        Translation_Y,
        Orientation))
    );
    set(Arrays.asList(Demand),ControlType);
  }

  public static synchronized void set(final List<SwerveModuleState> Demand, BooleanSupplier ControlType) {
    SwerveDriveKinematics.desaturateWheelSpeeds((SwerveModuleState[])Demand.toArray(),Values.MAXIMUM_VELOCITY);
    MODULES.forEach((Module) -> {
      Module.set(() -> Demand.iterator().next(), ControlType);
    });
  }

  public static synchronized void set() {
    set(List.of(
      new SwerveModuleState((0.0),new Rotation2d(Units.degreesToRadians((315)))),
      new SwerveModuleState((0.0),new Rotation2d(Units.degreesToRadians((45)))),
      new SwerveModuleState((0.0),new Rotation2d(Units.degreesToRadians((225)))),
      new SwerveModuleState((0.0),new Rotation2d(Units.degreesToRadians((135)))))
      ,() -> false);
  }

  public static synchronized void reset(Pose2d FieldRelativePose) {
    POSE_ESTIMATOR.resetPosition(
    Hardware.GYROSCOPE.getRotation2d(),    
    getModulePositions(),
    FieldRelativePose);
  }

  public static synchronized void stop() {
    MODULES.forEach((MODULE) -> MODULE.stop());
  }

  @Override
  public void periodic() {
    Double IntervalTime;
    if(TimeInterval != (0)) {
      var RealTime = Timer.getFPGATimestamp();
      IntervalTime = RealTime - TimeInterval;
      TimeInterval = RealTime;
    } else {
      IntervalTime = (0.02);
    }
    FIELD.setRobotPose(POSE_ESTIMATOR.updateWithTime((IntervalTime), Hardware.GYROSCOPE.getRotation2d(),getModulePositions()));
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//

  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  public static SwerveModulePosition[] getModulePositions() {
    return (SwerveModulePosition[])MODULES.map(
      (Module) -> 
        new SwerveModulePosition((Values.SCALE_FACTOR*(Module.getVelocity()) * Values.DRIVETRAIN_GEAR_RATIO * Values.WHEEL_PERIMETER), Module.getPosition()))
          .collect(Collectors.toList())
          .toArray();
  }

  public static SwerveModuleState[] getModuleStates() {
    return (SwerveModuleState[])MODULES.map((Module) -> Module.getModuleState()).collect(Collectors.toList()).toArray();
  }

  public static synchronized DrivebaseSubsystem getInstance() {
    if(java.util.Objects.equals(INSTANCE, null)) {
      INSTANCE = new DrivebaseSubsystem();
    }
    return INSTANCE;
  }
}
