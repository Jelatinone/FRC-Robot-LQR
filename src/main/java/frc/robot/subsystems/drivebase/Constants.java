// ------------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot.subsystems.drivebase;
// -----------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Nat;
import frc.lib.motion.legacycontrol.LinearSystemModule;
import frc.robot.subsystems.drivebase.Constants.Values.Chassis;

import java.util.function.Supplier;
// --------------------------------------------------------------[Constants Class]------------------------------------------------------------//
public final class Constants {
    
  public static final class Values {

    public static final class ComponentData {
      public static final StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = new StatorCurrentLimitConfiguration((true), (60), (60), (0));      
      public static final StatorCurrentLimitConfiguration AZIMUTH_CURRENT_LIMIT = new StatorCurrentLimitConfiguration((true), (20), (20), (0));      
      public static final Double STANDARD_ENCODER_SENSITIVITY = (2048.0);   
      public static final Double AZIMUTH_DEADBAND = (0.06);       
      public static final Double SCALE_FACTOR = (1.0/50.0);   
    }

    public static final class PathPlanner {
      public static final Double TRANSLATION_KP = (Chassis.IS_NEO_SWERVE)? (11.0): (0.2); //FALCON SWERVE: {15, 3.25, 2.75, 2.5, 2.1, 2, 0.018, 0.03, 0.004, 0.001}
      public static final Double TRANSLATION_KI = (Chassis.IS_NEO_SWERVE)? (0.0): (0.0); 
      public static final Double TRANSLATION_KD = (Chassis.IS_NEO_SWERVE)? (0.0): (0.0);
      public static final Double ROTATION_KP = (Chassis.IS_NEO_SWERVE)? (0.0): (0.008); //FALCON SWERVE: {6.25, 12.5, 15}
      public static final Double ROTATION_KI = (Chassis.IS_NEO_SWERVE)? (0.0): (0.0);
      public static final Double ROTATION_KD = (Chassis.IS_NEO_SWERVE)? (0.0): (0.0);
    }

    public static final class Chassis {
      public static final Boolean IS_SIMULATED = (RobotBase.isSimulation());    
      public static final Boolean IS_NEO_SWERVE = (true);           
      public static final Double WHEEL_DIAMETER = (IS_NEO_SWERVE)? (Units.inchesToMeters((4))): (0.1016);          
      public static final Double WHEEL_PERIMETER = (WHEEL_DIAMETER) * Math.PI;        
      public static final Double DRIVETRAIN_LINEAR_GEAR_RATIO = ((6.75) / 1.0);
      public static final Double DRIVETRAIN_AZIMUTH_GEAR_RATIO = (((150.0)/(7.0)) / 1.0);
      public static final Double DRIVETRAIN_WIDTH = (IS_NEO_SWERVE)? (Units.inchesToMeters((21.75))): (0.6858);   
      public static final double ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER * Math.PI) / DRIVETRAIN_LINEAR_GEAR_RATIO;
      public static final Double LINEAR_ENCODER_VELOCITY_FACTOR = ENCODER_POSITION_FACTOR / (60.0);
      public static final Double AZIMUTH_ENCODER_POSITION_FACTOR = (360.0) / DRIVETRAIN_AZIMUTH_GEAR_RATIO;       
    }

    public static final class Maximum {   
      public static final Double ROBOT_MAXIMUM_X_TRANSLATION_OUTPUT = (5.4);   
      public static final Double ROBOT_MAXIMUM_Y_TRANSLATION_OUTPUT = (5.4);   
      public static final Double ROBOT_MAXIMUM_ORIENTATION_OUTPUT = (5.4);   
    }

    public static final class Ports {
      public static final Integer PATHPLANNER_SERVER_PORT = (6969);
      public static final Integer GYROSCOPE_ID = (Chassis.IS_NEO_SWERVE)? (1): (3);   
    }
  }

  public static final class Hardware {
    public static final WPI_Pigeon2 GYROSCOPE = new WPI_Pigeon2(Values.Ports.GYROSCOPE_ID);

    public static final class Modules {
        public static final class FL_Module {

          public static final class Values {
            public static final CANSparkMaxLowLevel.MotorType AZIMUTH_MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;  
            public static final String AZIMUTH_NAME = ("AZIMUTH [FL]");            
            public static final Double AZIMUTH_MAXIMUM_ACCELERATION = (Math.PI*2);    
            public static final Double AZIMUTH_MAXIMUM_VOLTAGE = (12.0);            
            public static final Double AZIMUTH_MAXIMUM_VELOCITY = (5.4);
            public static final Double AZIMUTH_NOMINAL_VOLTAGE = (12.0);
            public static final Integer AZIMUTH_MAXIMUM_AMPERAGE = (20);   
            public static final Integer AZIMUTH_ID = (Chassis.IS_NEO_SWERVE)? (21): (21);    
            public static final Boolean AZIMUTH_INVERTED = (true);

            public static final CANSparkMaxLowLevel.MotorType LINEAR_MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless; 
            public static final String LINEAR_NAME = ("LINEAR [FL]");             
            public static final Double LINEAR_ENCODER_SENSITIVITY = (2048.0);                          
            public static final Double LINEAR_MAXIMUM_VELOCITY = (5.4);
            public static final Double LINEAR_NOMINAL_VOLTAGE = (12.0);      
            public static final Integer LINEAR_MAXIMUM_AMPERAGE = (80);
            public static final Integer LINEAR_ID = (Chassis.IS_NEO_SWERVE)? (11): (11);   
            public static final Boolean LINEAR_INVERTED = (false);

            public static final String PRIMARY_ENCODER_NAME = ("ENCODER [FL]");
            public static final Double PRIMARY_ENCODER_OFFSET = (Chassis.IS_NEO_SWERVE)? (40.166016 + 90): (-313.006);  
            public static final Double PRIMARY_ENCODER_SENSITIVITY = (4096.0);    
            public static final Boolean PRIMARY_ENCODER_INVERTED = (true);
            public static final Integer PRIMARY_ENCODER_ID = (Chassis.IS_NEO_SWERVE)? (31): (4);            

            public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
            public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
            public static final Double MODEL_ACCELERATION_GAIN = (0.27); 
            public static final Double MODEL_VELOCITY_GAIN = (0.01);

            public static final Double DISCRETIZATION_TIMESTEP = (1/50.0);            
          }

          public static final class Components {
            public static final WPI_CANCoder PRIMARY_ENCODER = LinearSystemModule.configureRotationEncoder((new WPI_CANCoder(Values.PRIMARY_ENCODER_ID)), Values.PRIMARY_ENCODER_OFFSET, Values.PRIMARY_ENCODER_INVERTED);
            public static final MotorController LINEAR_CONTROLLER = (Constants.Values.Chassis.IS_NEO_SWERVE)?
              (LinearSystemModule.configureController(new CANSparkMax(
                      Values.LINEAR_ID,
                      Values.LINEAR_MOTOR_TYPE),
                      Values.LINEAR_MAXIMUM_AMPERAGE,
                      Values.LINEAR_NOMINAL_VOLTAGE,
                      Values.LINEAR_INVERTED)):
              (LinearSystemModule.configureTranslationController(new WPI_TalonFX(
                      Values.LINEAR_ID, Values.LINEAR_NAME),
                      Constants.Values.ComponentData.DRIVE_CURRENT_LIMIT,
                      Values.LINEAR_INVERTED));
            public static final FlywheelSim LINEAR_FLYWHEEL = (Chassis.IS_SIMULATED)?
            (new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                (Chassis.IS_NEO_SWERVE)?
                  (DCMotor.getNEO((1))):
                  (DCMotor.getFalcon500((1))),
                 (505.597120488),                            
                (Chassis.DRIVETRAIN_LINEAR_GEAR_RATIO)),
              (Chassis.IS_NEO_SWERVE)?
                (DCMotor.getNEO((1))):
                (DCMotor.getFalcon500((1))),
              (Chassis.DRIVETRAIN_LINEAR_GEAR_RATIO), 
              (VecBuilder.fill((0.025))))):       
            (null);
            public static final RelativeEncoder LINEAR_ENCODER = (Chassis.IS_NEO_SWERVE)? (LinearSystemModule.configureEncoder(((CANSparkMax)LINEAR_CONTROLLER).getEncoder(),(Chassis.LINEAR_ENCODER_VELOCITY_FACTOR),(Chassis.AZIMUTH_ENCODER_POSITION_FACTOR))): (null);
            public static final MotorController AZIMUTH_CONTROLLER = (Constants.Values.Chassis.IS_NEO_SWERVE)?
              (LinearSystemModule.configureController(new CANSparkMax(
                      Values.AZIMUTH_ID,
                      Values.AZIMUTH_MOTOR_TYPE),
                      Values.AZIMUTH_MAXIMUM_AMPERAGE,
                      Values.AZIMUTH_NOMINAL_VOLTAGE,
                      Values.AZIMUTH_INVERTED)):
              (LinearSystemModule.configureController(new WPI_TalonFX(
                      Values.AZIMUTH_ID, Values.AZIMUTH_NAME),
                      PRIMARY_ENCODER,
                      Constants.Values.ComponentData.AZIMUTH_CURRENT_LIMIT,
                      Constants.Values.ComponentData.AZIMUTH_DEADBAND, 
                      Values.AZIMUTH_INVERTED));

            public static final FlywheelSim AZIMUTH_FLYWHEEL = (Chassis.IS_SIMULATED)?
            (new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                (Chassis.IS_NEO_SWERVE)?
                  (DCMotor.getNEO((1))):
                  (DCMotor.getFalcon500((1))),
                (249.71004822),                     
                (Chassis.DRIVETRAIN_AZIMUTH_GEAR_RATIO)),
              (Chassis.IS_NEO_SWERVE)?
                (DCMotor.getNEO((1))):
                (DCMotor.getFalcon500((1))),
              (Chassis.DRIVETRAIN_AZIMUTH_GEAR_RATIO), 
              (VecBuilder.fill((0.025))))):          
            (null);
            public static final RelativeEncoder AZIMUTH_ENCODER = (Chassis.IS_NEO_SWERVE)? (LinearSystemModule.configureEncoder(((CANSparkMax)LINEAR_CONTROLLER).getEncoder(),(Chassis.LINEAR_ENCODER_VELOCITY_FACTOR),(Chassis.AZIMUTH_ENCODER_POSITION_FACTOR))): (null);
            public static final Supplier<SwerveModuleState> STATE_SENSOR = (Constants.Values.Chassis.IS_NEO_SWERVE)?
              (() -> new SwerveModuleState(
                  (LINEAR_ENCODER.getVelocity() / (1000.0)),
                  Rotation2d.fromDegrees(PRIMARY_ENCODER.getPosition() % (360))
              )):
              (() -> {
                  assert LINEAR_CONTROLLER instanceof WPI_TalonFX;
                  return new SwerveModuleState(
                    (2.0)*(((((WPI_TalonFX)LINEAR_CONTROLLER).getSelectedSensorVelocity() / Values.LINEAR_ENCODER_SENSITIVITY) * (10)) / Constants.Values.Chassis.DRIVETRAIN_AZIMUTH_GEAR_RATIO) * Math.PI * Constants.Values.Chassis.WHEEL_DIAMETER,
                    Rotation2d.fromDegrees(PRIMARY_ENCODER.getPosition() % (360)));
              });
              public static final LinearSystemModule MODULE = (Constants.Values.Chassis.IS_SIMULATED)? 
              (new LinearSystemModule(
                LINEAR_FLYWHEEL,
                AZIMUTH_FLYWHEEL,
                STATE_SENSOR,
                Values.LINEAR_MAXIMUM_VELOCITY,
                Control.CONSTRAINTS,
                Control.CONTROL_LOOP, 
                Units.degreesToRadians(Values.PRIMARY_ENCODER_OFFSET))):
              (new LinearSystemModule(
                (LINEAR_CONTROLLER),
                (AZIMUTH_CONTROLLER),
                STATE_SENSOR,
                Values.LINEAR_MAXIMUM_VELOCITY,
                Control.CONSTRAINTS,
                Control.CONTROL_LOOP));
          }
                                                                                                                                                                                                           // TODO: Read Documentation Below
          public static final class Control {
            public static final Double REGULATOR_QELMS_DEGREES_ACCURACY_POSITION_WEIGHT = (0.01);                                                                                                          // Weight of control percision position (Qelms)
            public static final Double REGULATOR_QELMS_ROTATIONS_ACCURACY_VELOCITY_WEIGHT = (0.01);                                                                                                        // Weight of control percision velocity (Qelms)
            public static final Double REGULATOR_RELMS_CONTROLLER_EFFORT_WEIGHT = (0.01);                                                                                                                  // Weight of control effort (Relms)
            public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.AZIMUTH_MAXIMUM_VELOCITY, Values.AZIMUTH_MAXIMUM_ACCELERATION);                         // Constraints for maximum velocity and acceleration of azimuth
            public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.MODEL_VELOCITY_GAIN, Values.MODEL_ACCELERATION_GAIN);                                          // Single Input, Single Output State-Space System
            public static final LinearSystemLoop<N2,N1,N1> CONTROL_LOOP = new LinearSystemLoop<>(                                                                                                          // Control Optimizer Loop
              PLANT,                                                                                                                                                                                       // + State-Space Model of our system being controlled
              new LinearQuadraticRegulator<>(                                                                                                                                                              // + State-Space controller (Regulator) of our system
                PLANT,                                                                                                                                                                                     //   - State-Space Model of our system being controlled
                VecBuilder.fill(Units.degreesToRadians(REGULATOR_QELMS_DEGREES_ACCURACY_POSITION_WEIGHT), Units.rotationsPerMinuteToRadiansPerSecond(REGULATOR_QELMS_ROTATIONS_ACCURACY_VELOCITY_WEIGHT)), //   - Controller percision weight matrix (How much to penalize large error of the motor to a reference), more elements penalize less (Qelms)
                VecBuilder.fill(Values.AZIMUTH_MAXIMUM_VOLTAGE),                                                                                                                                           //   - Controller effort weight matrix (How much to penalize large usage of the motor), more elements penalize less (Relms)
                Values.DISCRETIZATION_TIMESTEP),                                                                                                                                                           //   - Nominal timestep
              new KalmanFilter<>(                                                                                                                                                                          // + State-Space observer (Kalman Filter)
                  Nat.N2(),                                                                                                                                                                                //   - Measured States of the system (Controller velocity, Controller position)
                  Nat.N1(),                                                                                                                                                                                //   - Outputs of our system (Controller Voltage)
                  PLANT,                                                                                                                                                                                   //   - State-Space Model of our system being controlled
                  VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES), Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)),                         //   - Measured State standard deviations, How accurate we expect our system model to be
                  VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(1 / Values.PRIMARY_ENCODER_SENSITIVITY)),                                                                                     //   - Output standard deviations, How accurate we expect our encoder data to be
                  Values.DISCRETIZATION_TIMESTEP),                                                                                                                                                         //   - Nominal timestep
              Values.AZIMUTH_MAXIMUM_VOLTAGE,                                                                                                                                                              // + Maximum applicable voltage to the azimuth
              Values.DISCRETIZATION_TIMESTEP);                                                                                                                                                             // + Nominal timestep
          }
        }

        public static final class FR_Module {

          public static final class Values {
            public static final CANSparkMaxLowLevel.MotorType AZIMUTH_MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless; 
            public static final String AZIMUTH_NAME = ("AZIMUTH [FR]");             
            public static final Double AZIMUTH_MAXIMUM_ACCELERATION = (Math.PI*2);    
            public static final Double AZIMUTH_MAXIMUM_VOLTAGE = (12.0);            
            public static final Double AZIMUTH_MAXIMUM_VELOCITY = (5.4);
            public static final Double AZIMUTH_NOMINAL_VOLTAGE = (12.0);
            public static final Integer AZIMUTH_MAXIMUM_AMPERAGE = (20);   
            public static final Integer AZIMUTH_ID = (Chassis.IS_NEO_SWERVE)? (22): (22);    
            public static final Boolean AZIMUTH_INVERTED = (true);

            public static final CANSparkMaxLowLevel.MotorType LINEAR_MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;  
            public static final String LINEAR_NAME = ("LINEAR [FR]");            
            public static final Double LINEAR_ENCODER_SENSITIVITY = (2048.0);                          
            public static final Double LINEAR_MAXIMUM_VELOCITY = (5.4);
            public static final Double LINEAR_NOMINAL_VOLTAGE = (12.0);      
            public static final Integer LINEAR_MAXIMUM_AMPERAGE = (80);
            public static final Integer LINEAR_ID = (Chassis.IS_NEO_SWERVE)? (12): (12);   
            public static final Boolean LINEAR_INVERTED = (false);

            public static final String PRIMARY_ENCODER_NAME = ("ENCODER [FR]");
            public static final Double PRIMARY_ENCODER_OFFSET = (Chassis.IS_NEO_SWERVE)? (114.257812 + 120): (-68.582);  
            public static final Double PRIMARY_ENCODER_SENSITIVITY = (4096.0);    
            public static final Boolean PRIMARY_ENCODER_INVERTED = (false);
            public static final Integer PRIMARY_ENCODER_ID = (Chassis.IS_NEO_SWERVE)? (32): (5);            

            public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
            public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
            public static final Double MODEL_ACCELERATION_GAIN = (0.27); 
            public static final Double MODEL_VELOCITY_GAIN = (0.01);

            public static final Double DISCRETIZATION_TIMESTEP = (1/50.0);            
          }

          public static final class Components {
            public static final WPI_CANCoder PRIMARY_ENCODER = LinearSystemModule.configureRotationEncoder((new WPI_CANCoder(Values.PRIMARY_ENCODER_ID)), Values.PRIMARY_ENCODER_OFFSET, Values.PRIMARY_ENCODER_INVERTED);
            public static final MotorController LINEAR_CONTROLLER = (Constants.Values.Chassis.IS_NEO_SWERVE)?
              (LinearSystemModule.configureController(new CANSparkMax(
                      Values.LINEAR_ID,
                      Values.LINEAR_MOTOR_TYPE),
                      Values.LINEAR_MAXIMUM_AMPERAGE,
                      Values.LINEAR_NOMINAL_VOLTAGE,
                      Values.LINEAR_INVERTED)):
              (LinearSystemModule.configureTranslationController(new WPI_TalonFX(
                      Values.LINEAR_ID, Values.LINEAR_NAME),
                      Constants.Values.ComponentData.DRIVE_CURRENT_LIMIT,
                      Values.LINEAR_INVERTED));
            public static final FlywheelSim LINEAR_FLYWHEEL = (Chassis.IS_SIMULATED)?
            (new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                (Chassis.IS_NEO_SWERVE)?
                  (DCMotor.getNEO((1))):
                  (DCMotor.getFalcon500((1))),
                 (505.597120488),                            
                (Chassis.DRIVETRAIN_LINEAR_GEAR_RATIO)),
              (Chassis.IS_NEO_SWERVE)?
                (DCMotor.getNEO((1))):
                (DCMotor.getFalcon500((1))),
              (Chassis.DRIVETRAIN_LINEAR_GEAR_RATIO), 
              (VecBuilder.fill((0.025))))):       
            (null);
            public static final RelativeEncoder LINEAR_ENCODER = (Chassis.IS_NEO_SWERVE)? (LinearSystemModule.configureEncoder(((CANSparkMax)LINEAR_CONTROLLER).getEncoder(),(Chassis.LINEAR_ENCODER_VELOCITY_FACTOR),(Chassis.AZIMUTH_ENCODER_POSITION_FACTOR))): (null);
            public static final MotorController AZIMUTH_CONTROLLER = (Constants.Values.Chassis.IS_NEO_SWERVE)?
              (LinearSystemModule.configureController(new CANSparkMax(
                      Values.AZIMUTH_ID,
                      Values.AZIMUTH_MOTOR_TYPE),
                      Values.AZIMUTH_MAXIMUM_AMPERAGE,
                      Values.AZIMUTH_NOMINAL_VOLTAGE,
                      Values.AZIMUTH_INVERTED)):
              (LinearSystemModule.configureController(new WPI_TalonFX(
                      Values.AZIMUTH_ID, Values.AZIMUTH_NAME),
                      PRIMARY_ENCODER,
                      Constants.Values.ComponentData.AZIMUTH_CURRENT_LIMIT,
                      Constants.Values.ComponentData.AZIMUTH_DEADBAND, 
                      Values.AZIMUTH_INVERTED));

            public static final FlywheelSim AZIMUTH_FLYWHEEL = (Chassis.IS_SIMULATED)?
            (new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                (Chassis.IS_NEO_SWERVE)?
                  (DCMotor.getNEO((1))):
                  (DCMotor.getFalcon500((1))),
                (249.71004822),                     
                (Chassis.DRIVETRAIN_AZIMUTH_GEAR_RATIO)),
              (Chassis.IS_NEO_SWERVE)?
                (DCMotor.getNEO((1))):
                (DCMotor.getFalcon500((1))),
              (Chassis.DRIVETRAIN_AZIMUTH_GEAR_RATIO), 
              (VecBuilder.fill((0.025))))):          
            (null);
            public static final RelativeEncoder AZIMUTH_ENCODER = (Chassis.IS_NEO_SWERVE)? (LinearSystemModule.configureEncoder(((CANSparkMax)LINEAR_CONTROLLER).getEncoder(),(Chassis.LINEAR_ENCODER_VELOCITY_FACTOR),(Chassis.AZIMUTH_ENCODER_POSITION_FACTOR))): (null);
            public static final Supplier<SwerveModuleState> STATE_SENSOR = (Constants.Values.Chassis.IS_NEO_SWERVE)?
              (() -> new SwerveModuleState(
                  (LINEAR_ENCODER.getVelocity() / (1000.0)),
                  Rotation2d.fromDegrees(PRIMARY_ENCODER.getPosition() % (360))
              )):
              (() -> {
                  assert LINEAR_CONTROLLER instanceof WPI_TalonFX;
                  return new SwerveModuleState(
                    (2.0)*(((((WPI_TalonFX)LINEAR_CONTROLLER).getSelectedSensorVelocity() / Values.LINEAR_ENCODER_SENSITIVITY) * (10)) / Constants.Values.Chassis.DRIVETRAIN_AZIMUTH_GEAR_RATIO) * Math.PI * Constants.Values.Chassis.WHEEL_DIAMETER,
                    Rotation2d.fromDegrees(PRIMARY_ENCODER.getPosition() % (360)));
              });
              public static final LinearSystemModule MODULE = (Constants.Values.Chassis.IS_SIMULATED)? 
              (new LinearSystemModule(
                LINEAR_FLYWHEEL,
                AZIMUTH_FLYWHEEL,
                STATE_SENSOR,
                Values.LINEAR_MAXIMUM_VELOCITY,
                Control.CONSTRAINTS,
                Control.CONTROL_LOOP, 
                Units.degreesToRadians(Values.PRIMARY_ENCODER_OFFSET))):
              (new LinearSystemModule(
                (LINEAR_CONTROLLER),
                (AZIMUTH_CONTROLLER),
                STATE_SENSOR,
                Values.LINEAR_MAXIMUM_VELOCITY,
                Control.CONSTRAINTS,
                Control.CONTROL_LOOP));
          }
      
                                                                                                                                                                                             
          public static final class Control {
            public static final Double REGULATOR_QELMS_DEGREES_ACCURACY_POSITION_WEIGHT = (1.0);
            public static final Double REGULATOR_QELMS_ROTATIONS_ACCURACY_VELOCITY_WEIGHT = (1.0);
            public static final Double REGULATOR_RELMS_CONTROLLER_EFFORT_WEIGHT = (0.01);                                                                                                   
            public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.AZIMUTH_MAXIMUM_VELOCITY, Values.AZIMUTH_MAXIMUM_ACCELERATION);              
            public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.MODEL_VELOCITY_GAIN, Values.MODEL_ACCELERATION_GAIN);                                  
            public static final LinearSystemLoop<N2,N1,N1> CONTROL_LOOP = new LinearSystemLoop<>(                                                                                                        
              PLANT,                                                                                                                                                                                  
              new LinearQuadraticRegulator<>(                                                                                                                                                         
                PLANT,                                                                                                                                                                                 
                VecBuilder.fill(Units.degreesToRadians(REGULATOR_QELMS_DEGREES_ACCURACY_POSITION_WEIGHT), Units.rotationsPerMinuteToRadiansPerSecond(REGULATOR_QELMS_ROTATIONS_ACCURACY_VELOCITY_WEIGHT)), 
                VecBuilder.fill(Values.AZIMUTH_MAXIMUM_VOLTAGE),                                                                                                                                         
                Values.DISCRETIZATION_TIMESTEP),                                                                                                                          
              new KalmanFilter<>(                                                                                                                                               
                  Nat.N2(),                                                                                                                                                                      
                  Nat.N1(),                                                                                                                                                                    
                  PLANT,                                                                                                                                                              
                  VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES), Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)),                
                  VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(1 / Values.PRIMARY_ENCODER_SENSITIVITY)),                                                                 
                  Values.DISCRETIZATION_TIMESTEP),                                                                                                                                    
              Values.AZIMUTH_MAXIMUM_VOLTAGE,                                                                                                                                                
              Values.DISCRETIZATION_TIMESTEP);                                                                                                                                         
          }
        }

        public static final class RL_Module {

          public static final class Values {
            public static final CANSparkMaxLowLevel.MotorType AZIMUTH_MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;  
            public static final String AZIMUTH_NAME = ("AZIMUTH [RL]");            
            public static final Double AZIMUTH_MAXIMUM_ACCELERATION = (Math.PI*2);    
            public static final Double AZIMUTH_MAXIMUM_VOLTAGE = (12.0);            
            public static final Double AZIMUTH_MAXIMUM_VELOCITY = (5.4);
            public static final Double AZIMUTH_NOMINAL_VOLTAGE = (12.0);
            public static final Integer AZIMUTH_MAXIMUM_AMPERAGE = (20);   
            public static final Integer AZIMUTH_ID = (Chassis.IS_NEO_SWERVE)? (23): (23);    
            public static final Boolean AZIMUTH_INVERTED = (true);

            public static final CANSparkMaxLowLevel.MotorType LINEAR_MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless; 
            public static final String LINEAR_NAME = ("LINEAR [RL]");             
            public static final Double LINEAR_ENCODER_SENSITIVITY = (2048.0);                          
            public static final Double LINEAR_MAXIMUM_VELOCITY = (5.4);
            public static final Double LINEAR_NOMINAL_VOLTAGE = (12.0);      
            public static final Integer LINEAR_MAXIMUM_AMPERAGE = (80);
            public static final Integer LINEAR_ID = (Chassis.IS_NEO_SWERVE)? (13): (13);   
            public static final Boolean LINEAR_INVERTED = (false);

            public static final String PRIMARY_ENCODER_NAME = ("ENCODER [RL]");
            public static final Double PRIMARY_ENCODER_OFFSET = (Chassis.IS_NEO_SWERVE)? (302.695312 - 45): (134.209);   
            public static final Double PRIMARY_ENCODER_SENSITIVITY = (4096.0);    
            public static final Boolean PRIMARY_ENCODER_INVERTED = (false);
            public static final Integer PRIMARY_ENCODER_ID = (Chassis.IS_NEO_SWERVE)? (33): (6);

            public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
            public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
            public static final Double MODEL_ACCELERATION_GAIN = (0.27); 
            public static final Double MODEL_VELOCITY_GAIN = (0.01);

            public static final Double DISCRETIZATION_TIMESTEP = (1/50.0);            
          }

          public static final class Components {
            public static final WPI_CANCoder PRIMARY_ENCODER = LinearSystemModule.configureRotationEncoder((new WPI_CANCoder(Values.PRIMARY_ENCODER_ID)), Values.PRIMARY_ENCODER_OFFSET, Values.PRIMARY_ENCODER_INVERTED);
            public static final MotorController LINEAR_CONTROLLER = (Constants.Values.Chassis.IS_NEO_SWERVE)?
              (LinearSystemModule.configureController(new CANSparkMax(
                      Values.LINEAR_ID,
                      Values.LINEAR_MOTOR_TYPE),
                      Values.LINEAR_MAXIMUM_AMPERAGE,
                      Values.LINEAR_NOMINAL_VOLTAGE,
                      Values.LINEAR_INVERTED)):
              (LinearSystemModule.configureTranslationController(new WPI_TalonFX(
                      Values.LINEAR_ID, Values.LINEAR_NAME),
                      Constants.Values.ComponentData.DRIVE_CURRENT_LIMIT,
                      Values.LINEAR_INVERTED));
            public static final FlywheelSim LINEAR_FLYWHEEL = (Chassis.IS_SIMULATED)?
            (new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                (Chassis.IS_NEO_SWERVE)?
                  (DCMotor.getNEO((1))):
                  (DCMotor.getFalcon500((1))),
                 (505.597120488),                            
                (Chassis.DRIVETRAIN_LINEAR_GEAR_RATIO)),
              (Chassis.IS_NEO_SWERVE)?
                (DCMotor.getNEO((1))):
                (DCMotor.getFalcon500((1))),
              (Chassis.DRIVETRAIN_LINEAR_GEAR_RATIO), 
              (VecBuilder.fill((0.025))))):       
            (null);
            public static final RelativeEncoder LINEAR_ENCODER = (Chassis.IS_NEO_SWERVE)? (LinearSystemModule.configureEncoder(((CANSparkMax)LINEAR_CONTROLLER).getEncoder(),(Chassis.LINEAR_ENCODER_VELOCITY_FACTOR),(Chassis.AZIMUTH_ENCODER_POSITION_FACTOR))): (null);
            public static final MotorController AZIMUTH_CONTROLLER = (Constants.Values.Chassis.IS_NEO_SWERVE)?
              (LinearSystemModule.configureController(new CANSparkMax(
                      Values.AZIMUTH_ID,
                      Values.AZIMUTH_MOTOR_TYPE),
                      Values.AZIMUTH_MAXIMUM_AMPERAGE,
                      Values.AZIMUTH_NOMINAL_VOLTAGE,
                      Values.AZIMUTH_INVERTED)):
              (LinearSystemModule.configureController(new WPI_TalonFX(
                      Values.AZIMUTH_ID, Values.AZIMUTH_NAME),
                      PRIMARY_ENCODER,
                      Constants.Values.ComponentData.AZIMUTH_CURRENT_LIMIT,
                      Constants.Values.ComponentData.AZIMUTH_DEADBAND, 
                      Values.AZIMUTH_INVERTED));

            public static final FlywheelSim AZIMUTH_FLYWHEEL = (Chassis.IS_SIMULATED)?
            (new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                (Chassis.IS_NEO_SWERVE)?
                  (DCMotor.getNEO((1))):
                  (DCMotor.getFalcon500((1))),
                (249.71004822),                     
                (Chassis.DRIVETRAIN_AZIMUTH_GEAR_RATIO)),
              (Chassis.IS_NEO_SWERVE)?
                (DCMotor.getNEO((1))):
                (DCMotor.getFalcon500((1))),
              (Chassis.DRIVETRAIN_AZIMUTH_GEAR_RATIO), 
              (VecBuilder.fill((0.025))))):          
            (null);
            public static final RelativeEncoder AZIMUTH_ENCODER = (Chassis.IS_NEO_SWERVE)? (LinearSystemModule.configureEncoder(((CANSparkMax)LINEAR_CONTROLLER).getEncoder(),(Chassis.LINEAR_ENCODER_VELOCITY_FACTOR),(Chassis.AZIMUTH_ENCODER_POSITION_FACTOR))): (null);
            public static final Supplier<SwerveModuleState> STATE_SENSOR = (Constants.Values.Chassis.IS_NEO_SWERVE)?
              (() -> new SwerveModuleState(
                  (LINEAR_ENCODER.getVelocity() / (1000.0)),
                  Rotation2d.fromDegrees(PRIMARY_ENCODER.getPosition() % (360))
              )):
              (() -> {
                  assert LINEAR_CONTROLLER instanceof WPI_TalonFX;
                  return new SwerveModuleState(
                    (2.0)*(((((WPI_TalonFX)LINEAR_CONTROLLER).getSelectedSensorVelocity() / Values.LINEAR_ENCODER_SENSITIVITY) * (10)) / Constants.Values.Chassis.DRIVETRAIN_AZIMUTH_GEAR_RATIO) * Math.PI * Constants.Values.Chassis.WHEEL_DIAMETER,
                    Rotation2d.fromDegrees(PRIMARY_ENCODER.getPosition() % (360)));
              });
              public static final LinearSystemModule MODULE = (Constants.Values.Chassis.IS_SIMULATED)? 
              (new LinearSystemModule(
                LINEAR_FLYWHEEL,
                AZIMUTH_FLYWHEEL,
                STATE_SENSOR,
                Values.LINEAR_MAXIMUM_VELOCITY,
                Control.CONSTRAINTS,
                Control.CONTROL_LOOP, 
                Units.degreesToRadians(Values.PRIMARY_ENCODER_OFFSET))):
              (new LinearSystemModule(
                (LINEAR_CONTROLLER),
                (AZIMUTH_CONTROLLER),
                STATE_SENSOR,
                Values.LINEAR_MAXIMUM_VELOCITY,
                Control.CONSTRAINTS,
                Control.CONTROL_LOOP));
          }
      
          public static final class Control {
            public static final Double REGULATOR_QELMS_DEGREES_ACCURACY_POSITION_WEIGHT = (1.0);
            public static final Double REGULATOR_QELMS_ROTATIONS_ACCURACY_VELOCITY_WEIGHT = (1.0);
            public static final Double REGULATOR_RELMS_CONTROLLER_EFFORT_WEIGHT = (0.01);                                                                                                            
            public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.AZIMUTH_MAXIMUM_VELOCITY, Values.AZIMUTH_MAXIMUM_ACCELERATION);              
            public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.MODEL_VELOCITY_GAIN, Values.MODEL_ACCELERATION_GAIN);                                  
            public static final LinearSystemLoop<N2,N1,N1> CONTROL_LOOP = new LinearSystemLoop<>(                                                                                                        
              PLANT,                                                                                                                                                                                  
              new LinearQuadraticRegulator<>(                                                                                                                                                         
                PLANT,                                                                                                                                                                                 
                VecBuilder.fill(Units.degreesToRadians(REGULATOR_QELMS_DEGREES_ACCURACY_POSITION_WEIGHT), Units.rotationsPerMinuteToRadiansPerSecond(REGULATOR_QELMS_ROTATIONS_ACCURACY_VELOCITY_WEIGHT)), 
                VecBuilder.fill(Values.AZIMUTH_MAXIMUM_VOLTAGE),                                                                                                                                         
                Values.DISCRETIZATION_TIMESTEP),                                                                                                                          
              new KalmanFilter<>(                                                                                                                                               
                  Nat.N2(),                                                                                                                                                                      
                  Nat.N1(),                                                                                                                                                                    
                  PLANT,                                                                                                                                                              
                  VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES), Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)),                
                  VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(1 / Values.PRIMARY_ENCODER_SENSITIVITY)),                                                                 
                  Values.DISCRETIZATION_TIMESTEP),                                                                                                                                    
              Values.AZIMUTH_MAXIMUM_VOLTAGE,                                                                                                                                                
              Values.DISCRETIZATION_TIMESTEP);                                                                                                                                         
          }
        }

        public static final class RR_Module {

          public static final class Values {

            public static final CANSparkMaxLowLevel.MotorType AZIMUTH_MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;  
            public static final String AZIMUTH_NAME = ("AZIMUTH [RR]"); 
            public static final Double AZIMUTH_MAXIMUM_ACCELERATION = (Math.PI*2);    
            public static final Double AZIMUTH_MAXIMUM_VOLTAGE = (12.0);            
            public static final Double AZIMUTH_MAXIMUM_VELOCITY = (5.4);
            public static final Double AZIMUTH_NOMINAL_VOLTAGE = (12.0);
            public static final Integer AZIMUTH_MAXIMUM_AMPERAGE = (20);   
            public static final Integer AZIMUTH_ID = (Chassis.IS_NEO_SWERVE)? (24): (24);    
            public static final Boolean AZIMUTH_INVERTED = (true);

            public static final CANSparkMaxLowLevel.MotorType LINEAR_MOTOR_TYPE = CANSparkMaxLowLevel.MotorType.kBrushless;  
            public static final String LINEAR_NAME = ("AZIMUTH [RR]");            
            public static final Double LINEAR_ENCODER_SENSITIVITY = (2048.0);                          
            public static final Double LINEAR_MAXIMUM_VELOCITY = (5.4);
            public static final Double LINEAR_NOMINAL_VOLTAGE = (12.0);      
            public static final Integer LINEAR_MAXIMUM_AMPERAGE = (80);
            public static final Integer LINEAR_ID = (Chassis.IS_NEO_SWERVE)? (14): (14);   
            public static final Boolean LINEAR_INVERTED = (false);

            public static final String PRIMARY_ENCODER_NAME = ("AZIMUTH [RR]");
            public static final Double PRIMARY_ENCODER_OFFSET = (Chassis.IS_NEO_SWERVE)? (358.154297 + 180): (-257.783);             
            public static final Double PRIMARY_ENCODER_SENSITIVITY = (4096.0);    
            public static final Boolean PRIMARY_ENCODER_INVERTED = (false);
            public static final Integer PRIMARY_ENCODER_ID = (Chassis.IS_NEO_SWERVE)? (34): (7);

            public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
            public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
            public static final Double MODEL_ACCELERATION_GAIN = (0.27); 
            public static final Double MODEL_VELOCITY_GAIN = (0.01);

            public static final Double DISCRETIZATION_TIMESTEP = (1/50.0);            
          }

          public static final class Components {
            public static final WPI_CANCoder PRIMARY_ENCODER = LinearSystemModule.configureRotationEncoder((new WPI_CANCoder(Values.PRIMARY_ENCODER_ID)), Values.PRIMARY_ENCODER_OFFSET, Values.PRIMARY_ENCODER_INVERTED);
            public static final MotorController LINEAR_CONTROLLER = (Constants.Values.Chassis.IS_NEO_SWERVE)?
              (LinearSystemModule.configureController(new CANSparkMax(
                      Values.LINEAR_ID,
                      Values.LINEAR_MOTOR_TYPE),
                      Values.LINEAR_MAXIMUM_AMPERAGE,
                      Values.LINEAR_NOMINAL_VOLTAGE,
                      Values.LINEAR_INVERTED)):
              (LinearSystemModule.configureTranslationController(new WPI_TalonFX(
                      Values.LINEAR_ID, Values.LINEAR_NAME),
                      Constants.Values.ComponentData.DRIVE_CURRENT_LIMIT,
                      Values.LINEAR_INVERTED));
            public static final FlywheelSim LINEAR_FLYWHEEL = (Chassis.IS_SIMULATED)?
            (new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                (Chassis.IS_NEO_SWERVE)?
                  (DCMotor.getNEO((1))):
                  (DCMotor.getFalcon500((1))),
                 (505.597120488),                            
                (Chassis.DRIVETRAIN_LINEAR_GEAR_RATIO)),
              (Chassis.IS_NEO_SWERVE)?
                (DCMotor.getNEO((1))):
                (DCMotor.getFalcon500((1))),
              (Chassis.DRIVETRAIN_LINEAR_GEAR_RATIO), 
              (VecBuilder.fill((0.025))))):       
            (null);
            public static final RelativeEncoder LINEAR_ENCODER = (Chassis.IS_NEO_SWERVE)? (LinearSystemModule.configureEncoder(((CANSparkMax)LINEAR_CONTROLLER).getEncoder(),(Chassis.LINEAR_ENCODER_VELOCITY_FACTOR),(Chassis.AZIMUTH_ENCODER_POSITION_FACTOR))): (null);
            public static final MotorController AZIMUTH_CONTROLLER = (Constants.Values.Chassis.IS_NEO_SWERVE)?
              (LinearSystemModule.configureController(new CANSparkMax(
                      Values.AZIMUTH_ID,
                      Values.AZIMUTH_MOTOR_TYPE),
                      Values.AZIMUTH_MAXIMUM_AMPERAGE,
                      Values.AZIMUTH_NOMINAL_VOLTAGE,
                      Values.AZIMUTH_INVERTED)):
              (LinearSystemModule.configureController(new WPI_TalonFX(
                      Values.AZIMUTH_ID, Values.AZIMUTH_NAME),
                      PRIMARY_ENCODER,
                      Constants.Values.ComponentData.AZIMUTH_CURRENT_LIMIT,
                      Constants.Values.ComponentData.AZIMUTH_DEADBAND, 
                      Values.AZIMUTH_INVERTED));

            public static final FlywheelSim AZIMUTH_FLYWHEEL = (Chassis.IS_SIMULATED)?
            (new FlywheelSim(
              LinearSystemId.createFlywheelSystem(
                (Chassis.IS_NEO_SWERVE)?
                  (DCMotor.getNEO((1))):
                  (DCMotor.getFalcon500((1))),
                (249.71004822),                     
                (Chassis.DRIVETRAIN_AZIMUTH_GEAR_RATIO)),
              (Chassis.IS_NEO_SWERVE)?
                (DCMotor.getNEO((1))):
                (DCMotor.getFalcon500((1))),
              (Chassis.DRIVETRAIN_AZIMUTH_GEAR_RATIO), 
              (VecBuilder.fill((0.025))))):          
            (null);
            public static final RelativeEncoder AZIMUTH_ENCODER = (Chassis.IS_NEO_SWERVE)? (LinearSystemModule.configureEncoder(((CANSparkMax)LINEAR_CONTROLLER).getEncoder(),(Chassis.LINEAR_ENCODER_VELOCITY_FACTOR),(Chassis.AZIMUTH_ENCODER_POSITION_FACTOR))): (null);
            public static final Supplier<SwerveModuleState> STATE_SENSOR = (Constants.Values.Chassis.IS_NEO_SWERVE)?
              (() -> new SwerveModuleState(
                  (LINEAR_ENCODER.getVelocity() / (1000.0)),
                  Rotation2d.fromDegrees(PRIMARY_ENCODER.getPosition() % (360))
              )):
              (() -> {
                  assert LINEAR_CONTROLLER instanceof WPI_TalonFX;
                  return new SwerveModuleState(
                    (2.0)*(((((WPI_TalonFX)LINEAR_CONTROLLER).getSelectedSensorVelocity() / Values.LINEAR_ENCODER_SENSITIVITY) * (10)) / Constants.Values.Chassis.DRIVETRAIN_AZIMUTH_GEAR_RATIO) * Math.PI * Constants.Values.Chassis.WHEEL_DIAMETER,
                    Rotation2d.fromDegrees(PRIMARY_ENCODER.getPosition() % (360)));
              });
              public static final LinearSystemModule MODULE = (Constants.Values.Chassis.IS_SIMULATED)? 
              (new LinearSystemModule(
                LINEAR_FLYWHEEL,
                AZIMUTH_FLYWHEEL,
                STATE_SENSOR,
                Values.LINEAR_MAXIMUM_VELOCITY,
                Control.CONSTRAINTS,
                Control.CONTROL_LOOP, 
                Units.degreesToRadians(Values.PRIMARY_ENCODER_OFFSET))):
              (new LinearSystemModule(
                (LINEAR_CONTROLLER),
                (AZIMUTH_CONTROLLER),
                STATE_SENSOR,
                Values.LINEAR_MAXIMUM_VELOCITY,
                Control.CONSTRAINTS,
                Control.CONTROL_LOOP));
          }

          public static final class Control {
            public static final Double REGULATOR_QELMS_DEGREES_ACCURACY_POSITION_WEIGHT = (1.0);
            public static final Double REGULATOR_QELMS_ROTATIONS_ACCURACY_VELOCITY_WEIGHT = (1.0);
            public static final Double REGULATOR_RELMS_CONTROLLER_EFFORT_WEIGHT = (0.01);                                                                                                                
            public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.AZIMUTH_MAXIMUM_VELOCITY, Values.AZIMUTH_MAXIMUM_ACCELERATION);              
            public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.MODEL_VELOCITY_GAIN, Values.MODEL_ACCELERATION_GAIN);                                  
            public static final LinearSystemLoop<N2,N1,N1> CONTROL_LOOP = new LinearSystemLoop<>(                                                                                                        
              PLANT,                                                                                                                                                                                  
              new LinearQuadraticRegulator<>(                                                                                                                                                         
                PLANT,                                                                                                                                                                                 
                VecBuilder.fill(Units.degreesToRadians(REGULATOR_QELMS_DEGREES_ACCURACY_POSITION_WEIGHT), Units.rotationsPerMinuteToRadiansPerSecond(REGULATOR_QELMS_ROTATIONS_ACCURACY_VELOCITY_WEIGHT)), 
                VecBuilder.fill(Values.AZIMUTH_MAXIMUM_VOLTAGE),                                                                                                                                         
                Values.DISCRETIZATION_TIMESTEP),                                                                                                                          
              new KalmanFilter<>(                                                                                                                                               
                  Nat.N2(),                                                                                                                                                                      
                  Nat.N1(),                                                                                                                                                                    
                  PLANT,                                                                                                                                                              
                  VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES), Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)),                
                  VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(1 / Values.PRIMARY_ENCODER_SENSITIVITY)),                                                                 
                  Values.DISCRETIZATION_TIMESTEP),                                                                                                                                    
              Values.AZIMUTH_MAXIMUM_VOLTAGE,                                                                                                                                                
              Values.DISCRETIZATION_TIMESTEP);                                                                                                                                         
          }
        }
    }
  }
}
