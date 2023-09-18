// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot.subsystems.drivebase;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import java.util.function.Supplier;



// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//

import frc.lib.SwerveModule;
// ------------------------------------------------------------[Constants Class]------------------------------------------------------------//
public final class Constants {
    
    public static final Integer PATHPLANNER_SERVER_PORT = (6969); 

    public static final class FL_Module {

      public static final class Values {
        public static final Double MAXIMUM_AZIMUTH_ACCELERATION = (Math.PI*2);    
        public static final Double MAXIMM_AZIMUTH_VELOCITY = (Math.PI * 2);                  
        public static final Double MAXIMUM_AZIMUTH_VOLTAGE = (12.0);        
        public static final Double MAXIMUM_LINEAR_VELOCITY = (5.4);
        public static final Double ACCELERATION_GAIN = (0.0); //TODO: Find Value
        public static final Double VELOCITY_GAIN = (0.0); //TODO: Find Value

        public static final Double ENCODER_POSITION_ACCURACY_DEGREES = (2.44140625e-4);
        public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
        public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
        public static final Double DISCRETIZATION_TIMESTEP = (0.02);

        public static final Integer AZIMUTH_ID = (21);                       
        public static final Integer LINEAR_ID = (11);    
        public static final Integer SENSOR_ID = (4);
      }

      public static final class Hardware {
        public static final WPI_CANCoder AZIMUTH_SENSOR = new WPI_CANCoder(Values.SENSOR_ID);
        public static final Supplier<SwerveModuleState> STATE_SENSOR = () -> new SwerveModuleState();
        public static final MotorController AZIMUTH_CONTROLLER = new WPI_TalonFX(Values.AZIMUTH_ID);                                 
        public static final MotorController LINEAR_CONTROLLER = new WPI_TalonFX(Values.LINEAR_ID);
        public static final SwerveModule MODULE = new SwerveModule(new MotorControllerGroup(AZIMUTH_CONTROLLER), new MotorControllerGroup(LINEAR_CONTROLLER),STATE_SENSOR,() -> Control.CONSTRAINTS,Control.LOOP, () -> Values.MAXIMUM_LINEAR_VELOCITY);
      }
  

      public static final class Control {
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.MAXIMM_AZIMUTH_VELOCITY, Values.MAXIMUM_AZIMUTH_ACCELERATION);
        public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.VELOCITY_GAIN, Values.ACCELERATION_GAIN);        
        public static final LinearSystemLoop<N2,N1,N1> LOOP = new LinearSystemLoop<N2,N1,N1>(
          PLANT,
          new LinearQuadraticRegulator<>(
            PLANT,
            VecBuilder.fill(Units.degreesToRadians(Hardware.AZIMUTH_SENSOR.getAbsolutePosition()),Units.rotationsPerMinuteToRadiansPerSecond(Hardware.AZIMUTH_SENSOR.getVelocity())), 
            VecBuilder.fill(Values.MAXIMUM_AZIMUTH_VOLTAGE),
            Values.DISCRETIZATION_TIMESTEP),
          new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            PLANT, 
            VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES),Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)),
            VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(Values.ENCODER_POSITION_ACCURACY_DEGREES)),
            Values.DISCRETIZATION_TIMESTEP),
          Values.MAXIMUM_AZIMUTH_VOLTAGE,
          Values.DISCRETIZATION_TIMESTEP);        
      }

    }
    public static final class FR_Module {

      public static final class Values {
        public static final Double MAXIMUM_AZIMUTH_ACCELERATION = (Math.PI*2);    
        public static final Double MAXIMM_AZIMUTH_VELOCITY = (Math.PI * 2);                  
        public static final Double MAXIMUM_AZIMUTH_VOLTAGE = (12.0);        
        public static final Double MAXIMUM_LINEAR_VELOCITY = (5.4);
        public static final Double ACCELERATION_GAIN = (0.0); //TODO: Find Value
        public static final Double VELOCITY_GAIN = (0.0); //TODO: Find Value

        public static final Double ENCODER_POSITION_ACCURACY_DEGREES = (2.44140625e-4);
        public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
        public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
        public static final Double DISCRETIZATION_TIMESTEP = (0.02);

        public static final Integer AZIMUTH_ID = (22);                       
        public static final Integer LINEAR_ID = (12);    
        public static final Integer SENSOR_ID = (5);
      }

      public static final class Hardware {
        public static final WPI_CANCoder AZIMUTH_SENSOR = new WPI_CANCoder(Values.SENSOR_ID);
        public static final Supplier<SwerveModuleState> STATE_SENSOR = () -> new SwerveModuleState();
        public static final MotorController AZIMUTH_CONTROLLER = new WPI_TalonFX(Values.AZIMUTH_ID);                                 
        public static final MotorController LINEAR_CONTROLLER = new WPI_TalonFX(Values.LINEAR_ID);
        public static final SwerveModule MODULE = new SwerveModule(new MotorControllerGroup(AZIMUTH_CONTROLLER), new MotorControllerGroup(LINEAR_CONTROLLER),STATE_SENSOR,() -> Control.CONSTRAINTS,Control.LOOP, () -> Values.MAXIMUM_LINEAR_VELOCITY);
      }
  

      public static final class Control {
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.MAXIMM_AZIMUTH_VELOCITY, Values.MAXIMUM_AZIMUTH_ACCELERATION);
        public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.VELOCITY_GAIN, Values.ACCELERATION_GAIN);        
        public static final LinearSystemLoop<N2,N1,N1> LOOP = new LinearSystemLoop<N2,N1,N1>(
          PLANT,
          new LinearQuadraticRegulator<>(
            PLANT,
            VecBuilder.fill(Units.degreesToRadians(Hardware.AZIMUTH_SENSOR.getAbsolutePosition()),Units.rotationsPerMinuteToRadiansPerSecond(Hardware.AZIMUTH_SENSOR.getVelocity())), 
            VecBuilder.fill(Values.MAXIMUM_AZIMUTH_VOLTAGE),
            Values.DISCRETIZATION_TIMESTEP),
          new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            PLANT, 
            VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES),Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)),
            VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(Values.ENCODER_POSITION_ACCURACY_DEGREES)),
            Values.DISCRETIZATION_TIMESTEP),
          Values.MAXIMUM_AZIMUTH_VOLTAGE,
          Values.DISCRETIZATION_TIMESTEP);        
      }

    }

    public static final class RL_Module {

      public static final class Values {
        public static final Double MAXIMUM_AZIMUTH_ACCELERATION = (Math.PI*2);    
        public static final Double MAXIMM_AZIMUTH_VELOCITY = (Math.PI * 2);                  
        public static final Double MAXIMUM_AZIMUTH_VOLTAGE = (12.0);        
        public static final Double MAXIMUM_LINEAR_VELOCITY = (5.4);
        public static final Double ACCELERATION_GAIN = (0.0); //TODO: Find Value
        public static final Double VELOCITY_GAIN = (0.0); //TODO: Find Value

        public static final Double ENCODER_POSITION_ACCURACY_DEGREES = (2.44140625e-4);
        public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
        public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
        public static final Double DISCRETIZATION_TIMESTEP = (0.02);

        public static final Integer AZIMUTH_ID = (23);                       
        public static final Integer LINEAR_ID = (13);    
        public static final Integer SENSOR_ID = (6);
      }

      public static final class Hardware {
        public static final WPI_CANCoder AZIMUTH_SENSOR = new WPI_CANCoder(Values.SENSOR_ID);
        public static final Supplier<SwerveModuleState> STATE_SENSOR = () -> new SwerveModuleState();
        public static final MotorController AZIMUTH_CONTROLLER = new WPI_TalonFX(Values.AZIMUTH_ID);                                 
        public static final MotorController LINEAR_CONTROLLER = new WPI_TalonFX(Values.LINEAR_ID);
        public static final SwerveModule MODULE = new SwerveModule(new MotorControllerGroup(AZIMUTH_CONTROLLER), new MotorControllerGroup(LINEAR_CONTROLLER),STATE_SENSOR,() -> Control.CONSTRAINTS,Control.LOOP, () -> Values.MAXIMUM_LINEAR_VELOCITY);
      }
  

      public static final class Control {
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.MAXIMM_AZIMUTH_VELOCITY, Values.MAXIMUM_AZIMUTH_ACCELERATION);
        public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.VELOCITY_GAIN, Values.ACCELERATION_GAIN);        
        public static final LinearSystemLoop<N2,N1,N1> LOOP = new LinearSystemLoop<N2,N1,N1>(
          PLANT,
          new LinearQuadraticRegulator<>(
            PLANT,
            VecBuilder.fill(Units.degreesToRadians(Hardware.AZIMUTH_SENSOR.getAbsolutePosition()),Units.rotationsPerMinuteToRadiansPerSecond(Hardware.AZIMUTH_SENSOR.getVelocity())), 
            VecBuilder.fill(Values.MAXIMUM_AZIMUTH_VOLTAGE),
            Values.DISCRETIZATION_TIMESTEP),
          new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            PLANT, 
            VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES),Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)),
            VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(Values.ENCODER_POSITION_ACCURACY_DEGREES)),
            Values.DISCRETIZATION_TIMESTEP),
          Values.MAXIMUM_AZIMUTH_VOLTAGE,
          Values.DISCRETIZATION_TIMESTEP);        
      }

    }
    public static final class RR_Module {

      public static final class Values {
        public static final Double MAXIMUM_AZIMUTH_ACCELERATION = (Math.PI*2);    
        public static final Double MAXIMM_AZIMUTH_VELOCITY = (Math.PI * 2);                  
        public static final Double MAXIMUM_AZIMUTH_VOLTAGE = (12.0);        
        public static final Double MAXIMUM_LINEAR_VELOCITY = (5.4);
        public static final Double ACCELERATION_GAIN = (0.0); //TODO: Find Value
        public static final Double VELOCITY_GAIN = (0.0); //TODO: Find Value

        public static final Double ENCODER_POSITION_ACCURACY_DEGREES = (2.44140625e-4);
        public static final Double MODEL_VELOCITY_ACCURACY_DEGREES = (7e-5);
        public static final Double MODEL_POSITION_ACCURACY_RPM = (2.5e-4);
        public static final Double DISCRETIZATION_TIMESTEP = (0.02);

        public static final Integer AZIMUTH_ID = (24);                       
        public static final Integer LINEAR_ID = (14);    
        public static final Integer SENSOR_ID = (7);
      }

      public static final class Hardware {
        public static final WPI_CANCoder AZIMUTH_SENSOR = new WPI_CANCoder(Values.SENSOR_ID);
        public static final Supplier<SwerveModuleState> STATE_SENSOR = () -> new SwerveModuleState();
        public static final MotorController AZIMUTH_CONTROLLER = new WPI_TalonFX(Values.AZIMUTH_ID);                                 
        public static final MotorController LINEAR_CONTROLLER = new WPI_TalonFX(Values.LINEAR_ID);
        public static final SwerveModule MODULE = new SwerveModule(new MotorControllerGroup(AZIMUTH_CONTROLLER), new MotorControllerGroup(LINEAR_CONTROLLER),STATE_SENSOR,() -> Control.CONSTRAINTS,Control.LOOP, () -> Values.MAXIMUM_LINEAR_VELOCITY);
      }
  

      public static final class Control {
        public static final TrapezoidProfile.Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Values.MAXIMM_AZIMUTH_VELOCITY, Values.MAXIMUM_AZIMUTH_ACCELERATION);
        public static final LinearSystem<N2,N1,N1> PLANT = LinearSystemId.identifyPositionSystem(Values.VELOCITY_GAIN, Values.ACCELERATION_GAIN);        
        public static final LinearSystemLoop<N2,N1,N1> LOOP = new LinearSystemLoop<N2,N1,N1>(
          PLANT,
          new LinearQuadraticRegulator<>(
            PLANT,
            VecBuilder.fill(Units.degreesToRadians(Hardware.AZIMUTH_SENSOR.getAbsolutePosition()),Units.rotationsPerMinuteToRadiansPerSecond(Hardware.AZIMUTH_SENSOR.getVelocity())), 
            VecBuilder.fill(Values.MAXIMUM_AZIMUTH_VOLTAGE),
            Values.DISCRETIZATION_TIMESTEP),
          new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            PLANT, 
            VecBuilder.fill(Units.degreesToRadians(Values.MODEL_VELOCITY_ACCURACY_DEGREES),Units.rotationsPerMinuteToRadiansPerSecond(Values.MODEL_POSITION_ACCURACY_RPM)),
            VecBuilder.fill(Units.rotationsPerMinuteToRadiansPerSecond(Values.ENCODER_POSITION_ACCURACY_DEGREES)),
            Values.DISCRETIZATION_TIMESTEP),
          Values.MAXIMUM_AZIMUTH_VOLTAGE,
          Values.DISCRETIZATION_TIMESTEP);        
      }

    }
}
