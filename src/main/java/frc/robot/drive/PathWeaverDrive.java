package frc.robot.drive;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.maps.RobotMap;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class PathWeaverDrive extends SubsystemBase {
    CANSparkMax leftMaster = new CANSparkMax(RobotMap.LEFT_MIDDLE, MotorType.kBrushless);
    CANSparkMax leftFront = new CANSparkMax(RobotMap.LEFT_FRONT, MotorType.kBrushless);
    CANSparkMax leftRear = new CANSparkMax(RobotMap.LEFT_REAR, MotorType.kBrushless);
    CANSparkMax rightMaster = new CANSparkMax(RobotMap.RIGHT_MIDDLE, MotorType.kBrushless);
    CANSparkMax rightFront = new CANSparkMax(RobotMap.RIGHT_FRONT, MotorType.kBrushless);
    CANSparkMax rightRear = new CANSparkMax(RobotMap.RIGHT_REAR, MotorType.kBrushless);

    private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(leftMaster, leftFront, leftRear);
    private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(rightMaster, rightFront, rightRear);

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    private final AHRS m_gyro = new AHRS(Port.kMXP);

    private DifferentialDriveOdometry m_odometry;
    
    public PathWeaverDrive() {
        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }

    @Override
    public void periodic() {
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderInches(), getRightEncoderInches());
    }

    //return is in incorrect unit
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftMaster.getEncoder().getVelocity(), rightMaster.getEncoder().getVelocity())
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void resetEncoders() {
        leftMaster.getEncoder().setPosition(0);
        rightMaster.getEncoder().setPosition(0);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(-rightVolts);
    }

    public double getAverageEncoderDistance() {
        return (getLeftEncoderInches() + getRightEncoderInches()) / 2.0;
    }

    public double getLeftEncoderValue() {
        return leftRear.getEncoder().getPosition();
    }

    public double getRightEncoderValue() {
        return rightMaster.getEncoder().getPosition();
    }

    public double getLeftEncoderInches() {
        return -(RobotMap.WHEEL_DIAMETER * Math.PI * (getLeftEncoderValue() / RobotMap.ENCODER_TICK_PER_REV));
    }

    public double getRightEncoderInches() {
        return (RobotMap.WHEEL_DIAMETER * Math.PI * (getRightEncoderValue() / RobotMap.ENCODER_TICK_PER_REV));
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360);
    }
    
    public Command getAutonomousCommand() {
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(RobotMap.kS_VOLTS, RobotMap.kV_VOLT_SECONDS_PER_FOOT, RobotMap.kA_VOLT_SECONDS_PER_SQUARE_FOOT), RobotMap.kDRIVE_KINEMATICS, 10);

        TrajectoryConfig config = new TrajectoryConfig(RobotMap.kMAX_SPEED_FEET_PER_SECOND, RobotMap.kMAX_ACCELERATION_FEET_PER_SECOND_SQUARED).setKinematics(RobotMap.kDRIVE_KINEMATICS).addConstraint(autoVoltageConstraint);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(1, 1)), new Pose2d(3, 0, new Rotation2d(0)), config);
    
        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, arachne::getPose, new RamseteController(RobotMap.kRAMSETE_B, RobotMap.kRAMSETE_ZETA),
        new SimpleMotorFeedforward(RobotMap.kS_VOLTS, RobotMap.kV_VOLT_SECONDS_PER_FOOT, RobotMap.kA_VOLT_SECONDS_PER_SQUARE_FOOT), RobotMap.kDRIVE_KINEMATICS, arachne::getWheelSpeeds,
        new PIDController(RobotMap.kP_DRIVE_VEL, 0, 0), new PIDController(RobotMap.kP_DRIVE_VEL, 0, 0), arachne::tankDriveVolts, arachne);

        return ramseteCommand.andThen(() -> arachne.tankDriveVolts(0, 0));
    }
}