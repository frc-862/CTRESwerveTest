package frc.robot.subsytems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ThunderSwerveRequest extends SwerveRequest {

    public class SwerveDriveBrake extends SwerveRequest.SwerveDriveBrake  {
        @Override
        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {

            for (int i = 0; i < modulesToApply.length; ++i) {
                SwerveModuleState state = new SwerveModuleState(0, parameters.swervePositions[i].getAngle());
                ((ThunderSwerveModule) modulesToApply[i]).apply(i, state, DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        @Override
        public SwerveDriveBrake withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }
        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public SwerveDriveBrake withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    public class FieldCentric extends SwerveRequest.FieldCentric {
        @Override
        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            double toApplyX = VelocityX;
            double toApplyY = VelocityY;
            double toApplyOmega = RotationalRate;
            if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if (Math.abs(toApplyOmega) < RotationalDeadband) {
                toApplyOmega = 0;
            }

            ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                        parameters.currentPose.getRotation()), parameters.updatePeriod);

            var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

            for (int i = 0; i < modulesToApply.length; ++i) {
                ((ThunderSwerveModule) modulesToApply[i]).apply(i, states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Sets the velocity in the X direction, in m/s.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         *
         * @param velocityX Velocity in the X direction, in m/s
         * @return this request
         */
        @Override
        public ThunderSwerveRequest.FieldCentric withVelocityX(double velocityX) {
            this.VelocityX = velocityX;
            return this;
        }

        /**
         * Sets the velocity in the Y direction, in m/s.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         *
         * @param velocityY Velocity in the Y direction, in m/s
         * @return this request
         */
        @Override
        public ThunderSwerveRequest.FieldCentric withVelocityY(double velocityY) {
            this.VelocityY = velocityY;
            return this;
        }

        /**
         * The angular rate to rotate at, in radians per second.
         * Angular rate is defined as counterclockwise positive,
         * so this determines how fast to turn counterclockwise.
         *
         * @param rotationalRate Angular rate to rotate at, in radians per second
         * @return this request
         */
        @Override
        public ThunderSwerveRequest.FieldCentric withRotationalRate(double rotationalRate) {
            this.RotationalRate = rotationalRate;
            return this;
        }

        /**
         * Sets the allowable deadband of the request.
         *
         * @param deadband Allowable deadband of the request
         * @return this request
         */
        @Override
        public ThunderSwerveRequest.FieldCentric withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }
        /**
         * Sets the rotational deadband of the request.
         *
         * @param rotationalDeadband Rotational deadband of the request
         * @return this request
         */
        @Override
        public ThunderSwerveRequest.FieldCentric withRotationalDeadband(double rotationalDeadband) {
            this.RotationalDeadband = rotationalDeadband;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        @Override
        public ThunderSwerveRequest.FieldCentric withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }
        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        @Override
        public ThunderSwerveRequest.FieldCentric withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    public class FieldCentricFacingAngle extends SwerveRequest.FieldCentricFacingAngle {
        @Override
        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            double toApplyX = VelocityX;
            double toApplyY = VelocityY;

            double rotationRate = HeadingController.calculate(parameters.currentPose.getRotation().getRadians(),
                    TargetDirection.getRadians(), parameters.timestamp);

            double toApplyOmega = rotationRate;
            if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if (Math.abs(toApplyOmega) < RotationalDeadband) {
                toApplyOmega = 0;
            }

            ChassisSpeeds speeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                    parameters.currentPose.getRotation()), parameters.updatePeriod);

            var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

            for (int i = 0; i < modulesToApply.length; ++i) {
                ((ThunderSwerveModule) modulesToApply[i]).apply(i, states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Sets the velocity in the X direction, in m/s.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         *
         * @param velocityX Velocity in the X direction, in m/s
         * @return this request
         */
        public FieldCentricFacingAngle withVelocityX(double velocityX) {
            this.VelocityX = velocityX;
            return this;
        }

        /**
         * Sets the velocity in the Y direction, in m/s.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         *
         * @param velocityY Velocity in the Y direction, in m/s
         * @return this request
         */
        public FieldCentricFacingAngle withVelocityY(double velocityY) {
            this.VelocityY = velocityY;
            return this;
        }

        /**
         * Sets the desired direction to face.
         * 0 Degrees is defined as in the direction of the X axis.
         * As a result, a TargetDirection of 90 degrees will point along
         * the Y axis, or to the left.
         *
         * @param targetDirection Desired direction to face
         * @return this request
         */
        public FieldCentricFacingAngle withTargetDirection(Rotation2d targetDirection) {
            this.TargetDirection = targetDirection;
            return this;
        }

        /**
         * Sets the allowable deadband of the request.
         *
         * @param deadband Allowable deadband of the request
         * @return this request
         */
        public FieldCentricFacingAngle withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }
        /**
         * Sets the rotational deadband of the request.
         *
         * @param rotationalDeadband Rotational deadband of the request
         * @return this request
         */
        public FieldCentricFacingAngle withRotationalDeadband(double rotationalDeadband) {
            this.RotationalDeadband = rotationalDeadband;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public FieldCentricFacingAngle withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }
        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public FieldCentricFacingAngle withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    //No logging added for idle

    public class PointWheelsAt extends SwerveRequest.PointWheelsAt {
        @Override
        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {

            for (int i = 0; i < modulesToApply.length; ++i) {
                SwerveModuleState state = new SwerveModuleState(0, ModuleDirection);
                ((ThunderSwerveModule) modulesToApply[i]).apply(i, state, DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Sets the direction to point the modules toward.
         * This direction is still optimized to what the module was previously at.
         *
         * @param moduleDirection Direction to point the modules toward
         * @return this request
         */
        public PointWheelsAt withModuleDirection(Rotation2d moduleDirection) {
            this.ModuleDirection = moduleDirection;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public PointWheelsAt withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }
        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public PointWheelsAt withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    public class RobotCentric extends SwerveRequest.RobotCentric {
        @Override
        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            double toApplyX = VelocityX;
            double toApplyY = VelocityY;
            double toApplyOmega = RotationalRate;
            if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if (Math.abs(toApplyOmega) < RotationalDeadband) {
                toApplyOmega = 0;
            }
            ChassisSpeeds speeds = new ChassisSpeeds(toApplyX, toApplyY, toApplyOmega);

            var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

            for (int i = 0; i < modulesToApply.length; ++i) {
                ((ThunderSwerveModule) modulesToApply[i]).apply(i, states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Sets the velocity in the X direction, in m/s.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         *
         * @param velocityX Velocity in the X direction, in m/s
         * @return this request
         */
        public RobotCentric withVelocityX(double velocityX) {
            this.VelocityX = velocityX;
            return this;
        }

        /**
         * Sets the velocity in the Y direction, in m/s.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         *
         * @param velocityY Velocity in the Y direction, in m/s
         * @return this request
         */
        public RobotCentric withVelocityY(double velocityY) {
            this.VelocityY = velocityY;
            return this;
        }

        /**
         * The angular rate to rotate at, in radians per second.
         * Angular rate is defined as counterclockwise positive,
         * so this determines how fast to turn counterclockwise.
         *
         * @param rotationalRate Angular rate to rotate at, in radians per second
         * @return this request
         */
        public RobotCentric withRotationalRate(double rotationalRate) {
            this.RotationalRate = rotationalRate;
            return this;
        }

        /**
         * Sets the allowable deadband of the request.
         *
         * @param deadband Allowable deadband of the request
         * @return this request
         */
        public RobotCentric withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }
        /**
         * Sets the rotational deadband of the request.
         *
         * @param rotationalDeadband Rotational deadband of the request
         * @return this request
         */
        public RobotCentric withRotationalDeadband(double rotationalDeadband) {
            this.RotationalDeadband = rotationalDeadband;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public RobotCentric withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }
        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public RobotCentric withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }

    public class ApplyChassisSpeeds extends SwerveRequest.ApplyChassisSpeeds {
        @Override
        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            var states = parameters.kinematics.toSwerveModuleStates(Speeds, CenterOfRotation);
            for (int i = 0; i < modulesToApply.length; ++i) {
                ((ThunderSwerveModule) modulesToApply[i]).apply(i, states[i], DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Sets the chassis speeds to apply to the drivetrain.
         *
         * @param speeds Chassis speeds to apply to the drivetrain
         * @return this request
         */
        public ApplyChassisSpeeds withSpeeds(ChassisSpeeds speeds) {
            this.Speeds = speeds;
            return this;
        }
        /**
         * Sets the center of rotation to rotate around.
         *
         * @param centerOfRotation Center of rotation to rotate around
         * @return this request
         */
        public ApplyChassisSpeeds withCenterOfRotation(Translation2d centerOfRotation) {
            this.CenterOfRotation = centerOfRotation;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive motor
         * @return this request
         */
        public ApplyChassisSpeeds withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }
        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer motor
         * @return this request
         */
        public ApplyChassisSpeeds withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }
}
