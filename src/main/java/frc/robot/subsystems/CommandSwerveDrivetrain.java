package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // private final Field2d field2d = new Field2d();
    // ShuffleboardTab tab = Shuffleboard.getTab("Field Map");

    //private CANrange armCANrange = new CANrange(51, "rio");
    private double ARM_RANGE_L4 = 0.30;
    private double ARM_RANGE_LOWER = 0.4;
    private LinearFilter m_filter = LinearFilter.movingAverage(5);
    public double armRangeFiltered = 0;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    public enum LimelightUpdateMode {
        front,
        back,
        both,
        none
    };

    private LimelightUpdateMode currentLimelightUpdateMode = LimelightUpdateMode.front;

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineRotation;
    // private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
    // //Default setting

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();

        // SmartDashboard.putData(field2d);
        // tab.add("Field", field2d).withPosition(0, 0).withSize(10, 5);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPosewithLimelight, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(10, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(9, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * A method that applies the specified control request to this swerve
     * drivetrain.
     * Does not require the above command
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public void applyRequestMethod(Supplier<SwerveRequest> requestSupplier) {
        this.setControl(requestSupplier.get());
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // if (DriverStation.isEnabled()) {
            if (currentLimelightUpdateMode == LimelightUpdateMode.both){
                updateOdometryFromLL_CTRE("limelight-front");
                updateOdometryFromLL_CTRE("limelight-rear");
            } else if (currentLimelightUpdateMode == LimelightUpdateMode.front) {
                updateOdometryFromLL_CTRE("limelight-front");
            } else if (currentLimelightUpdateMode == LimelightUpdateMode.back) {
                updateOdometryFromLL_CTRE("limelight-rear");
            }
        // }

        // field2d.setRobotPose(getState().Pose);
        // SmartDashboard.putNumber("Front Distance", frontCANrange.getDistance().getValueAsDouble());
        // armRangeFiltered = m_filter.calculate(armCANrange.getDistance().getValueAsDouble()); 
        // SmartDashboard.putNumber("Arm Range", armRangeFiltered);   
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public LimelightUpdateMode getCurrentLimelightUpdateMode() {
        return currentLimelightUpdateMode;
    }

    public void setLimelightUpdateMode(LimelightUpdateMode mode) {
        currentLimelightUpdateMode = mode;
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    /** Get Turn Rate */
    public double getTurnRate() {
        return Math.toDegrees(getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
    }

    /*
     * Sampe code from Limelight
     * Updates the field relative position of the robot.
     */
    public void updateOdometryFromLL(String limelightName) {
        boolean useMegaTag2 = true; // set to false to use MegaTag1

        boolean doRejectUpdate = false;
        // String limelightName = "limelight-front";
        if (useMegaTag2 == false) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if (mt1.rawFiducials[0].ambiguity > .7) {
                    doRejectUpdate = true;
                }
                if (mt1.rawFiducials[0].distToCamera > 3) {
                    doRejectUpdate = true;
                }
            }
            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate) {
                super.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                addVisionMeasurement(
                        mt1.pose,
                        mt1.timestampSeconds);
            }
        } else if (useMegaTag2 == true) {

            LimelightHelpers.SetRobotOrientation(limelightName,
                    getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
            if (Math.abs(getTurnRate()) > 360) // if our angular velocity is greater than 360 degrees per second,
                                               // ignore vision updates
            {
                doRejectUpdate = true;
            }
            if (mt2.tagCount == 0) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate && mt2.avgTagDist < 2) {
                super.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds);
            }
        }
    }

    /*
     * Sample code from CTRE
     * This example of adding Limelight is very simple and may not be sufficient for
     * on-field use.
     * Users typically need to provide a standard deviation that scales with the
     * distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible,
     * though exact implementation
     * of how to use vision should be tuned per-robot and to the team's
     * specification.
     */
    public PoseEstimate updateOdometryFromLL_CTRE(String limelightName) {
        double distance;
        var driveState = getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation(limelightName, headingDeg, 0, 0, 0, 0, 0);
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if(llMeasurement != null){
            distance = getState().Pose.getTranslation().getDistance(llMeasurement.pose.getTranslation());
        } else {
            distance = 1000;
        }
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0
                && llMeasurement.avgTagDist < 3 && distance < 2) {
            // super.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds, VecBuilder.fill(.7, .7, 9999999));
        }

        return llMeasurement;
    }

    /** Force Pose Update from Limelight */
    public void forcePoseUpdate(String limelightName) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        resetPose(mt1.pose);
    }

    /** Standby Limelight IMU Updates */
    public void standyLimelightUpdate(String limelightName) {
        LimelightHelpers.SetIMUMode(limelightName, 1);
        LimelightHelpers.SetRobotOrientation(limelightName,
                getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

    }

    /** Standby Limelight IMU Updates Forced Flip for Auto */
    public void standyLimelightUpdate180(String limelightName) {
        LimelightHelpers.SetIMUMode(limelightName, 1);
        LimelightHelpers.SetRobotOrientation(limelightName,
                180, 0, 0, 0, 0, 0);

    }

    public void standbyLimelightZero(String limelightName) {
        LimelightHelpers.SetIMUMode(limelightName, 1);
        LimelightHelpers.SetRobotOrientation(limelightName,
                0, 0, 0, 0, 0, 0);

    }

    /** Active Limelight IMU nUpdates */
    public void activeLimelightUpdate(String limelightName) {
        LimelightHelpers.SetIMUMode(limelightName, 2);
    }

    public void zeroFieldCentric() {
        seedFieldCentric();
        standbyLimelightZero("limelight-front");
        standbyLimelightZero("limelight-rear");
        activeLimelightUpdate("limelight-front");
        activeLimelightUpdate("limelight-rear");
    }

    public void resetPosewithLimelight(Pose2d pose){
        // Not really sure if this matters? Can the periodic method run while this is running?
        LimelightUpdateMode lastUpdateMode = currentLimelightUpdateMode;
        currentLimelightUpdateMode = LimelightUpdateMode.none;

        resetPose(pose);

        LimelightHelpers.SetIMUMode("limelight-front", 1);
        LimelightHelpers.SetRobotOrientation("limelight-front",
                pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        // LimelightHelpers.SetIMUMode("limelight-front", 2);

        LimelightHelpers.SetIMUMode("limelight-rear", 1);
        LimelightHelpers.SetRobotOrientation("limelight-rear",
                pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        // LimelightHelpers.SetIMUMode("limelight-rear", 2);

        currentLimelightUpdateMode = lastUpdateMode;

    }

    public int getNearestTag(PoseEstimate poseEstimate) {
        RawFiducial[] fiducials = poseEstimate.rawFiducials;
        int tag = 0;
        double tagDistance = 1000;

        for (RawFiducial fiducial : fiducials) {
            if (fiducial != null) {
                if (fiducial.distToRobot < tagDistance) {
                    tagDistance = fiducial.distToRobot;
                    tag = fiducial.id;
                }
            }
        }

        return tag;
    }

    public void setFrontLimelight(){
        currentLimelightUpdateMode = LimelightUpdateMode.front;
    }

    public void setBackLimelight(){
        currentLimelightUpdateMode = LimelightUpdateMode.back;
    }

    public void setBothLimelight(){
        currentLimelightUpdateMode = LimelightUpdateMode.both;
    }

    public void setNoLimelight(){
        currentLimelightUpdateMode = LimelightUpdateMode.none;
    }

    // public double getArmRange(){
    //     return armCANrange.getDistance().getValueAsDouble();
    // }

    public boolean isArmInRangeL4(){
        return armRangeFiltered <= ARM_RANGE_L4;
    }

    public boolean isArmInRangeLower(){
        return armRangeFiltered <= ARM_RANGE_LOWER;
    }

    public double getArmRangeFiltered(){
        return armRangeFiltered;
    }
}
