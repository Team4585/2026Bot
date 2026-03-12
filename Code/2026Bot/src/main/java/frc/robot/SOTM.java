package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
    //credit to 5962 for class
    public class SOTM{
        public record LaunchParameters(
    double rpm,
    double timeOfFlightSec,
    Rotation2d driveAngle,
    double driveAngularVelocityRadPerSec,
    boolean isValid,
    double confidence,
    double solvedDistanceM,
    int iterationsUsed,
    boolean warmStartUsed) {

    public static final LaunchParameters INVALID =
        new LaunchParameters(0, 0, new Rotation2d(), 0, false, 0, 0, 0, false);
}

public record ShotInputs(
    Pose2d robotPose,
    ChassisSpeeds fieldVelocity,
    ChassisSpeeds robotVelocity,
    Translation2d hubCenter,
    Translation2d hubForward,
    double visionConfidence,
    double pitchDeg,
    double rollDeg) {

    public ShotInputs(
        Pose2d robotPose,
        ChassisSpeeds fieldVelocity,
        ChassisSpeeds robotVelocity,
        Translation2d hubCenter,
        Translation2d hubForward,
        double visionConfidence) {
    this(robotPose, fieldVelocity, robotVelocity, hubCenter, hubForward, visionConfidence, 0.0, 0.0);
    }
}

public static class Config {
    public double launcherOffsetX = 0.20; 
    public double launcherOffsetY = 0.0;  

    public double minScoringDistance = 0.5;
    public double maxScoringDistance = 5.0;

    public int maxIterations = 25;
    public double convergenceTolerance = 0.001; 
    public double tofMin = 0.05;
    public double tofMax = 5.0;

    public double minSOTMSpeed = 0.1;
    public double maxSOTMSpeed = 3.0;

    public double phaseDelayMs = 30.0;  
    public double mechLatencyMs = 20.0; 

    public double sotmDragCoeff = 0.47;

    public double wConvergence = 1.0;
    public double wVelocityStability = 0.8;
    public double wVisionConfidence = 1.2;
    public double wHeadingAccuracy = 1.5;
    public double wDistanceInRange = 0.5;
    public double headingMaxErrorRad = Math.toRadians(15);

    public double headingSpeedScalar = 1.0;
    public double headingReferenceDistance = 2.5; 
    public double maxTiltDeg = 5.0;
}

private final Config config;
private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
private final InterpolatingDoubleTreeMap correctionRpmMap = new InterpolatingDoubleTreeMap();
private final InterpolatingDoubleTreeMap correctionTofMap = new InterpolatingDoubleTreeMap();

private double rpmOffset = 0;
private double previousTOF = -1;
private double previousSpeed = 0;
private double prevRobotVx = 0;
private double prevRobotVy = 0;
private double prevRobotOmega = 0;

public SOTM(Config config) {
    this.config = config;
}

public SOTM() {
    this(new Config());
}

public void loadLUTEntry(double distanceM, double rpm, double tof) {
    rpmMap.put(distanceM, rpm);
    tofMap.put(distanceM, tof);
}

double effectiveRPM(double distance) {
    double base = rpmMap.get(distance);
    Double correction = correctionRpmMap.get(distance);
    return base + (correction != null ? correction : 0.0) + rpmOffset;
}

double effectiveTOF(double distance) {
    double base = tofMap.get(distance);
    Double correction = correctionTofMap.get(distance);
    return base + (correction != null ? correction : 0.0);
}

private double dragCompensatedTOF(double tof) {
    double c = config.sotmDragCoeff;
    if (c < 1e-6) return tof; 
    return (1.0 - Math.exp(-c * tof)) / c;
}

private static final double DERIV_H = 0.01; 

double tofMapDerivative(double d) {
    double tHigh = effectiveTOF(d + DERIV_H);
    double tLow = effectiveTOF(d - DERIV_H);
    return (tHigh - tLow) / (2.0 * DERIV_H);
}

public LaunchParameters calculate(ShotInputs inputs) {
    if (inputs == null || inputs.robotPose() == null
        || inputs.fieldVelocity() == null || inputs.robotVelocity() == null) {
    return LaunchParameters.INVALID;
    }

    Pose2d rawPose = inputs.robotPose();
    ChassisSpeeds fieldVel = inputs.fieldVelocity();
    ChassisSpeeds robotVel = inputs.robotVelocity();

    double poseX = rawPose.getX();
    double poseY = rawPose.getY();
    if (Double.isNaN(poseX) || Double.isNaN(poseY)
        || Double.isInfinite(poseX) || Double.isInfinite(poseY)) {
    return LaunchParameters.INVALID;
    }

    double dt = config.phaseDelayMs / 1000.0;
    double ax = (robotVel.vxMetersPerSecond - prevRobotVx) / 0.02;
    double ay = (robotVel.vyMetersPerSecond - prevRobotVy) / 0.02;
    double aOmega = (robotVel.omegaRadiansPerSecond - prevRobotOmega) / 0.02;
    Pose2d compensatedPose =
        rawPose.exp(
            new Twist2d(
                robotVel.vxMetersPerSecond * dt + 0.5 * ax * dt * dt,
                robotVel.vyMetersPerSecond * dt + 0.5 * ay * dt * dt,
                robotVel.omegaRadiansPerSecond * dt + 0.5 * aOmega * dt * dt));
    prevRobotVx = robotVel.vxMetersPerSecond;
    prevRobotVy = robotVel.vyMetersPerSecond;
    prevRobotOmega = robotVel.omegaRadiansPerSecond;

    double robotX = compensatedPose.getX();
    double robotY = compensatedPose.getY();
    double heading = compensatedPose.getRotation().getRadians();

    Translation2d hubCenter = inputs.hubCenter();
    double hubX = hubCenter.getX();
    double hubY = hubCenter.getY();

    Translation2d hubForward = inputs.hubForward();
    double dot = (hubX - robotX) * hubForward.getX() + (hubY - robotY) * hubForward.getY();
    if (dot < 0) {
    return LaunchParameters.INVALID;
    }

    if (Math.abs(inputs.pitchDeg()) > config.maxTiltDeg
        || Math.abs(inputs.rollDeg()) > config.maxTiltDeg) {
    return LaunchParameters.INVALID;
    }

    double cosH = Math.cos(heading);
    double sinH = Math.sin(heading);
    double launcherX = robotX + config.launcherOffsetX * cosH - config.launcherOffsetY * sinH;
    double launcherY = robotY + config.launcherOffsetX * sinH + config.launcherOffsetY * cosH;

    double launcherFieldOffX = config.launcherOffsetX * cosH - config.launcherOffsetY * sinH;
    double launcherFieldOffY = config.launcherOffsetX * sinH + config.launcherOffsetY * cosH;
    double omega = fieldVel.omegaRadiansPerSecond;
    double vx = fieldVel.vxMetersPerSecond + (-launcherFieldOffY) * omega;
    double vy = fieldVel.vyMetersPerSecond + launcherFieldOffX * omega;

    double rx = hubX - launcherX;
    double ry = hubY - launcherY;
    double distance = Math.hypot(rx, ry);

    if (distance < config.minScoringDistance || distance > config.maxScoringDistance) {
    return LaunchParameters.INVALID;
    }

    double robotSpeed = Math.hypot(vx, vy);

    if (robotSpeed > config.maxSOTMSpeed) {
    return LaunchParameters.INVALID;
    }

    boolean velocityFiltered = robotSpeed < config.minSOTMSpeed;

    double solvedTOF;
    double projDist;
    int iterationsUsed;
    boolean warmStartUsed;

    if (velocityFiltered) {
    solvedTOF = effectiveTOF(distance);
    projDist = distance;
    iterationsUsed = 0;
    warmStartUsed = false;
    } else {
    int maxIter = config.maxIterations;
    double convTol = config.convergenceTolerance;

    double tof;
    if (previousTOF > 0) {
        tof = previousTOF;
        warmStartUsed = true;
    } else {
        tof = effectiveTOF(distance);
        warmStartUsed = false;
    }

    projDist = distance;
    iterationsUsed = 0;

    for (int i = 0; i < maxIter; i++) {
        double prevTOF = tof;
        double driftTOF = dragCompensatedTOF(tof);
        double prx = rx - vx * driftTOF;
        double pry = ry - vy * driftTOF;
        projDist = Math.hypot(prx, pry);

        if (projDist < 0.01) {
        tof = effectiveTOF(distance);
        iterationsUsed = maxIter + 1; 
        break;
        }

        double lookupTOF = effectiveTOF(projDist);

        double dPrime = -(prx * vx + pry * vy) / projDist;
        double gPrime = tofMapDerivative(projDist);
        double f = lookupTOF - tof;
        double fPrime = gPrime * dPrime - 1.0;

        if (Math.abs(fPrime) > 0.01) {
        tof = tof - f / fPrime;
        } else {
        tof = lookupTOF; 
        }

        tof = MathUtil.clamp(tof, config.tofMin, config.tofMax);
        iterationsUsed = i + 1;

        if (Math.abs(tof - prevTOF) < convTol) {
        break;
        }
    }

    if (tof > config.tofMax || tof < 0.0 || Double.isNaN(tof)) {
        tof = effectiveTOF(distance);
        iterationsUsed = maxIter + 1;
    }

    solvedTOF = tof;
    }

    previousTOF = solvedTOF;

    double effectiveTOF = solvedTOF + config.mechLatencyMs / 1000.0;
    double effectiveRPMValue = effectiveRPM(projDist);

    double compTargetX;
    double compTargetY;
    if (velocityFiltered) {
    compTargetX = hubX;
    compTargetY = hubY;
    } else {
    double headingDriftTOF = dragCompensatedTOF(solvedTOF);
    compTargetX = hubX - vx * headingDriftTOF;
    compTargetY = hubY - vy * headingDriftTOF;
    }
    double aimX = compTargetX - robotX;
    double aimY = compTargetY - robotY;
    Rotation2d driveAngle = new Rotation2d(aimX, aimY);

    double headingErrorRad = MathUtil.angleModulus(driveAngle.getRadians() - heading);

    double driveAngularVelocity = 0;
    if (!velocityFiltered && distance > 0.1) {
    double tangentialVel = (-ry * vx + rx * vy) / distance;
    driveAngularVelocity = tangentialVel / distance;
    }

    double solverQuality;
    if (velocityFiltered) {
    solverQuality = 1.0;
    } else {
    int maxIter = config.maxIterations;
    if (iterationsUsed > maxIter) {
        solverQuality = 0.0;
    } else if (iterationsUsed <= 3) {
        solverQuality = 1.0;
    } else {
        solverQuality = MathUtil.interpolate(1.0, 0.1, (double) (iterationsUsed - 3) / (maxIter - 3));
    }
    }

    double confidence = computeConfidence(
        solverQuality, robotSpeed, headingErrorRad, distance, inputs.visionConfidence());

    previousSpeed = robotSpeed;

    return new LaunchParameters(
        effectiveRPMValue,
        effectiveTOF,
        driveAngle,
        driveAngularVelocity,
        true,
        confidence,
        distance,
        iterationsUsed,
        warmStartUsed);
}

private double computeConfidence(
    double solverQuality, double currentSpeed, double headingErrorRad,
    double distance, double visionConfidence) {

    double convergenceQuality = solverQuality;
    double speedDelta = Math.abs(currentSpeed - previousSpeed);
    double velocityStability = MathUtil.clamp(1.0 - speedDelta / 0.5, 0, 1);
    double visionConf = MathUtil.clamp(visionConfidence, 0, 1);

    double distanceScale = MathUtil.clamp(config.headingReferenceDistance / distance, 0.5, 2.0);
    double speedScale = 1.0 / (1.0 + config.headingSpeedScalar * currentSpeed);
    double scaledMaxError = config.headingMaxErrorRad * distanceScale * speedScale;
    double headingErr = Math.abs(headingErrorRad);
    double headingAccuracy = MathUtil.clamp(1.0 - headingErr / scaledMaxError, 0, 1);

    double rangeSpan = config.maxScoringDistance - config.minScoringDistance;
    double rangeFraction = (distance - config.minScoringDistance) / rangeSpan;
    double distInRange = 1.0 - 2.0 * Math.abs(rangeFraction - 0.5);
    distInRange = MathUtil.clamp(distInRange, 0, 1);

    double[] c = {convergenceQuality, velocityStability, visionConf, headingAccuracy, distInRange};
    double[] w = {
    config.wConvergence,
    config.wVelocityStability,
    config.wVisionConfidence,
    config.wHeadingAccuracy,
    config.wDistanceInRange
    };

    double sumW = 0;
    double logSum = 0;
    for (int i = 0; i < 5; i++) {
    if (c[i] <= 0) return 0;
    logSum += w[i] * Math.log(c[i]);
    sumW += w[i];
    }

    if (sumW <= 0) return 0;
    double composite = Math.exp(logSum / sumW) * 100.0;
    return MathUtil.clamp(composite, 0, 100);
}

public void addRpmCorrection(double distance, double deltaRpm) {
    correctionRpmMap.put(distance, deltaRpm);
}

public void addTofCorrection(double distance, double deltaTof) {
    correctionTofMap.put(distance, deltaTof);
}

public void clearCorrections() {
    correctionRpmMap.clear();
    correctionTofMap.clear();
}

public void adjustOffset(double delta) {
    rpmOffset = MathUtil.clamp(rpmOffset + delta, -200, 200);
}

public void resetOffset() {
    rpmOffset = 0;
}

public double getOffset() {
    return rpmOffset;
}

public double getTimeOfFlight(double distanceM) {
    return effectiveTOF(distanceM);
}

public double getBaseRPM(double distance) {
    return rpmMap.get(distance);
}

public void resetWarmStart() {
    previousTOF = -1;
    previousSpeed = 0;
    prevRobotVx = 0;
    prevRobotVy = 0;
    prevRobotOmega = 0;
}

InterpolatingDoubleTreeMap getRpmMap() {
    return rpmMap;
}

InterpolatingDoubleTreeMap getTofMap() {
    return tofMap;
}
}