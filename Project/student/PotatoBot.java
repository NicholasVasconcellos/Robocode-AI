package student;

import robocode.*;
import robocode.util.Utils;
import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.*;

/**
 * PotatoBot - Advanced Robocode Robot (Enhanced)
 * 
 * Improvements:
 * - Distance-based bullet power (high power up close, low power far away)
 * - Retreat mode when low on energy or outgunned
 * - Smarter energy management
 * - Ram detection and counter-ramming logic
 * 
 * @author Nicholas
 */
public class PotatoBot extends TeamRobot {    
    // Physics constants
    private static final double MAX_VELOCITY = 8.0;
    private static final double GUN_TURN_RATE_RADIANS = Math.toRadians(20.0);

    // Robot Dimentions
    private static int robotRadius = 18;

    // Field dimentions (on startup)
    private static double BATTLEFIELD_WIDTH = 0;
    private static double BATTLEFIELD_HEIGHT = 0;
    
    // Bullet physics
    private static final double MIN_BULLET_POWER = 0.1;
    private static final double MAX_BULLET_POWER = 3.0;
    private static final double BULLET_SPEED_BASE = 20.0;
    private static final double BULLET_SPEED_COEFFICIENT = 3.0;

    // Snipe Score Heuristics
    private static double tresholdScore = 0.01;
    private static int maxScanAge = 3;
    
    // Movement parameters
    private static final double wallMargin = 120.0;
    private static final double dangerWallMarging = 60.0;
    private static final double idealDistance = 300.0;
    
    // Targeting parameters
    private static final int maxPredictFrames = 5;
    private static final double minSnipeConfidence = 0.4;
    private static final int maxSnipeConfidence= 50;

    // Ram Detection Parameters
    private static final int ramApproachRate = 6;
    private static final int ramVelocity = 4;

    // State Machine
    private enum MovementState {
        VIBING,      // Evasive movement with bullet dodging
        CHASE,      // Pursuing distant target
        KILL_MODE,  // Rushing low-HP target
        RETREAT     // Running away when outgunned
    }
    
    private MovementState currState = MovementState.VIBING;
    private int moveDirection = 1;
    private long directionChangeTime = 0;
    private double desiredHeading = 0;

    // Kill mode parameters
    private static final double lowEnergy = 25.0;
    
    // Chase mode parameters
    private static final double chaseDistance = 400.0;
    
    // Retreat Mode Parameters
    private static final double retreatEnergyThreshold = 15.0;      // Retreat when our energy drops below this
    private static final double retreatEnergyRatio = 0.3;           // Retreat when our energy is < 30% of enemy's
    private static final double retreatSafeDistance = 450.0;        // Try to maintain this distance when retreating
    private static final double retreatCancelEnergy = 35.0;         // Stop retreating if we recover to this
    private static final double ramDetectionDistance = 150.0;       // Detect rammer if they're this close and approaching
    private long retreatStartTime = 0;
    private boolean isBeingRammed = false;

    // Power Calculation Parameters
    private static final double closeRange = 150.0;    // Max power zone
    private static final double midRange = 300.0;      // Medium power zone
    private static final double longRange = 500.0;     // Low power zone
    
    // Enemy tracking 
    // final = const pointer in cpp
    private final Map<String, EnemyBot> enemies = new HashMap<>();
    private EnemyBot currTarget = null;
    private EnemyBot killTarget = null;
    
    // Bullet tracking for avoidance
    private final List<IncomingBullet> incomingBullets = new ArrayList<>();
    private long lastBulletDodgeTime = 0;
    
    // Randomization
    private Random random = new Random();

    // Global Cached Variables
    private Point2D.Double myPos;
    private double gunHeading;
    private long currentTime;
    public void updateCache() {
        myPos = new Point2D.Double(getX(), getY());
        gunHeading = getGunHeadingRadians();
        currentTime = getTime();
    }
    

    // Enemy Data Structure
    public static class EnemyBot {
        public String name;
        public boolean isAlive = true;
        
        // Current state - Updated every scan
        public double energy;
        public double bearing;
        public double distance;
        public double heading;
        public double velocity;
        public Point2D.Double position;
        public long turnLastSeen;
        
        // Historical data - Last State
        public double prevEnergy = -1;
        public Point2D.Double prevPosition = null;
        public double prevHeading = -1;
        public double prevVelocity = 0;
        
        // Derived motion
        public double acceleration = 0;
        public double angularVelocity = 0;
        public int consecutiveScans = 0;
        
        // Aggression tracking 
        public int ramAttempts = 0;           // Count how often they close distance aggressively
        public double approachRate = 0;        // How fast they're closing in
        
        // Predictions
        public Point2D.Double[] predictedPositions;
        public double[] predictionConfidence;
        public double[] predictedDistances;
        
        // constructor with name
        public EnemyBot(String name) {
            this.name = name;
            this.predictedPositions = new Point2D.Double[maxPredictFrames];
            this.predictionConfidence = new double[maxPredictFrames];
            this.predictedDistances = new double[maxPredictFrames];
            this.consecutiveScans = 0;
        }
        
        // Update with new scan
        public void update(ScannedRobotEvent e, AdvancedRobot robot, long currentTime) {
            double prevDistance = distance;
            
            // Store previous state
            if (position != null) {
                prevPosition = (Point2D.Double) position.clone();
                prevHeading = heading;
                prevVelocity = velocity;
                consecutiveScans++;
            } else {
                consecutiveScans = 1;
            }
            
            if (energy > 0) {
                prevEnergy = energy;
            }
            
            // Update current state
            energy = e.getEnergy();
            bearing = e.getBearing();
            distance = e.getDistance();
            heading = e.getHeading();
            velocity = e.getVelocity();
            turnLastSeen = currentTime;
            
            // Get Global Space coordinates of enemy - reconstruct from distance and segment angle
            // Angle from Robot to Enemy in global coordinate system (absolute)
            double absoluteBearing = robot.getHeadingRadians() + e.getBearingRadians();
            position = new Point2D.Double(
                robot.getX() + distance * Math.sin(absoluteBearing),
                robot.getY() + distance * Math.cos(absoluteBearing)
            );
            
            // Calculate derived motion
            if (consecutiveScans >= 2) {
                acceleration = velocity - prevVelocity;
                double headingChange = Utils.normalRelativeAngle(
                    Math.toRadians(heading) - Math.toRadians(prevHeading)
                );
                angularVelocity = headingChange;
                
                // Track approach Rate
                approachRate = prevDistance - distance; 
                
                        // Track and Store Ramming Behaviour
                        if (approachRate > 5 && distance < ramDetectionDistance) {
                            ramAttempts++;
                        }
                }

                predictNextPositions();
        }
        
        // Generate up to N next positions
        private void predictNextPositions() {
            double px = position.x;
            double py = position.y;
            double h = Math.toRadians(heading);
            double v = velocity;
            
            for (int i = 0; i < predictedPositions.length; i++) {
                h += angularVelocity;
                if (consecutiveScans >= 2) {
                    v += acceleration;
                    // Clamp to max speed if needed
                    v = Math.max(-MAX_VELOCITY, Math.min(MAX_VELOCITY, v));
                }
                px += v * Math.sin(h);
                py += v * Math.cos(h);
                px = Math.max(robotRadius, Math.min(BATTLEFIELD_WIDTH - robotRadius, px));
                py = Math.max(robotRadius, Math.min(BATTLEFIELD_HEIGHT - robotRadius, py));
                predictedPositions[i] = new Point2D.Double(px, py);

                // Get Confidence
                // double baseConfidence = Math.exp(-0.08 * turn);
                // double scanBonus = Math.min(1.0, consecutiveScans / 5.0);
                // double historyFactor = 0.5 + 0.5 * historicalPredictionAccuracy;
                // double velocityFactor = (Math.abs(v) < 2) ? 1.2 : 1.0;
                
                // predictionConfidence[turn] = Math.min(1.0,
                // baseConfidence * scanBonus * historyFactor * velocityFactor);

            }


        }

        private void calcDistances(Point2D.Double startPos){
            for(int i =0; i < predictedPositions.length; i++){
                if(predictedPositions[i] != null){
                    predictedDistances[i] = startPos.distance(predictedPositions[i]);
                }
                else{
                    predictedDistances[i] = Double.MAX_VALUE;
                }
            }

        }

        public boolean isKillTarget() {
            return isAlive && energy > 0 && energy <= lowEnergy;
        }
        
        public boolean isRammer() {
            return ramAttempts >= 3;
        }
        
        public double checkFired() {
            if (prevEnergy > 0 && energy < prevEnergy) {
                double drop = prevEnergy - energy;
                if (drop >= 0.1 && drop <= 3.0) {
                    return drop;
                }
            }
            return -1;
        }
    }
    

    public static class Snipe{
        public double power;
        public int numTurns;
        public EnemyBot enemy;
        public double enemyHP;
        public double confidence;
        public double aimAngle;
        public Point2D.Double targetPos;

        public Snipe(double power, int numTurns, EnemyBot enemy) {
            this.power = power;
            this.numTurns = numTurns;
            this.enemy = enemy;
            this.enemyHP = enemy.energy;
            // this.confidence = confidence;
            // this.aimAngle = aimAngle;
            // this.targetPos = targetPos;
        }

        public float getScore(){
            // Calculates score
            // Turn number is risk, power is reward, so we look at risk rewards to choose best snipe
            float prod = (float)(numTurns * numTurns);
            
            // Favor closer oponents
            return (float)power / prod;
        }
    }
    // ========== INCOMING BULLET TRACKING ==========
    private static class IncomingBullet {
        Point2D.Double origin;
        double heading;
        double speed;
        double power;
        long firedTime;
        
        IncomingBullet(Point2D.Double origin, double heading, double power, long time) {
            this.origin = origin;
            this.heading = heading;
            this.power = power;
            this.speed = BULLET_SPEED_BASE - BULLET_SPEED_COEFFICIENT * power;
            this.firedTime = time;
        }
        
        Point2D.Double getPosition(long currentTime) {
            double travelTime = currentTime - firedTime;
            double distance = speed * travelTime;
            return new Point2D.Double(
                origin.x + distance * Math.sin(heading),
                origin.y + distance * Math.cos(heading)
            );
        }
        
        boolean isActive(long currentTime, double fieldWidth, double fieldHeight) {
            Point2D.Double pos = getPosition(currentTime);
            return pos.x > 0 && pos.x < fieldWidth && 
                   pos.y > 0 && pos.y < fieldHeight;
        }
    }
    
    // ========== MAIN ROBOT LOGIC ==========
    
    @Override
    public void run() {
        // Decoulpe radar and run from robot heading
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);
        
        // Cache battlefield dimensions once at startup
        BATTLEFIELD_WIDTH = getBattleFieldWidth();
        BATTLEFIELD_HEIGHT = getBattleFieldHeight();

        setTurnRadarRight(360);
        
        while (true) {
            cleanupBullets();
            setState();
            
            // Run Current State
            switch (currState) {
                case VIBING:
                    executeVIBINGMovement();
                    break;
                case CHASE:
                    executeChaseMovement();
                    break;
                case KILL_MODE:
                    executeKillMovement();
                    break;
                case RETREAT:
                    executeRetreatMovement();
                    break;
            }
            
            updateTargeting();
            executeRadarStrategy();
            snipeCheck();
            
            execute();
        }
    }
    
    private void cleanupBullets() {
        incomingBullets.removeIf(b -> 
            !b.isActive(getTime(), getBattleFieldWidth(), getBattleFieldHeight()));
    }
    

    // Set State    
    private void setState() {
        // Check for ramming behavior
        isBeingRammed = isTargetRamming();
        
        // Retreat Check
        if (shouldRetreat()) {
            // Set state to retreat
            if (currState != MovementState.RETREAT) {
                retreatStartTime = getTime();
                out.println("RETREATING - Low energy or outgunned!");
            }
            currState = MovementState.RETREAT;
            return;
        }
        
        // Cancel retreat if we've recovered
        if (currState == MovementState.RETREAT && shouldCancelRetreat()) {
            out.println("Ending retreat - recovered enough energy");
            currState = MovementState.VIBING;
        }
        
        // check for weak enemies
        if (getEnergy() > retreatEnergyThreshold) {
            killTarget = findKillTarget();
            if (killTarget != null) {
                currState = MovementState.KILL_MODE;
                return;
            }
        }
        
        // Chase far Target
        if (currTarget != null && currTarget.distance > chaseDistance) {
            currState = MovementState.CHASE;
            return;
        }
        
        // Default: VIBING
        if (currState != MovementState.RETREAT) {
            currState = MovementState.VIBING;
        }
    }
    
    // Retreat Check conditions
    private boolean shouldRetreat() {
        double myEnergy = getEnergy();
        
        // Always retreat if critically low
        if (myEnergy < retreatEnergyThreshold) {
            return true;
        }
        
        // Check if outgunned by current target
        if (currTarget != null && currTarget.isAlive) {
            double energyRatio = myEnergy / Math.max(1, currTarget.energy);
            
            // Retreat if we have significantly less energy AND they're close
            if (energyRatio < retreatEnergyRatio && currTarget.distance < 300) {
                return true;
            }
            
            // Retreat from aggressive rammers when we're damaged
            if (currTarget.isRammer() && myEnergy < 40 && currTarget.distance < 200) {
                return true;
            }
        }
        
        // Don't retreat if we're being rammed - fight back instead
        if (isBeingRammed && myEnergy > 10) {
            return false;
        }
        
        return false;
    }
    
    private boolean shouldCancelRetreat() {
        double myEnergy = getEnergy();
        
        // Cancel if we've recovered
        if (myEnergy >= retreatCancelEnergy) {
            return true;
        }
        
        // Cancel if all enemies are weaker than us now
        boolean allWeaker = true;
        for (EnemyBot enemy : enemies.values()) {
            if (enemy.isAlive && enemy.energy > myEnergy) {
                allWeaker = false;
                break;
            }
        }
        if (allWeaker && myEnergy > 20) {
            return true;
        }
        
        // Cancel if retreated for too long (60 turns max)
        if (getTime() - retreatStartTime > 60) {
            return true;
        }
        
        return false;
    }
    
    private boolean isTargetRamming() {
        if (currTarget == null) return false;
        
         return currTarget.distance < ramDetectionDistance && 
             currTarget.approachRate > ramApproachRate && // 6
               currTarget.velocity > ramVelocity; // 4
    }
    
    // Retreat.Run()
    private void executeRetreatMovement() {
        setMaxVelocity(MAX_VELOCITY);
        // Color for RETREAT state: blue
        setColors(Color.BLACK, Color.BLACK, Color.BLACK);
        
        if (currTarget == null || currTarget.position == null) {
            executeRandomMovement();
            return;
        }
        
        // Move AWAY from the threat
        double awayAngle = Math.atan2(
            getX() - currTarget.position.x,
            getY() - currTarget.position.y
        );
        
        // Add some perpendicular movement to avoid being predictable
        double retreatAngle = awayAngle + Math.toRadians(30 * moveDirection);
        
        // Strong wall avoidance during retreat
        retreatAngle = applyWallAvoidance(retreatAngle);
        
        // If we'd hit a wall, curve around instead of stopping
        double futureX = getX() + 150 * Math.sin(retreatAngle);
        double futureY = getY() + 150 * Math.cos(retreatAngle);
        if (!isPositionSafe(futureX, futureY)) {
            // Try curving the other direction
            retreatAngle = awayAngle + Math.toRadians(-30 * moveDirection);
            retreatAngle = applyWallAvoidance(retreatAngle);
        }
        
        double turnNeeded = Utils.normalRelativeAngle(retreatAngle - getHeadingRadians());
        
        if (Math.abs(turnNeeded) < Math.PI / 2) {
            setTurnRightRadians(turnNeeded);
            setAhead(150);
        } else {
            setTurnRightRadians(Utils.normalRelativeAngle(turnNeeded + Math.PI));
            setBack(150);
        }
        
        // Change direction occasionally
        if (getTime() - directionChangeTime > 15 + random.nextInt(15)) {
            directionChangeTime = getTime();
            moveDirection *= -1;
        }
        
        out.println("RETREAT: dist=" + (int)currTarget.distance + 
            " myHP=" + (int)getEnergy() + " theirHP=" + (int)currTarget.energy);
    }
    

    // Vibing.run()
    private void executeVIBINGMovement() {
        
        setMaxVelocity(MAX_VELOCITY);
        // Color for VIBING state: green
        setColors(Color.GREEN, Color.GREEN, Color.GREEN);
        
        IncomingBullet dangerBullet = findMostDangerousBullet();
        if (dangerBullet != null && getTime() - lastBulletDodgeTime > 5) {
            executeBulletDodge(dangerBullet);
            return;
        }
        
        if (currTarget != null) {
            executeStrafeMovement();
        } else {
            executeRandomMovement();
        }
    }
    
    private IncomingBullet findMostDangerousBullet() {
        IncomingBullet mostDangerous = null;
        double closestApproach = Double.MAX_VALUE;
        
        for (IncomingBullet bullet : incomingBullets) {
            Point2D.Double bulletPos = bullet.getPosition(getTime());
            
            double dx = getX() - bulletPos.x;
            double dy = getY() - bulletPos.y;
            
            double bvx = Math.sin(bullet.heading);
            double bvy = Math.cos(bullet.heading);
            
            double dot = dx * bvx + dy * bvy;
            
            if (dot > 0) {
                double perpDist = Math.abs(dx * bvy - dy * bvx);
                double timeToClosest = dot / bullet.speed;
                
                if (perpDist < 50 && timeToClosest < 15 && timeToClosest > 0) {
                    if (perpDist < closestApproach) {
                        closestApproach = perpDist;
                        mostDangerous = bullet;
                    }
                }
            }
        }
        
        return mostDangerous;
    }
    
    private void executeBulletDodge(IncomingBullet bullet) {
        lastBulletDodgeTime = getTime();
        
        double perpLeft = bullet.heading + Math.PI / 2;
        double perpRight = bullet.heading - Math.PI / 2;
        
        double leftX = getX() + 100 * Math.sin(perpLeft);
        double leftY = getY() + 100 * Math.cos(perpLeft);
        double rightX = getX() + 100 * Math.sin(perpRight);
        double rightY = getY() + 100 * Math.cos(perpRight);
        
        boolean leftSafe = isPositionSafe(leftX, leftY);
        boolean rightSafe = isPositionSafe(rightX, rightY);
        
        double dodgeAngle;
        if (leftSafe && !rightSafe) {
            dodgeAngle = perpLeft;
        } else if (rightSafe && !leftSafe) {
            dodgeAngle = perpRight;
        } else {
            dodgeAngle = (moveDirection > 0) ? perpLeft : perpRight;
        }
        
        double turnNeeded = Utils.normalRelativeAngle(dodgeAngle - getHeadingRadians());
        
        if (Math.abs(turnNeeded) < Math.PI / 2) {
            setTurnRightRadians(turnNeeded);
            setAhead(80);
        } else {
            setTurnRightRadians(Utils.normalRelativeAngle(turnNeeded + Math.PI));
            setBack(80);
        }
    }
    
    private void executeStrafeMovement() {
        double absoluteBearing = Math.atan2(
            currTarget.position.x - getX(),
            currTarget.position.y - getY()
        );
        
        double strafeAngle = absoluteBearing + (Math.PI / 2) * moveDirection;
        
        double distanceError = currTarget.distance - idealDistance;
        double approachComponent = Math.toRadians(distanceError / 10);
        strafeAngle = Utils.normalRelativeAngle(strafeAngle - approachComponent * moveDirection);
        
        strafeAngle = applyWallAvoidance(strafeAngle);
        
        double turnNeeded = Utils.normalRelativeAngle(strafeAngle - getHeadingRadians());
        
        if (Math.abs(turnNeeded) < Math.PI / 2) {
            setTurnRightRadians(turnNeeded);
            setAhead(100);
        } else {
            setTurnRightRadians(Utils.normalRelativeAngle(turnNeeded + Math.PI));
            setBack(100);
        }
        
        if (getTime() - directionChangeTime > 20 + random.nextInt(30)) {
            directionChangeTime = getTime();
            moveDirection *= -1;
        }
    }
    
    private void executeRandomMovement() {
        double randomAngle = getHeadingRadians() + Math.toRadians(random.nextGaussian() * 20);
        randomAngle = applyWallAvoidance(randomAngle);
        
        double turnNeeded = Utils.normalRelativeAngle(randomAngle - getHeadingRadians());
        setTurnRightRadians(turnNeeded * 0.5);
        setAhead(100);
        
        if (getTime() - directionChangeTime > 30) {
            directionChangeTime = getTime();
            moveDirection *= -1;
        }
    }
    
    // Chase.run()
    
    private void executeChaseMovement() {
        // Color for CHASE state: yellow
        setColors(Color.YELLOW, Color.YELLOW, Color.YELLOW);

        if (currTarget == null || currTarget.position == null) {
            currState = MovementState.VIBING;
            return;
        }
        
        setMaxVelocity(MAX_VELOCITY);
        
        Point2D.Double targetPos = currTarget.position;
        if (currTarget.predictedPositions[2] != null) {
            targetPos = currTarget.predictedPositions[2];
        }
        
        double absoluteBearing = Math.atan2(
            targetPos.x - getX(),
            targetPos.y - getY()
        );
        
        double approachAngle = absoluteBearing + Math.toRadians(20 * moveDirection);
        approachAngle = applyWallAvoidance(approachAngle);
        
        double turnNeeded = Utils.normalRelativeAngle(approachAngle - getHeadingRadians());
        
        if (Math.abs(turnNeeded) < Math.PI / 2) {
            setTurnRightRadians(turnNeeded);
            setAhead(currTarget.distance);
        } else {
            setTurnRightRadians(Utils.normalRelativeAngle(turnNeeded + Math.PI));
            setBack(currTarget.distance);
        }
    }
    
    // KillMode.run()
    private void executeKillMovement() {
        // Color for KILL_MODE state: red
        setColors(Color.RED, Color.RED, Color.RED);

        if (killTarget == null || !killTarget.isAlive) {
            currState = MovementState.VIBING;
            return;
        }
        
        setMaxVelocity(MAX_VELOCITY);
        
        Point2D.Double targetPos = killTarget.position;
        if (killTarget.predictedPositions[1] != null) {
            targetPos = killTarget.predictedPositions[1];
        }
        
        double absoluteBearing = Math.atan2(
            targetPos.x - getX(),
            targetPos.y - getY()
        );
        
        absoluteBearing = applyWallAvoidance(absoluteBearing);
        
        double turnNeeded = Utils.normalRelativeAngle(absoluteBearing - getHeadingRadians());
        
        if (Math.abs(turnNeeded) < Math.PI / 2) {
            setTurnRightRadians(turnNeeded);
            setAhead(killTarget.distance + 50);
        } else {
            setTurnRightRadians(Utils.normalRelativeAngle(turnNeeded + Math.PI));
            setBack(killTarget.distance + 50);
        }
    }
    
    // ========== WALL AVOIDANCE ==========
    
    private double applyWallAvoidance(double desiredHeading) {
        double x = getX();
        double y = getY();
        double fieldWidth = getBattleFieldWidth();
        double fieldHeight = getBattleFieldHeight();
        
        double futureX = x + 120 * Math.sin(desiredHeading);
        double futureY = y + 120 * Math.cos(desiredHeading);
        
        boolean hitLeft = futureX < wallMargin;
        boolean hitRight = futureX > fieldWidth - wallMargin;
        boolean hitBottom = futureY < wallMargin;
        boolean hitTop = futureY > fieldHeight - wallMargin;
        
        if (!hitLeft && !hitRight && !hitBottom && !hitTop) {
            return desiredHeading;
        }
        
        double centerX = fieldWidth / 2;
        double centerY = fieldHeight / 2;
        double toCenterAngle = Math.atan2(centerX - x, centerY - y);
        
        double minDist = Math.min(
            Math.min(x, fieldWidth - x),
            Math.min(y, fieldHeight - y)
        );
        
        double blendFactor = 1.0 - (minDist / wallMargin);
        blendFactor = Math.max(0, Math.min(1, blendFactor));
        
        if (minDist < dangerWallMarging) {
            blendFactor = 1.0;
        }
        
        double diff = Utils.normalRelativeAngle(toCenterAngle - desiredHeading);
        return Utils.normalRelativeAngle(desiredHeading + diff * blendFactor);
    }
    
    private boolean isPositionSafe(double x, double y) {
        return x > wallMargin && x < getBattleFieldWidth() - wallMargin &&
               y > wallMargin && y < getBattleFieldHeight() - wallMargin;
    }
    
    // ========== RADAR STRATEGY ==========
    
    private void executeRadarStrategy() {
        // Always locked on same starget until lost
        if (currTarget != null && getTime() - currTarget.turnLastSeen < 5) {
            narrowRadarLock(currTarget);
        } else {
            setTurnRadarRight(360);
        }
    }
    
    private void narrowRadarLock(EnemyBot target) {
        double absoluteBearing = Math.atan2(
            target.position.x - getX(),
            target.position.y - getY()
        );
        double radarTurn = Utils.normalRelativeAngle(
            absoluteBearing - getRadarHeadingRadians()
        );
        setTurnRadarRightRadians(2.0 * radarTurn);
    }
    
    // targeting system
    
    private void updateTargeting() {
        EnemyBot prevTarget = currTarget;
        currTarget = null;
        double bestScore = Double.MIN_VALUE;
        
        for (EnemyBot enemy : enemies.values()) {
            if (!enemy.isAlive || getTime() - enemy.turnLastSeen > 10) {
                continue;
            }
            
            double distanceScore = 100000 / (enemy.distance * enemy.distance + 100);
            double energyScore = (100 - enemy.energy) * 0.5;
            
            double approachBonus = 0;
            if (enemy.velocity != 0 && enemy.position != null) {
                double enemyHeadingRad = Math.toRadians(enemy.heading);
                double toUsAngle = Math.atan2(getX() - enemy.position.x, getY() - enemy.position.y);
                double angleDiff = Math.abs(Utils.normalRelativeAngle(enemyHeadingRad - toUsAngle));
                
                if (angleDiff < Math.PI / 3 && enemy.velocity > 0) {
                    approachBonus = 50 * (1 - angleDiff / (Math.PI / 3));
                }
            }
            
            double stalePenalty = (getTime() - enemy.turnLastSeen) * 5;
            
            double score = distanceScore + energyScore + approachBonus - stalePenalty;
            
            if (score > bestScore) {
                bestScore = score;
                currTarget = enemy;
            }
        }
        
        if (currTarget != null && (prevTarget == null || 
            !currTarget.name.equals(prevTarget.name))) {
            out.println("TARGET: " + currTarget.name + 
                " (dist=" + (int)currTarget.distance + ", hp=" + (int)currTarget.energy + ")");
        }
    }
    
    // Power Calculation
    private double calculateOptimalPower(double distance) {
        double myEnergy = getEnergy();
        
        // If retreating, use minimum power to conserve energy
        if (currState == MovementState.RETREAT) {
            return 0.5;
        }
        
        // Energy conservation - don't shoot ourselves to death
        if (myEnergy < 5) {
            return 0.1;  // Bare minimum
        }
        if (myEnergy < 15) {
            return 0.5;  // Very conservative
        }
        
        // Distance-based power scaling
        double basePower;
        
        if (distance < closeRange) {
            // CLOSE RANGE: Maximum power! High hit probability, maximum damage
            basePower = 3.0;
        } else if (distance < midRange) {
            // MID RANGE: Scale linearly from 2.5 to 1.5
            double ratio = (distance - closeRange) / (midRange - closeRange);
            basePower = 2.5 - ratio * 1.0;
        } else if (distance < longRange) {
            // LONG RANGE: Scale from 1.5 to 0.5
            double ratio = (distance - midRange) / (longRange - midRange);
            basePower = 1.5 - ratio * 1.0;
        } else {
            // VERY LONG RANGE: Minimum power, just for tracking/harassment
            basePower = 0.5;
        }
        
        // Adjust for kill mode - boost power to finish them off
        if (currState == MovementState.KILL_MODE && killTarget != null) {
            // Use enough power to kill, but not wastefully more
            double killPower = killTarget.energy / 4.0 + 0.1;  // 4 damage per power
            basePower = Math.max(basePower, Math.min(3.0, killPower));
        }
        
        // Enemy-specific adjustments
        if (currTarget != null) {
            // Against rammers, use high power when they're close
            if (currTarget.isRammer() && distance < 200) {
                basePower = Math.max(basePower, 2.5);
            }
            
            // If enemy is nearly dead, ensure we can kill them
            if (currTarget.energy < basePower * 4 && currTarget.energy > 0) {
                basePower = Math.min(3.0, currTarget.energy / 4.0 + 0.2);
            }
        }
        
        // Final energy check - never fire more than 1/3 of our remaining energy
        basePower = Math.min(basePower, myEnergy / 3.0);
        
        // Clamp to valid range
        return Math.max(0.1, Math.min(3.0, basePower));
    }


    // ========== SNIPING SYSTEM ==========

    private static double getBulletSpeed(double power){
        return 20.0 - 3.0 * power;
    }

    // Check what power is needed to hit target at distance in n turns
    private static Double getPowerForSnipe(double distance, int numTurns){
        if(numTurns <= 0){ return null; }

        double speed = distance / numTurns;
        double power = (20.0 - speed) / 3.0;

        if(power < MIN_BULLET_POWER || power > MAX_BULLET_POWER){
            return null;
        }

        return Math.round(power * 10) / 10.0;
    }

    private int gunTurnTime(double targetAngle){
        double turnNeeded = Math.abs(Utils.normalRelativeAngle(targetAngle - getGunHeadingRadians()));
        double gunTurnRate = Math.toRadians(20);  // 20 deg/turn
        return (int) Math.ceil(turnNeeded / gunTurnRate);
    }

    private boolean isTurnPossible(double distance, int turn) {
        // Minimum time: fastest bullet (power 0.1, speed 19.7)
        double minTime = distance / 19.7;
        // Maximum time: slowest bullet (power 3.0, speed 11.0)
        double maxTime = distance / 11.0;
        
        return turn >= minTime && turn <= maxTime;
    }

    /**
     * Find all snipes for a single enemy - O(maxTurns) complexity
     */
    private ArrayList<Snipe> getSnipesForEnemy(EnemyBot enemy, int maxTurns) {
        ArrayList<Snipe> snipes = new ArrayList<>();
        
        // Skip dead enemies
        if (enemy.energy <= 0) {
            return snipes;
        }
        
        Point2D.Double myPos = new Point2D.Double(getX(), getY());
        
        // Cache distances for this tick
        enemy.calcDistances(myPos);
        
        // For each future turn, calculate if we can hit
        for (int turn = 1; turn <= maxTurns && turn < enemy.predictedPositions.length; turn++) {
            Point2D.Double targetPos = enemy.predictedPositions[turn - 1];
            if (targetPos == null) continue;
            
            double distance = enemy.predictedDistances[turn - 1];
            
            // Quick rejection: is this turn even reachable?
            if (!isTurnPossible(distance, turn)) {
                continue;
            }
            
            // Calculate aim angle
            double aimAngle = Math.atan2(targetPos.x - myPos.x, targetPos.y - myPos.y);
            
            // Gun turn time
            int gunTurnTime = gunTurnTime(aimAngle);
            
            // Bullet must travel for (turn - gunTurnTime) ticks after firing
            int bulletFlightTurns = turn - gunTurnTime;
            if (bulletFlightTurns <= 0) {
                continue;  // Can't turn gun fast enough
            }
            
            // What power hits at exactly this turn?
            Double power = getPowerForSnipe(distance, bulletFlightTurns);
            if (power == null) {
                continue;  // No valid power for this timing
            }
            
            // Verify the math (sanity check)
            double bulletSpeed = getBulletSpeed(power);
            double actualFlightTime = distance / bulletSpeed;
            if (Math.abs(actualFlightTime - bulletFlightTurns) > 0.5) {
                continue;  // Rounding error made this invalid
            }
            
            // Valid snipe found
            snipes.add(new Snipe(power, turn, enemy));
        }
        
        return snipes;
    }

    /**
     * Choose the best snipe from available options
     */
    private Snipe chooseBestSnipe(ArrayList<Snipe> snipes) {
        if (snipes.isEmpty()) {
            return null;
        }
        
        Snipe best = null;
        float bestScore = Float.NEGATIVE_INFINITY;
        
        for (Snipe snipe : snipes) {
            float score = snipe.getScore();
            
            if (score > bestScore) {
                bestScore = score;
                best = snipe;
            }
        }
        
        // Check if best score above min threshold
        if (best != null && bestScore < tresholdScore) {
            return null;
        }
        
        return best;
    }

    /**
     * Check for snipe opportunities and fire if good shot available
     */
    private void snipeCheck() {
        if (getGunHeat() > 0) {
            return;
        }
        
        ArrayList<Snipe> snipes = new ArrayList<>();
        
        for (EnemyBot enemy : enemies.values()) {
            // Only consider recently scanned enemies
            if (getTime() - enemy.turnLastSeen > maxScanAge) {
                continue;
            }
            snipes.addAll(getSnipesForEnemy(enemy, maxPredictFrames));
        }
        
        Snipe best = chooseBestSnipe(snipes);
        
        if (best == null) {
            return;
        }
        
        // Aim at target position
        Point2D.Double targetPos = best.enemy.predictedPositions[best.numTurns - 1];
        if (targetPos == null) {
            return;
        }
        
        double aimAngle = Math.atan2(targetPos.x - getX(), targetPos.y - getY());
        double turnAngle = Utils.normalRelativeAngle(aimAngle - getGunHeadingRadians());
        setTurnGunRightRadians(turnAngle);
        
        // Fire if gun is aligned
        if (Math.abs(getGunTurnRemainingRadians()) < 0.05) {
            setFire(best.power);
        }
    }

    private EnemyBot findKillTarget() {
        EnemyBot best = null;
        double lowestEnergy = lowEnergy + 1;
        
        for (EnemyBot enemy : enemies.values()) {
            if (enemy.isKillTarget() && enemy.energy < lowestEnergy) {
                // Don't chase kill targets that are too far
                if (enemy.distance < 500) {
                    lowestEnergy = enemy.energy;
                    best = enemy;
                }
            }
        }
        
        return best;
    }
    
    // ========== EVENT HANDLERS ==========
    
    @Override
    public void onScannedRobot(ScannedRobotEvent e) {
        if (isTeammate(e.getName())) {
            return;
        }
        
        EnemyBot enemy = enemies.computeIfAbsent(e.getName(), EnemyBot::new);
        
        double bulletPower = enemy.checkFired();
        if (bulletPower > 0 && enemy.position != null) {
            double bulletHeading = Math.atan2(
                getX() - enemy.position.x,
                getY() - enemy.position.y
            );
            incomingBullets.add(new IncomingBullet(
                enemy.position, bulletHeading, bulletPower, getTime() - 1
            ));
        }
        
        enemy.update(e, this, getTime());
    }
    
    @Override
    public void onRobotDeath(RobotDeathEvent e) {
        EnemyBot enemy = enemies.get(e.getName());
        if (enemy != null) {
            enemy.isAlive = false;
            
            if (killTarget != null && killTarget.name.equals(e.getName())) {
                killTarget = null;
                currState = MovementState.VIBING;
            }
            if (currTarget != null && currTarget.name.equals(e.getName())) {
                currTarget = null;
            }
        }
    }
    
    @Override
    public void onHitWall(HitWallEvent e) {
        moveDirection *= -1;
        setBack(100);
    }
    
    @Override
    public void onHitRobot(HitRobotEvent e) {
        // When we ram someone, fire at max power!
        if (getGunHeat() == 0 && getEnergy() > 3) {
            setFire(3.0);
        }
        
        if (currState == MovementState.KILL_MODE && 
            killTarget != null && e.getName().equals(killTarget.name)) {
            setAhead(50);  // Keep ramming
        } else if (currState == MovementState.RETREAT) {
            // If retreating and we hit them, back away fast
            setBack(150);
        } else {
            moveDirection *= -1;
            setBack(80);
        }
    }
    
    @Override
    public void onHitByBullet(HitByBulletEvent e) {
        // 50% chance to invert movement on hit
        if (random.nextBoolean()) {
            moveDirection *= -1;
        }
        directionChangeTime = getTime();
    }
}