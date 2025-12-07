package student;

import robocode.*;
import robocode.util.Utils;
import java.awt.geom.Point2D;
import java.util.*;

/**
 * PotatoBot - Advanced Robocode Robot
 * 
 * Features:
 * - State machine movement (Vibing/Zig-Zag vs Kill/Rush mode)
 * - Predictive enemy tracking with 5-frame lookahead
 * - Intelligent targeting with snipe prioritization
 * - Advanced wall avoidance with smooth curves
 * - Energy and cooldown management
 * 
 * @author Nicholas
 */
public class PotatoBot extends TeamRobot {
    
    // ========== CONSTANTS ==========
    
    // Physics constants
    private static final double MAX_VELOCITY = 8.0;
    private static final double ACCELERATION = 1.0;
    private static final double DECELERATION = 2.0;
    private static final double GUN_TURN_RATE_RADIANS = Math.toRadians(20.0);
    private static final double GUN_COOLING_RATE = 0.1;
    
    // Bullet physics
    private static final double BULLET_SPEED_BASE = 20.0;
    private static final double BULLET_SPEED_COEFFICIENT = 3.0;
    
    // Movement parameters
    private static final double WALL_MARGIN = 150.0;        // Distance from wall to start turning
    private static final double ZIG_ZAG_AMPLITUDE = 30.0;   // Degrees for zig-zag pattern
    private static final double ZIG_ZAG_FREQUENCY = 20;     // Turns between direction changes
    
    // Targeting parameters
    private static final int PREDICTION_FRAMES = 5;         // Frames to predict ahead
    private static final double MIN_SNIPE_CONFIDENCE = 0.7; // Minimum hit probability
    private static final int MAX_SNIPE_TIME = 50;          // Max turns to consider for sniping
    
    // Kill mode parameters
    private static final double KILL_MODE_DISTANCE = 200.0; // Distance to activate kill mode
    private static final double KILL_MODE_ENERGY = 20.0;    // Enemy energy threshold for kill mode
    private static final int KILL_MODE_TURN_THRESHOLD = 30; // Max turns to reach target
    
    // ========== STATE VARIABLES ==========
    
    // Movement state
    private enum MovementState {
        VIBING,     // Normal evasive movement
        KILL_MODE   // Rushing low-HP target
    }
    
    private MovementState currentState = MovementState.VIBING;
    private int moveDirection = 1;      // 1 = forward, -1 = backward
    private int turnDirection = 1;      // 1 = right, -1 = left
    private long zigZagTimer = 0;       // Timer for zig-zag pattern
    
    // Enemy tracking
    private final Map<String, EnemyBot> enemies = new HashMap<>();
    private EnemyBot currentTarget = null;  // Current best target for sniping
    private EnemyBot killTarget = null;     // Target for kill mode
    
    // Randomization
    private Random random = new Random();
    
    // ========== ENEMY DATA STRUCTURE ==========
    
    /**
     * Enhanced enemy tracking with prediction capabilities
     */
    public static class EnemyBot {
        // Identity
        public String name;
        public boolean alive = true;
        
        // Current state (updated each scan)
        public double energy;
        public double bearing;      // Relative to us
        public double distance;
        public double heading;      // Absolute heading
        public double velocity;
        public Point2D.Double position;
        public long lastSeenTime;
        
        // Historical data for prediction
        public double previousEnergy = -1;
        public Point2D.Double previousPosition = null;
        public double previousHeading = -1;
        public double previousVelocity = 0;
        
        // Derived motion data
        public double acceleration = 0;      // Change in velocity
        public double angularVelocity = 0;   // Turn rate (radians/turn)
        public int consecutiveScans = 0;     // Consecutive turns we've seen this enemy
        
        // Predictions
        public Point2D.Double[] predictedPositions = new Point2D.Double[PREDICTION_FRAMES];
        
        public EnemyBot(String name) {
            this.name = name;
        }
        
        /**
         * Update enemy state with new scan data
         */
        public void update(ScannedRobotEvent e, AdvancedRobot robot, long currentTime) {
            // Store previous state
            if (position != null) {
                previousPosition = (Point2D.Double) position.clone();
                previousHeading = heading;
                previousVelocity = velocity;
                consecutiveScans++;
            } else {
                consecutiveScans = 1;
            }
            
            if (energy > 0) {
                previousEnergy = energy;
            }
            
            // Update current state
            energy = e.getEnergy();
            bearing = e.getBearing();
            distance = e.getDistance();
            heading = e.getHeading();
            velocity = e.getVelocity();
            lastSeenTime = currentTime;
            
            // Calculate absolute position
            double absoluteBearing = robot.getHeadingRadians() + e.getBearingRadians();
            position = new Point2D.Double(
                robot.getX() + distance * Math.sin(absoluteBearing),
                robot.getY() + distance * Math.cos(absoluteBearing)
            );
            
            // Calculate derived motion if we have history
            if (consecutiveScans >= 2) {
                acceleration = velocity - previousVelocity;
                
                double headingChange = Utils.normalRelativeAngle(
                    Math.toRadians(heading) - Math.toRadians(previousHeading)
                );
                angularVelocity = headingChange;
            }
            
            // Generate predictions
            predictFuturePositions(robot.getBattleFieldWidth(), robot.getBattleFieldHeight());
        }
        
        /**
         * Predict enemy positions for next N frames
         * Uses current velocity, acceleration, and turn rate
         */
        private void predictFuturePositions(double fieldWidth, double fieldHeight) {
            double px = position.x;
            double py = position.y;
            double h = Math.toRadians(heading);
            double v = velocity;
            
            for (int i = 0; i < PREDICTION_FRAMES; i++) {
                // Update heading with angular velocity
                h += angularVelocity;
                
                // Update velocity with acceleration (clamped)
                if (consecutiveScans >= 2) {
                    v += acceleration;
                    v = Math.max(-MAX_VELOCITY, Math.min(MAX_VELOCITY, v));
                }
                
                // Update position
                px += v * Math.sin(h);
                py += v * Math.cos(h);
                
                // Clamp to battlefield (simple wall bounce approximation)
                px = Math.max(18, Math.min(fieldWidth - 18, px));
                py = Math.max(18, Math.min(fieldHeight - 18, py));
                
                predictedPositions[i] = new Point2D.Double(px, py);
            }
        }
        
        /**
         * Check if this enemy is a good kill target
         */
        public boolean isKillTarget(double myX, double myY) {
            return alive && 
                   energy > 0 && 
                   energy <= KILL_MODE_ENERGY && 
                   distance < KILL_MODE_DISTANCE;
        }
        
        /**
         * Estimate turns to reach this target
         */
        public int turnsToReach(double myX, double myY, double myVelocity) {
            double dist = Point2D.distance(myX, myY, position.x, position.y);
            // Simplified: assume we accelerate and they move
            double avgSpeed = (myVelocity + MAX_VELOCITY) / 2;
            return (int) Math.ceil(dist / Math.max(avgSpeed, 1.0));
        }
    }
    
    // ========== MAIN ROBOT LOGIC ==========
    
    @Override
    public void run() {
        // Setup: independent gun and radar
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);
        
        // Initialize movement
        initializeMovement();
        
        // Main loop
        while (true) {
            // Update state machine
            updateMovementState();
            
            // Execute movement based on current state
            if (currentState == MovementState.VIBING) {
                executeVibingMovement();
            } else {
                executeKillModeMovement();
            }
            
            // Targeting and shooting
            updateTargeting();
            attemptSnipe();
            
            // Radar sweep
            setTurnRadarRight(360);
            
            execute();
        }
    }
    
    // ========== MOVEMENT SYSTEM ==========
    
    /**
     * Initialize smooth circular movement
     */
    private void initializeMovement() {
        setAhead(Double.POSITIVE_INFINITY * moveDirection);
        setTurnRight(5 * turnDirection);  // Start with gentle turn
    }
    
    /**
     * Update movement state based on enemy analysis
     */
    private void updateMovementState() {
        // Check if we should enter kill mode
        if (currentState == MovementState.VIBING) {
            killTarget = findKillTarget();
            if (killTarget != null) {
                int turnsToReach = killTarget.turnsToReach(getX(), getY(), getVelocity());
                if (turnsToReach <= KILL_MODE_TURN_THRESHOLD) {
                    currentState = MovementState.KILL_MODE;
                    out.println("KILL MODE ACTIVATED: Target " + killTarget.name + 
                               " (Energy: " + killTarget.energy + ")");
                }
            }
        } 
        // Check if we should exit kill mode
        else if (currentState == MovementState.KILL_MODE) {
            if (killTarget == null || !killTarget.alive || 
                killTarget.energy > KILL_MODE_ENERGY || 
                killTarget.distance > KILL_MODE_DISTANCE * 1.5) {
                currentState = MovementState.VIBING;
                killTarget = null;
                out.println("Exiting KILL MODE - returning to vibing");
                initializeMovement();
            }
        }
    }
    
    /**
     * Execute vibing movement: zig-zag evasive pattern with wall avoidance
     */
    private void executeVibingMovement() {
        // Zig-zag pattern: change direction periodically
        if (getTime() - zigZagTimer > ZIG_ZAG_FREQUENCY) {
            zigZagTimer = getTime();
            
            // 10% chance to reverse turn direction (adds unpredictability)
            if (random.nextDouble() < 0.10) {
                turnDirection *= -1;
            }
        }
        
        // Wall avoidance with smooth reflection
        WallAvoidanceResult wallAvoid = calculateWallAvoidance();
        
        if (wallAvoid.shouldTurn) {
            // Smoothly curve away from wall
            turnDirection = wallAvoid.suggestedDirection;
            
            // Reduce speed if turning sharply
            double turnAmount = Math.abs(wallAvoid.suggestedTurnAmount);
            if (turnAmount > 45) {
                setMaxVelocity(MAX_VELOCITY * 0.6);  // Slow down for sharp turns
            } else {
                setMaxVelocity(MAX_VELOCITY);  // Full speed for gentle turns
            }
            
            setTurnRight(wallAvoid.suggestedTurnAmount);
        } else {
            // Normal zig-zag movement
            setMaxVelocity(MAX_VELOCITY);
            double zigZagAngle = ZIG_ZAG_AMPLITUDE * Math.sin(getTime() / ZIG_ZAG_FREQUENCY);
            setTurnRight(zigZagAngle * turnDirection);
        }
        
        // Maintain forward movement
        setAhead(Double.POSITIVE_INFINITY * moveDirection);
    }
    
    /**
     * Execute kill mode movement: intercept and ram target
     */
    private void executeKillModeMovement() {
        if (killTarget == null || !killTarget.alive) {
            return;
        }
        
        // Predict where target will be
        Point2D.Double targetPos = killTarget.position;
        if (killTarget.predictedPositions[2] != null) {
            targetPos = killTarget.predictedPositions[2];  // 3 turns ahead
        }
        
        // Calculate intercept angle
        double angleToTarget = Math.atan2(
            targetPos.x - getX(),
            targetPos.y - getY()
        );
        
        double turnAmount = Utils.normalRelativeAngle(angleToTarget - getHeadingRadians());
        
        // Turn toward target
        setTurnRightRadians(turnAmount);
        
        // Move toward target at full speed
        setMaxVelocity(MAX_VELOCITY);
        if (Math.abs(Math.toDegrees(turnAmount)) < 90) {
            setAhead(killTarget.distance);
        } else {
            setBack(killTarget.distance);
        }
    }
    
    /**
     * Calculate wall avoidance with smooth reflection
     */
    private WallAvoidanceResult calculateWallAvoidance() {
        double x = getX();
        double y = getY();
        double heading = getHeading();
        double fieldWidth = getBattleFieldWidth();
        double fieldHeight = getBattleFieldHeight();
        
        // Calculate effective heading (account for backward movement)
        double effectiveHeading = heading;
        if (moveDirection == -1) {
            effectiveHeading = normalizeAngle(heading + 180);
        }
        
        WallAvoidanceResult result = new WallAvoidanceResult();
        
        // Check proximity and heading toward each wall
        if (x < WALL_MARGIN && isHeadingToward(270, effectiveHeading)) {
            // Too close to left wall, turn right
            result.shouldTurn = true;
            result.suggestedDirection = 1;
            result.suggestedTurnAmount = calculateWallReflectionAngle(x, WALL_MARGIN, 270, effectiveHeading);
        } 
        else if (x > fieldWidth - WALL_MARGIN && isHeadingToward(90, effectiveHeading)) {
            // Too close to right wall, turn left
            result.shouldTurn = true;
            result.suggestedDirection = -1;
            result.suggestedTurnAmount = -calculateWallReflectionAngle(
                fieldWidth - x, WALL_MARGIN, 90, effectiveHeading);
        } 
        else if (y < WALL_MARGIN && isHeadingToward(180, effectiveHeading)) {
            // Too close to bottom wall, turn away
            result.shouldTurn = true;
            result.suggestedDirection = effectiveHeading < 180 ? 1 : -1;
            result.suggestedTurnAmount = calculateWallReflectionAngle(
                y, WALL_MARGIN, 180, effectiveHeading) * result.suggestedDirection;
        } 
        else if (y > fieldHeight - WALL_MARGIN && isHeadingToward(0, effectiveHeading)) {
            // Too close to top wall, turn away
            result.shouldTurn = true;
            result.suggestedDirection = effectiveHeading > 180 ? 1 : -1;
            result.suggestedTurnAmount = calculateWallReflectionAngle(
                fieldHeight - y, WALL_MARGIN, 0, effectiveHeading) * result.suggestedDirection;
        }
        
        return result;
    }
    
    /**
     * Calculate smooth reflection angle away from wall
     * Returns larger angle when closer to wall
     */
    private double calculateWallReflectionAngle(double distToWall, double margin, 
                                                 double wallAngle, double heading) {
        // Intensity increases as we get closer (0 = far, 1 = at wall)
        double intensity = 1.0 - (distToWall / margin);
        intensity = Math.max(0, Math.min(1, intensity));
        
        // Base angle increases with proximity
        double baseAngle = 20 + (70 * intensity);  // 20° to 90°
        
        return baseAngle;
    }
    
    private static class WallAvoidanceResult {
        boolean shouldTurn = false;
        int suggestedDirection = 1;
        double suggestedTurnAmount = 0;
    }
    
    /**
     * Check if heading toward a direction (within 90 degrees)
     */
    private boolean isHeadingToward(double targetAngle, double currentHeading) {
        double diff = Math.abs(normalizeAngle(currentHeading - targetAngle));
        return diff < 90;
    }
    
    private double normalizeAngle(double angle) {
        while (angle >= 360) angle -= 360;
        while (angle < 0) angle += 360;
        return angle;
    }
    
    // ========== TARGETING SYSTEM ==========
    
    /**
     * Update current best target for sniping
     */
    private void updateTargeting() {
        currentTarget = null;
        SnipeOpportunity bestSnipe = null;
        
        // Evaluate all alive enemies
        for (EnemyBot enemy : enemies.values()) {
            if (!enemy.alive || getTime() - enemy.lastSeenTime > 10) {
                continue;  // Skip dead or stale enemies
            }
            
            // Calculate best snipe opportunity for this enemy
            SnipeOpportunity snipe = calculateBestSnipe(enemy);
            
            if (snipe != null && (bestSnipe == null || snipe.isBetterThan(bestSnipe))) {
                bestSnipe = snipe;
                currentTarget = enemy;
            }
        }
    }
    
    /**
     * Calculate best snipe opportunity for an enemy
     * Returns null if no good snipe is possible
     */
    private SnipeOpportunity calculateBestSnipe(EnemyBot enemy) {
        SnipeOpportunity best = null;
        
        // Try different power levels
        for (double power = 0.1; power <= 3.0; power += 0.1) {
            power = Math.round(power * 10) / 10.0;  // Round to 1 decimal
            
            // Try each prediction frame
            for (int frame = 0; frame < PREDICTION_FRAMES; frame++) {
                if (enemy.predictedPositions[frame] == null) continue;
                
                SnipeOpportunity snipe = checkSnipe(enemy, frame, power);
                
                if (snipe != null && snipe.confidence >= MIN_SNIPE_CONFIDENCE) {
                    if (best == null || snipe.isBetterThan(best)) {
                        best = snipe;
                    }
                }
            }
        }
        
        return best;
    }
    
    /**
     * Check if we can snipe enemy at predicted frame with given power
     * 
     * Algorithm:
     * 1. Calculate bullet travel time to predicted position
     * 2. Calculate gun turn time to aim at position
     * 3. Total time = gun turn time + bullet travel time
     * 4. Check if total time ≈ frame number (within tolerance)
     * 5. Calculate hit confidence based on timing accuracy
     */
    private SnipeOpportunity checkSnipe(EnemyBot enemy, int predictionFrame, double power) {
        Point2D.Double targetPos = enemy.predictedPositions[predictionFrame];
        
        // Calculate bullet speed and travel time
        double bulletSpeed = BULLET_SPEED_BASE - BULLET_SPEED_COEFFICIENT * power;
        double distance = Point2D.distance(getX(), getY(), targetPos.x, targetPos.y);
        double bulletTravelTime = distance / bulletSpeed;
        
        // Calculate gun turn time
        double aimAngle = Math.atan2(targetPos.x - getX(), targetPos.y - getY());
        double gunTurnNeeded = Math.abs(Utils.normalRelativeAngle(aimAngle - getGunHeadingRadians()));
        double gunTurnTime = gunTurnNeeded / GUN_TURN_RATE_RADIANS;
        
        // Total time from now until bullet reaches predicted position
        double totalTime = gunTurnTime + bulletTravelTime;
        
        // Check if timing aligns with prediction frame (with tolerance)
        double timingError = Math.abs(totalTime - predictionFrame);
        
        if (timingError > 3.0) {  // More than 3 turns off
            return null;  // Poor timing
        }
        
        // Calculate confidence (1.0 = perfect, decreases with timing error)
        double confidence = 1.0 - (timingError / 3.0);
        
        // Penalize if gun needs large turn
        if (gunTurnNeeded > Math.toRadians(45)) {
            confidence *= 0.8;
        }
        
        // Penalize very long shots
        if (distance > 400) {
            confidence *= 0.9;
        }
        
        return new SnipeOpportunity(
            enemy,
            predictionFrame,
            power,
            aimAngle,
            (int) Math.ceil(totalTime),
            confidence
        );
    }
    
    /**
     * Represents a potential snipe opportunity
     */
    private static class SnipeOpportunity {
        EnemyBot target;
        int predictionFrame;
        double power;
        double aimAngle;
        int turnsToHit;
        double confidence;
        
        SnipeOpportunity(EnemyBot target, int frame, double power, 
                        double aimAngle, int turns, double confidence) {
            this.target = target;
            this.predictionFrame = frame;
            this.power = power;
            this.aimAngle = aimAngle;
            this.turnsToHit = turns;
            this.confidence = confidence;
        }
        
        /**
         * Prioritization:
         * 1. Fewest turns to hit (faster is better)
         * 2. Higher power (more damage)
         * 3. Lower enemy energy (finish off weak enemies)
         */
        boolean isBetterThan(SnipeOpportunity other) {
            // Primary: prefer fewer turns
            if (this.turnsToHit < other.turnsToHit) return true;
            if (this.turnsToHit > other.turnsToHit) return false;
            
            // Secondary: prefer higher power
            if (this.power > other.power) return true;
            if (this.power < other.power) return false;
            
            // Tertiary: prefer lower enemy energy
            return this.target.energy < other.target.energy;
        }
    }
    
    /**
     * Attempt to execute the best snipe
     */
    private void attemptSnipe() {
        // Check if we can fire
        if (getGunHeat() > 0 || currentTarget == null) {
            return;
        }
        
        // Get best snipe for current target
        SnipeOpportunity snipe = calculateBestSnipe(currentTarget);
        
        if (snipe == null || snipe.confidence < MIN_SNIPE_CONFIDENCE) {
            return;  // No good shot
        }
        
        // Check if we have enough energy
        if (getEnergy() < snipe.power) {
            return;  // Not enough energy
        }
        
        // Aim gun
        double gunTurn = Utils.normalRelativeAngle(snipe.aimAngle - getGunHeadingRadians());
        setTurnGunRightRadians(gunTurn);
        
        // Fire if aimed (within tolerance)
        if (Math.abs(getGunTurnRemaining()) < Math.toRadians(5)) {
            setFire(snipe.power);
            out.println(String.format("SNIPE: %s | Power: %.1f | Frame: %d | Confidence: %.2f",
                snipe.target.name, snipe.power, snipe.predictionFrame, snipe.confidence));
        }
    }
    
    /**
     * Find best target for kill mode
     */
    private EnemyBot findKillTarget() {
        EnemyBot best = null;
        int minTurns = Integer.MAX_VALUE;
        
        for (EnemyBot enemy : enemies.values()) {
            if (enemy.isKillTarget(getX(), getY())) {
                int turns = enemy.turnsToReach(getX(), getY(), getVelocity());
                if (turns < minTurns) {
                    minTurns = turns;
                    best = enemy;
                }
            }
        }
        
        return best;
    }
    
    // ========== EVENT HANDLERS ==========
    
    @Override
    public void onScannedRobot(ScannedRobotEvent e) {
        // Skip teammates
        if (isTeammate(e.getName())) {
            return;
        }
        
        // Get or create enemy
        EnemyBot enemy = enemies.get(e.getName());
        if (enemy == null) {
            enemy = new EnemyBot(e.getName());
            enemies.put(e.getName(), enemy);
            out.println("New enemy detected: " + e.getName());
        }
        
        // Update enemy state and predictions
        enemy.update(e, this, getTime());
    }
    
    @Override
    public void onRobotDeath(RobotDeathEvent e) {
        EnemyBot enemy = enemies.get(e.getName());
        if (enemy != null) {
            enemy.alive = false;
            out.println("Enemy destroyed: " + e.getName());
            
            // Clear kill target if it died
            if (killTarget != null && killTarget.name.equals(e.getName())) {
                killTarget = null;
                currentState = MovementState.VIBING;
                initializeMovement();
            }
        }
    }
    
    @Override
    public void onHitWall(HitWallEvent e) {
        // Emergency: reverse both directions
        moveDirection *= -1;
        turnDirection *= -1;
        setBack(100);
        out.println("Wall hit - emergency reverse");
    }
    
    @Override
    public void onHitRobot(HitRobotEvent e) {
        // If we're in kill mode and this is our target, keep pushing
        if (currentState == MovementState.KILL_MODE && 
            killTarget != null && 
            e.getName().equals(killTarget.name)) {
            setAhead(100);  // Ram them!
            
            // Fire at point blank
            if (getGunHeat() == 0) {
                setFire(3.0);
            }
        } else {
            // Not our kill target, back away
            moveDirection *= -1;
            setBack(100);
        }
    }
    
    @Override
    public void onHitByBullet(HitByBulletEvent e) {
        // Evasive maneuver: change direction unpredictably
        if (random.nextDouble() < 0.3) {
            moveDirection *= -1;
        }
        if (random.nextDouble() < 0.3) {
            turnDirection *= -1;
        }
    }
}