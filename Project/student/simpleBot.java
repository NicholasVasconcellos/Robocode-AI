package student;

import robocode.*;
import robocode.util.Utils;
import java.awt.Color;
import java.awt.geom.Point2D;

/**
 * WaveRider - A competitive Robocode bot
 * 
 * Strategy:
 * - Oscillating perpendicular movement (dodges linear/predictive shots)
 * - Tight radar lock on enemy
 * - Linear predictive targeting
 * - Wall smoothing to avoid corners
 */
public class simpleBot extends TeamRobot {

    private int moveDirection = 1;        // 1 = forward, -1 = backward
    private int turnCounter = 0;          // Controls oscillation timing
    private double enemyEnergy = 100;     // Track enemy energy for bullet detection

    public void run() {
        // Visual customization
        setColors(Color.yellow, Color.yellow, Color.black);
        
        // Independent movement: gun/radar turn independently from body
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);
        
        // Initial radar sweep to find enemies
        setTurnRadarRight(Double.POSITIVE_INFINITY);
        
        while (true) {
            // Keep radar spinning if we lose track
            if (getRadarTurnRemaining() == 0) {
                setTurnRadarRight(Double.POSITIVE_INFINITY);
            }
            execute();
        }
    }

    public void onScannedRobot(ScannedRobotEvent e) {
        // ========== RADAR LOCK ==========
        // Tight radar lock using "narrow lock" technique
        double radarTurn = getHeadingRadians() + e.getBearingRadians() - getRadarHeadingRadians();
        setTurnRadarRightRadians(1.9 * Utils.normalRelativeAngle(radarTurn));

        // ========== MOVEMENT (Oscillating Perpendicular) ==========
        // Detect if enemy fired (energy drop between 0.1 and 3.0)
        double energyDrop = enemyEnergy - e.getEnergy();
        if (energyDrop > 0.1 && energyDrop <= 3.0) {
            // Enemy likely fired - reverse direction
            moveDirection *= -1;
            turnCounter = 0;
        }
        enemyEnergy = e.getEnergy();

        // Oscillate direction periodically even without detected shots
        turnCounter++;
        if (turnCounter > 20 + (int)(Math.random() * 20)) {
            moveDirection *= -1;
            turnCounter = 0;
        }

        // Move perpendicular to enemy (offset by ~80° for slight approach)
        double angleToEnemy = e.getBearingRadians() + getHeadingRadians();
        double desiredAngle = angleToEnemy + (Math.PI / 2.2) * moveDirection;
        
        // Wall smoothing - adjust if too close to walls
        desiredAngle = wallSmoothing(desiredAngle, moveDirection);
        
        // Turn and move
        setTurnRightRadians(Utils.normalRelativeAngle(desiredAngle - getHeadingRadians()));
        setAhead(150 * moveDirection);

        // ========== TARGETING (Linear Prediction) ==========
        double bulletPower = Math.min(3.0, Math.min(e.getEnergy() / 4, getEnergy() / 10));
        bulletPower = Math.max(0.5, bulletPower);  // Minimum power
        
        double bulletSpeed = 20 - 3 * bulletPower;

        // Calculate enemy position
        double enemyX = getX() + e.getDistance() * Math.sin(angleToEnemy);
        double enemyY = getY() + e.getDistance() * Math.cos(angleToEnemy);

        // Linear prediction: where will enemy be when bullet arrives?
        double enemyHeading = e.getHeadingRadians();
        double enemyVelocity = e.getVelocity();
        
        double deltaTime = 0;
        double predictedX = enemyX;
        double predictedY = enemyY;
        
        // Iterative prediction (accounts for bullet travel time)
        while (deltaTime * bulletSpeed < Point2D.distance(getX(), getY(), predictedX, predictedY)) {
            predictedX += Math.sin(enemyHeading) * enemyVelocity;
            predictedY += Math.cos(enemyHeading) * enemyVelocity;
            
            // Keep prediction in bounds
            predictedX = Math.max(18, Math.min(getBattleFieldWidth() - 18, predictedX));
            predictedY = Math.max(18, Math.min(getBattleFieldHeight() - 18, predictedY));
            
            deltaTime++;
            if (deltaTime > 50) break;  // Safety limit
        }

        // Aim gun at predicted position
        double gunAngle = Utils.normalAbsoluteAngle(
            Math.atan2(predictedX - getX(), predictedY - getY())
        );
        setTurnGunRightRadians(Utils.normalRelativeAngle(gunAngle - getGunHeadingRadians()));

        // Fire when gun is aligned
        if (getGunHeat() == 0 && Math.abs(getGunTurnRemaining()) < 10) {
            setFire(bulletPower);
        }
    }

    /**
     * Wall smoothing - adjusts angle to glide along walls instead of hitting them
     */
    private double wallSmoothing(double angle, int direction) {
        double margin = 120;  // Distance from wall to start smoothing
        double x = getX();
        double y = getY();
        double fieldW = getBattleFieldWidth();
        double fieldH = getBattleFieldHeight();

        // Project future position
        double futureX = x + Math.sin(angle) * margin * direction;
        double futureY = y + Math.cos(angle) * margin * direction;

        // Adjust if projected position is outside safe bounds
        double smoothAngle = angle;
        if (futureX < margin || futureX > fieldW - margin || 
            futureY < margin || futureY > fieldH - margin) {
            
            // Turn toward center
            double centerAngle = Math.atan2(fieldW / 2 - x, fieldH / 2 - y);
            smoothAngle = centerAngle + (Math.PI / 2.5) * direction;
        }
        return smoothAngle;
    }

    public void onHitWall(HitWallEvent e) {
        // Reverse direction on wall hit
        moveDirection *= -1;
    }

    public void onHitRobot(HitRobotEvent e) {
        // Ram damage bonus - fire point blank if we collide
        if (e.isMyFault()) {
            setTurnRight(e.getBearing());
            setFire(3);
        }
        moveDirection *= -1;
    }
}