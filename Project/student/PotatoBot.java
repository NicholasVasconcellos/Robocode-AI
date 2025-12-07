package student;

import robocode.*;
import java.util.Map;
import java.util.HashMap;
import java.util.Random;

public class PotatoBot extends TeamRobot
{

    private Random random = new Random();
    private int moveDirection = 1; // 1 = forward, -1 = backward
    private int turnDirection = 1; // 1 = right, -1 = left

    // Map of enemies keyed by their robot name
    private final Map<String, Enemy> enemies = new HashMap<>();

    // Simple struct to hold enemy state
    public static class Enemy {
        public String name;
        public double energy;
        public double bearing;
        public double distance;
        public double heading;
        public double velocity;
        public long lastSeenTime;
        public boolean alive = true;

        public Enemy(String name){
            this.name = name;
        }
    }

    public void run(){
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);

        // Start continuous circular movement
        setAhead(Double.POSITIVE_INFINITY * moveDirection);
        setTurnRight(Double.POSITIVE_INFINITY * turnDirection);

        while(true){
            // Check wall proximity and adjust if needed
            avoidWalls();

            // Keep radar spinning
            setTurnRadarRight(360);

            execute();
        }
    }

    private void avoidWalls() {
        double margin = 150; // Distance from wall to start turning
        double x = getX();
        double y = getY();
        double heading = getHeading();
        double fieldWidth = getBattleFieldWidth();
        double fieldHeight = getBattleFieldHeight();

        // In Robocode: 0° = North (up), 90° = East (right), 180° = South (down), 270° = West (left)
        
        // Calculate effective heading (account for moving backward)
        double effectiveHeading = heading;
        if (moveDirection == -1) {
            effectiveHeading = normalizeAngle(heading + 180);
        }

        boolean needToTurn = false;

        // Check each wall - are we close AND heading toward it?
        // Left wall (x = 0): heading toward it if heading is between 180-360 (heading west component)
        if (x < margin && isHeadingToward(270, effectiveHeading)) {
            needToTurn = true;
        }
        // Right wall (x = max): heading toward it if heading is between 0-180 (heading east component)
        else if (x > fieldWidth - margin && isHeadingToward(90, effectiveHeading)) {
            needToTurn = true;
        }
        // Bottom wall (y = 0): heading toward it if heading is between 90-270 (heading south component)
        else if (y < margin && isHeadingToward(180, effectiveHeading)) {
            needToTurn = true;
        }
        // Top wall (y = max): heading toward it if heading is between 270-360 or 0-90 (heading north component)
        else if (y > fieldHeight - margin && isHeadingToward(0, effectiveHeading)) {
            needToTurn = true;
        }

        if (needToTurn) {
            // Reverse turn direction to arc away from wall (like a reflection)
            reverseTurnDirection();
        }
    }

    // Check if robot is roughly heading toward a direction (within 90 degrees)
    private boolean isHeadingToward(double targetAngle, double currentHeading) {
        double diff = Math.abs(normalizeAngle(currentHeading - targetAngle));
        return diff < 90;
    }

    private double normalizeAngle(double angle) {
        while (angle >= 360) angle -= 360;
        while (angle < 0) angle += 360;
        return angle;
    }

    private void reverseTurnDirection() {
        turnDirection *= -1;
        // Keep moving, just change turn direction for the arc reflection
        setAhead(Double.POSITIVE_INFINITY * moveDirection);
        setTurnRight(Double.POSITIVE_INFINITY * turnDirection);
    }

    private void reverseMove() {
        moveDirection *= -1;
        setAhead(Double.POSITIVE_INFINITY * moveDirection);
    }

    // Update or create enemy info when we scan a robot
    public void onScannedRobot(ScannedRobotEvent e){
        String name = e.getName();
        Enemy en = enemies.get(name);
        if (en == null){
            en = new Enemy(name);
            enemies.put(name, en);
        }
        en.energy = e.getEnergy();
        en.bearing = e.getBearing();
        en.distance = e.getDistance();
        en.heading = e.getHeading();
        en.velocity = e.getVelocity();
        en.lastSeenTime = getTime();
        en.alive = true;
    }

    // Mark enemy dead when we receive a death event
    public void onRobotDeath(RobotDeathEvent e){
        String name = e.getName();
        Enemy en = enemies.get(name);
        if (en != null){
            en.alive = false;
        }
    }

    public void onHitWall(HitWallEvent e){
        // Reverse move direction and turn to escape
        reverseMove();
        reverseTurnDirection();
    }

    public void onHitRobot(HitRobotEvent e){
        // Reverse move to get away from the other robot
        reverseMove();
    }

    private void predictPos(Enemy enemy, int n){
        // predict frame by frame position of this enemy up to n frames in the future
        // Return array size n of positions
    }

    private void getNextPos(Enemy enemy){
        // return the next frame position
    }

    // Check snipe
    private void checkSnipe(int i, int power){
        // start pos, end pos, bullet speed, rotation speed
        // check snipe at i turns in the future is possible with power
        // || Pi - P0 || ≤ Vbullet * ( i -  ceil([curr heading - arcTan(pi-p0)])
    }
}