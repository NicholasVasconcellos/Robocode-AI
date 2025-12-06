package student;

import robocode.*;
import java.util.Map;
import java.util.HashMap;

public class PotatoBot extends TeamRobot
{
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

        while(true){

            setAhead(100);

            while (getDistanceRemaining() > 0) {
                execute();

            }

        }
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
        if(Math.ceil(getTime())%2 == 0){
            turnRight(90);
        }
        else{
            turnLeft(90);
        }
    }

    public void onHitRobot(HitRobotEvent e){
        if(Math.ceil(getTime())%2 == 0){
            turnRight(90);
        }
        else{
            turnLeft(90);
        }
    }

    private void predictPos(Enemy enemy, int n){
        // predict frame by frame positin of this enemy up to n frames in the future
        // Return array size n of positions
    }

    private void getNextPos(Enemy enemy){
        // return the next frame position
    }

    // Check snipe
    private void checkSnipe(int i, int power){
        // start pos, end pos, bullet speed, rotation speed
        // check snipe at i turns in the future is possible with power
        // || Pi - P0 || ≤ Vbullet * ( i -  ceil([curr heading - arcTan(pi-p0]))
    }
}
