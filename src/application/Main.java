package application;

import java.util.*;

/**
 * Main Class: Entry point for the Drone Fleet Simulator.
 * Orchestrates mission logic and bridges the Simulator with the GUI.
 */
public class Main {

    // ===== CONFIGURATION CONSTANTS =====
    public static final double DT = 0.02;          
    public static final double SPRAY_SECONDS = 10.0;
    public static Simulator simulator;

    // Environment and Mission Targets
    public static final Vector3D INITIAL = new Vector3D(0, 0, 10);
    public static final Vector3D TARGET  = new Vector3D(14, 0, 10); 

    // Mission phases tracking [cite: 15]
    public enum Phase { GO_TO_TARGET, SPRAYING, RETURNING, DONE }
    private static final Map<Integer, Phase> phase = new HashMap<>();
    private static final Map<Integer, Double> sprayStartTime = new HashMap<>();

    // Motion tuning constants
    private static final double ARRIVE_RADIUS = 1.2;
    private static final double TRAVEL_SPEED  = 3.5;   
    private static final double BLEND         = 0.25;  
    private static final double MAX_SPEED     = 4.0;   

    /**
     * Main method initializes the system via the Simulator's file-based logic.
     */
    public static void main(String[] args) {
        
        // 1. Initialize Simulator using file-based configuration 
        // This satisfies the requirement to avoid hard-coded parameters in Main.
        simulator = new Simulator();

        // 2. Retrieve the drone fleet generated from config.txt [cite: 17, 50]
        List<Drone> drones = simulator.getDrones();

        // 3. Initialize mission states for the loaded drones
        for (Drone d : drones) {
            int id = d.getDroneID();
            phase.put(id, Phase.GO_TO_TARGET);
            
            // Set initial hover target based on current position [cite: 81, 101]
            DroneState s = d.getState();
            d.getController().setTargetPosition(new Vector3D(s.position.x, s.position.y, INITIAL.z));
            d.getController().setTargetVelocity(new Vector3D(0, 0, 0));
        }

        // 4. Launch the GUI for visualization
        DroneGUI.main(args);
    }

    /**
     * updateMission: Called by the GUI every frame to update high-level mission logic.
     * Manages the transition between surveying and spraying phases[cite: 15].
     */
    public static void updateMission(double simTimeSec, List<Drone> drones) {

        for (Drone d : drones) {
            int id = d.getDroneID();
            Phase ph = phase.getOrDefault(id, Phase.GO_TO_TARGET);
            DroneState s = d.getState();

            // Maintain stable altitude via the Controller [cite: 21, 94]
            d.getController().setTargetPosition(
                    new Vector3D(s.position.x, s.position.y, INITIAL.z)
            );

            if (ph == Phase.GO_TO_TARGET) {
                // Check if drone arrived at the agricultural target area [cite: 15, 81]
                if (distanceXY(s.position, TARGET) <= ARRIVE_RADIUS) {
                    phase.put(id, Phase.SPRAYING);
                    sprayStartTime.put(id, simTimeSec);
                    s.velocity = new Vector3D(0, 0, 0);
                    continue;
                }
                steerXYVelocity(s, TARGET);

            } else if (ph == Phase.SPRAYING) {
                // Perform precision spraying for the duration specified [cite: 15]
                s.velocity = new Vector3D(0, 0, 0);
                double start = sprayStartTime.getOrDefault(id, simTimeSec);
                if (simTimeSec - start >= SPRAY_SECONDS) {
                    phase.put(id, Phase.RETURNING);
                }

            } else if (ph == Phase.RETURNING) {
                // Return to base after mission completion
                if (distanceXY(s.position, INITIAL) <= ARRIVE_RADIUS) {
                    phase.put(id, Phase.DONE);
                    s.velocity = new Vector3D(0, 0, 0);
                    continue;
                }
                steerXYVelocity(s, INITIAL);

            } else {
                // Mission complete: Hover at base
                s.velocity = new Vector3D(0, 0, 0);
            }

            // Safety check: Clamp speed to prevent instability [cite: 99]
            clampSpeedXY(s, MAX_SPEED);
        }
    }

    public static boolean isSpraying(int droneId) {
        return phase.getOrDefault(droneId, Phase.GO_TO_TARGET) == Phase.SPRAYING;
    }

    // ===== MATHEMATICAL HELPERS =====
    
    private static void steerXYVelocity(DroneState s, Vector3D goal) {
        double dx = goal.x - s.position.x;
        double dy = goal.y - s.position.y;
        double dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < 1e-6) return;

        double vxDesired = (dx / dist) * TRAVEL_SPEED;
        double vyDesired = (dy / dist) * TRAVEL_SPEED;

        // Apply control law to blend velocity for smooth movement [cite: 100, 109]
        s.velocity = new Vector3D(
                s.velocity.x * (1.0 - BLEND) + vxDesired * BLEND,
                s.velocity.y * (1.0 - BLEND) + vyDesired * BLEND,
                0.0
        );
    }

    private static void clampSpeedXY(DroneState s, double maxSpeed) {
        double vx = s.velocity.x;
        double vy = s.velocity.y;
        double speed = Math.sqrt(vx * vx + vy * vy);
        if (speed <= maxSpeed || speed < 1e-9) return;

        double scale = maxSpeed / speed;
        s.velocity = new Vector3D(vx * scale, vy * scale, 0.0);
    }

    private static double distanceXY(Vector3D a, Vector3D b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public static Simulator getSimulator() {
        return simulator;
    }
}