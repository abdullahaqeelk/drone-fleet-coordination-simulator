package application;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.*;

public class Simulator {

    private List<Drone> drones = new ArrayList<>();
    private CommunicationModule comm;
    private FormationManager fm;
    private CollisionAvoidance ca;
    private Environment env;
    private Logger logger;
    private double dt; 

    public Simulator() {
        loadConfig("config.txt");
        
    }

    private void loadConfig(String fileName) {
        Map<String, String> params = new HashMap<>();
        try (Scanner scanner = new Scanner(new File(fileName))) {
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine().trim();
                if (line.isEmpty() || !line.contains("=")) continue;
                String[] parts = line.split("=");
                params.put(parts[0].trim(), parts[1].trim());
            }

            this.dt = Double.parseDouble(params.getOrDefault("timeStep", "0.02"));
            int numDrones = Integer.parseInt(params.getOrDefault("numDrones", "6"));

            double w = Double.parseDouble(params.getOrDefault("width", "50"));
            double h = Double.parseDouble(params.getOrDefault("height", "50"));
            this.env = new Environment(-w/2, w/2, -h/2, h/2, 0, 20, 1.0, false);

            this.comm = new CommunicationModule(18.0, 0.2); 
            this.fm = new FormationManager(0.35, 0.2, 18.0);
            this.ca = new CollisionAvoidance(25.0, 3.5);
            this.logger = new Logger(); 

            double mass = Double.parseDouble(params.getOrDefault("mass", "1.0"));
            double kd = Double.parseDouble(params.getOrDefault("kd", "0.12"));

            for (int i = 0; i < numDrones; i++) {
                double offsetX = (i % 3) * 2.5;
                double offsetY = (i / 3) * 2.5;
                Vector3D startPos = new Vector3D(offsetX, offsetY, 10.0);
                drones.add(new Drone(i, mass, kd, startPos));
            }
            System.out.println("Initialized " + drones.size() + " drones.");

        } catch (FileNotFoundException e) {
            System.err.println("Config file not found in root directory.");
        }
    }

    public void step() {
        for (Drone d : drones) d.setNeighbors(comm.getNeighbors(d, drones));
        for (Drone d : drones) fm.computeFormationForce(d, d.getNeighbors());
        for (Drone d : drones) ca.computeRepulsiveForce(d, d.getNeighbors());
        for (Drone d : drones) d.getController().computeControl(d);
        for (Drone d : drones) env.apply(d);
        for (Drone d : drones) d.integrate(dt);

        if (logger != null) {
            for (Drone d : drones) logger.record(d);
        }
    }

    public List<Drone> getDrones() { return this.drones; }
}