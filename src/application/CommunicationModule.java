package application;



import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**

 */
public class CommunicationModule {

    private double commRange;          // max communication distance
    private double lossProbability;    // probability of message loss

    private Random rng = new Random();

    public CommunicationModule(double commRange, double lossProbability) {
        this.commRange = commRange;
        this.lossProbability = lossProbability;//ploss
    }

  
    public List<Drone> getNeighbors(Drone d, List<Drone> allDrones) {

        List<Drone> neighbors = new ArrayList<>();

        DroneState si = d.getState();
        Vector3D pos_i = si.position;

        for (Drone other : allDrones) {

            if (other == d) continue;

            DroneState sj = other.getState();
            Vector3D pos_j = sj.position;

            double dist = pos_i.subtract(pos_j).magnitude();

       
            if (dist > commRange) continue;

    
            
            
            //random val generates no with 0to1,
            double randomVal = rng.nextDouble(); // 0 to 1
            if (randomVal < lossProbability) continue;

            // successfully communicated
            neighbors.add(other);
        }

        return neighbors;
    }
}

