package application;



import java.util.List;

/**
 * FormationManager:
 * Computes formation forces for each drone using:
 *
 * F_form_i = - Σ_j [ kp (p_i - p_j) + kv (v_i - v_j) ]
 *
 * Exactly as required in the CEP document.
 */
public class FormationManager {

    private double kp;   // position coupling
    private double kv;   // velocity coupling
    private double commRange;  // max neighbor distance

    public FormationManager(double kp, double kv, double commRange) {
        this.kp = kp;
        this.kv = kv;
        this.commRange = commRange;
    }

    /**
     * Compute formation force for a single drone.
     * INPUT:
     *   drone      = the drone we are computing force for
     *   allDrones  = list of all drones in the fleet
     *
     * OUTPUT:
     *   The force is written into drone.setFormationForce()
     */
    public void computeFormationForce(Drone drone, List<Drone> allDrones) {

        DroneState si = drone.getState();
        Vector3D pos_i = si.position;
        Vector3D vel_i = si.velocity;

        Vector3D totalForce = new Vector3D(0,0,0);

        for (Drone dj : allDrones) {

            if (dj == drone) continue; //skip the already picked 1st drone all made relative to this

            DroneState sj = dj.getState();
            Vector3D pos_j = sj.position;
            Vector3D vel_j = sj.velocity;

            double dist = pos_i.subtract(pos_j).magnitude();
            if (dist > commRange) continue; // not a neighbor maxneighbour dist>dist

          
            Vector3D posDiff = pos_i.subtract(pos_j).multiply(kp);//evaluates kp

        
            Vector3D velDiff = vel_i.subtract(vel_j).multiply(kv);//evaluates kv

            totalForce.addInPlace( posDiff.add(velDiff).multiply(-1) ); // Sum: -[posDiff + velDiff]
        }

        // calls dronestate .setter to set formation force
        drone.setFormationForce(totalForce);
    }
}
