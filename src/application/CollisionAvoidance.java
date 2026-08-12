package application;



import java.util.List;

/**

 */
public class CollisionAvoidance {

    private double kRep;          // repulsion gain
    private double minDistance;   // repulsion if distance < minDistance

    public CollisionAvoidance(double kRep, double minDistance) {
        this.kRep = kRep;
        this.minDistance = minDistance;
    }

    public void computeRepulsiveForce(Drone drone, List<Drone> allDrones) {

        DroneState si = drone.getState();
        Vector3D pos_i = si.position;

        Vector3D totalRepForce = new Vector3D(0,0,0);

        for (Drone dj : allDrones) {

            if (dj == drone) continue;

            DroneState sj = dj.getState();
            Vector3D pos_j = sj.position;

            Vector3D diff = pos_i.subtract(pos_j);
            double dist = diff.magnitude();

            // Only repel if inside danger threshold
            if (dist < minDistance && dist > 0.0001) {

                // (p_i - p_j) / |p_i - p_j|^2
                Vector3D dir = diff.divide(dist * dist);

                // k_rep * above
                Vector3D repForce = dir.multiply(kRep);

                totalRepForce.addInPlace(repForce);
            }
        }

        //calls setter of drone
        drone.setRepulsiveForce(totalRepForce);
    }
}

