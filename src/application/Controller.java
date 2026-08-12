package application;




public class Controller {

    private double kp_pos;   // position gain
    private double kv_vel;   // velocity gain

    private double kOmega;   // angular velocity damping

    private Vector3D targetPosition = new Vector3D(0,0,0);
    private Vector3D targetVelocity = new Vector3D(0,0,0);

    public Controller(double kp_pos, double kv_vel, double kOmega) {
        this.kp_pos = kp_pos;
        this.kv_vel = kv_vel;
        this.kOmega = kOmega;
    }

    //Set target waypoint
    public void setTargetPosition(Vector3D p) {
        this.targetPosition = p;
    }

    public void setTargetVelocity(Vector3D v) {
        this.targetVelocity = v;
    }

    //control computation
    public void computeControl(Drone drone) {

        DroneState s = drone.getState();

        // 1. ep =pdes-currnt p and ev=vdesired -current
        Vector3D ep = targetPosition.subtract(s.position);
        Vector3D ev = targetVelocity.subtract(s.velocity);

        // 2. Desired acceleration
        Vector3D a_cmd =
            ep.multiply(kp_pos)
              .add(ev.multiply(kv_vel))
              .add(s.getGravity());  // gravity compensation

        // 3. Vertical thrust 
        double Tz = s.mass * a_cmd.z;

        //Torque:angular velocity damping
        Vector3D tau = s.angularVelocity.multiply(-kOmega);

        // 
        drone.setControllerInputs(Tz, tau);
    }
}

