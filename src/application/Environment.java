package application;

import application.Drone;
import application.DroneState;
import application.Vector3D;


public class Environment {

    private double xmin, xmax;
    private double ymin, ymax;
    private double zmin, zmax;

    private double wallK;   // repulsive wall force gain
    private boolean useSoftWalls;

    public Environment(
            double xmin, double xmax,
            double ymin, double ymax,
            double zmin, double zmax,
            double wallK,
            boolean useSoftWalls)
    {
        this.xmin = xmin;
        this.xmax = xmax;

        this.ymin = ymin;
        this.ymax = ymax;

        this.zmin = zmin;
        this.zmax = zmax;

        this.wallK = wallK;
        this.useSoftWalls = useSoftWalls;
    }

    public void apply(Drone drone) {

        DroneState s = drone.getState();
        Vector3D pos = s.position;
        Vector3D vel = s.velocity;

        Vector3D envForce = new Vector3D(0,0,0);

        // ------- X bounds ------------
        if (pos.x < xmin) {
            pos.x = xmin;                       // clamp position
            vel.x = 0;                          // stop velocity
            if (useSoftWalls)
                envForce.x += wallK * (xmin - pos.x);
        }
        else if (pos.x > xmax) {
            pos.x = xmax;
            vel.x = 0;
            if (useSoftWalls)
                envForce.x += wallK * (xmax - pos.x);
        }

        // ------- Y bounds ------------
        if (pos.y < ymin) {
            pos.y = ymin;
            vel.y = 0;
            if (useSoftWalls)
                envForce.y += wallK * (ymin - pos.y);
        }
        else if (pos.y > ymax) {
            pos.y = ymax;
            vel.y = 0;
            if (useSoftWalls)
                envForce.y += wallK * (ymax - pos.y);
        }

        // ------- Z bounds ------------
        if (pos.z < zmin) {
            pos.z = zmin;
            vel.z = 0;
            if (useSoftWalls)
                envForce.z += wallK * (zmin - pos.z);
        }
        else if (pos.z > zmax) {
            pos.z = zmax;
            vel.z = 0;
            if (useSoftWalls)
                envForce.z += wallK * (zmax - pos.z);
        }

        // output environment force to drone
        drone.setEnvironmentForce(envForce);
    }
}

