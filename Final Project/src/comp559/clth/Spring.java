package comp559.clth;

import javax.vecmath.Vector3d;

import no.uib.cipr.matrix.Matrix;
import no.uib.cipr.matrix.Vector;

/**
 * Spring class for 599 assignment 1
 * 
 * @author kry
 */
public class Spring {

	Particle p1 = null;
	Particle p2 = null;

	/** Spring stiffness, sometimes written k_s in equations */
	public static double k = 1000;
	/**
	 * Spring damping (along spring direction), sometimes written k_d in equations
	 */
	public static double c = 1;
	/** Rest length of this spring */
	double l0 = 0;

	/**
	 * Creates a spring between two particles
	 * 
	 * @param p1
	 * @param p2
	 */
	public Spring(Particle p1, Particle p2) {
		this.p1 = p1;
		this.p2 = p2;
		recomputeRestLength();
		p1.springs.add(this);
		p2.springs.add(this);
	}

	/**
	 * Computes and sets the rest length based on the original position of the two
	 * particles
	 */
	public void recomputeRestLength() {
		l0 = p1.p0.distance(p2.p0);
	}

	/**
	 * Applies the spring force by adding a force to each particle
	 */
	public void apply() {
		// TODO: Objective 1, FINISH THIS CODE!

		// Spring force
		Vector3d force = new Vector3d();
		double l = p1.p.distance(p2.p);

		force.sub(p2.p, p1.p);
		force.scale(k * (l - l0) / l);

		p1.addForce(force);
		force.scale(-1);
		p2.addForce(force);

		// Damping force
		Vector3d v = new Vector3d();
		v.sub(p2.v, p1.v);

		force.sub(p2.p, p1.p);
		force.scale(1 / l);
		force.scale(c * force.dot(v));

		p1.addForce(force);
		force.scale(-1);
		p2.addForce(force);
	}

	/** TODO: the functions below are for the backwards Euler solver */

	/**
	 * Computes the force and adds it to the appropriate components of the force
	 * vector. (This function is something you might use for a backward Euler
	 * integrator)
	 * 
	 * @param f
	 */
	public void addForce(Vector f) {
		// TODO: Objective 8, FINISH THIS CODE for backward Euler method (probably very
		// simlar to what you did above)

		Vector3d force = new Vector3d();
		double l = p1.p.distance(p2.p);

		force.sub(p2.p, p1.p);
		force.scale(k * (l - l0) / l);

		f.add(p1.index * 3, force.x);
		f.add(p1.index * 3 + 1, force.y);
		f.add(p1.index * 3 + 2, force.z);
		f.add(p2.index * 3, -force.x);
		f.add(p2.index * 3 + 1, -force.y);
		f.add(p2.index * 3 + 2, -force.z);

		Vector3d v = new Vector3d();
		v.sub(p2.v, p1.v);

		force.sub(p2.p, p1.p);
		force.scale(1 / l);

		force.scale(c * force.dot(v));

		f.add(p1.index * 3, force.x);
		f.add(p1.index * 3 + 1, force.y);
		f.add(p1.index * 3 + 2, force.z);
		f.add(p2.index * 3, -force.x);
		f.add(p2.index * 3 + 1, -force.y);
		f.add(p2.index * 3 + 2, -force.z);

	}

	/**
	 * Adds this springs contribution to the stiffness matrix
	 * 
	 * @param dfdx
	 */
	public void addDfdx(Matrix dfdx) {
		// TODO: Objective 8, FINISH THIS CODE... necessary for backward euler
		// integration

		Vector3d force = new Vector3d();
		double l = p1.p.distance(p2.p);
		force.sub(p2.p, p1.p);

		double dFAdA = (-k * (1 - l0) / l) - (k * l0 / (l * l * l)) * (force.x * force.x + force.y * force.y + force.z * force.z);

		dfdx.add(p1.index * 3, p1.index * 3, dFAdA);
		dfdx.add(p2.index * 3, p2.index * 3, dFAdA);
		dfdx.add(p1.index * 3 + 1, p1.index * 3 + 1, dFAdA);
		dfdx.add(p2.index * 3 + 1, p2.index * 3 + 1, dFAdA);
		dfdx.add(p1.index * 3 + 2, p1.index * 3 + 2, dFAdA);
		dfdx.add(p2.index * 3 + 2, p2.index * 3 + 2, dFAdA);

		dfdx.add(p1.index * 3, p2.index * 3, -dFAdA);
		dfdx.add(p2.index * 3, p1.index * 3, -dFAdA);
		dfdx.add(p1.index * 3 + 1, p2.index * 3 + 1, -dFAdA);
		dfdx.add(p2.index * 3 + 1, p1.index * 3 + 1, -dFAdA);
		dfdx.add(p1.index * 3 + 2, p2.index * 3 + 2, -dFAdA);
		dfdx.add(p2.index * 3 + 2, p1.index * 3 + 2, -dFAdA);

	}

	/**
	 * Adds this springs damping contribution to the implicit damping matrix
	 * 
	 * @param dfdv
	 */
	public void addDfdv(Matrix dfdv) {
		// TODO: Objective 8, FINISH THIS CODE... necessary for backward Euler
		// integration

		Vector3d force = new Vector3d();
		double l = p1.p.distance(p2.p);
		force.sub(p2.p, p1.p);

		double dFAcdA = -c / (l * l) * (force.x * force.x + force.y * force.y + force.z * force.z);

		dfdv.add(p1.index * 3, p1.index * 3, dFAcdA);
		dfdv.add(p2.index * 3, p2.index * 3, dFAcdA);
		dfdv.add(p1.index * 3 + 1, p1.index * 3 + 1, dFAcdA);
		dfdv.add(p2.index * 3 + 1, p2.index * 3 + 1, dFAcdA);
		dfdv.add(p1.index * 3 + 2, p1.index * 3 + 2, dFAcdA);
		dfdv.add(p2.index * 3 + 2, p2.index * 3 + 2, dFAcdA);

		dfdv.add(p1.index * 3, p2.index * 3, -dFAcdA);
		dfdv.add(p2.index * 3, p1.index * 3, -dFAcdA);
		dfdv.add(p1.index * 3 + 1, p2.index * 3 + 1, -dFAcdA);
		dfdv.add(p2.index * 3 + 1, p1.index * 3 + 1, -dFAcdA);
		dfdv.add(p1.index * 3 + 1, p2.index * 3 + 2, -dFAcdA);
		dfdv.add(p2.index * 3 + 1, p1.index * 3 + 2, -dFAcdA);

	}

}
