package comp559.clth;

import java.util.LinkedList;
import java.util.List;

import javax.swing.JPanel;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.GLAutoDrawable;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.SceneGraphNode;
import no.uib.cipr.matrix.DenseMatrix;
import no.uib.cipr.matrix.DenseVector;
import no.uib.cipr.matrix.Matrix;
import no.uib.cipr.matrix.Vector;

public class Cloth implements SceneGraphNode, Filter, Function {
	Particle[][] cloth;
	List<Spring> springs = new LinkedList<Spring>();
	List<Triangle> triangles = new LinkedList<Triangle>();

	boolean collideSphere = false;
	boolean collideTable = false;
	boolean windSystem = false;
	boolean cornerPin = false;

	int length;
	int width;

	Cloth() {

	}

	public void createSystem(int i) {
		clearParticles();

		collideSphere = false;
		collideTable = false;
		windSystem = false;
		cornerPin = false;

		if (i == 1) {
			collideSphere = true;
			loc = new Point3d(5, 10, 5);
			createCloth(10, 10);
		} else if (i == 2) {
			collideTable = true;
			createCloth(20, 20);
		} else if (i == 3) {
			windSystem = true;
			createCloth(10, 10);
		} else if (i == 4) {
			cornerPin = true;
			createCloth(10, 10);
		} else if (i == 5) {
			collideSphere = true;
			loc = new Point3d(9.5, 13, 9.5);
			createCloth(20, 20);
		}
	}

	public void createCloth(int x, int y) {
		this.length = x;
		this.width = y;

		cloth = new Particle[x][y];
		int particleIndex = 0;
		// Initializes grid of particles
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < width; j++) {
				double xLoc = i;
				double yLoc = j;
				cloth[i][j] = new Particle(xLoc, 20, yLoc, 0, 0, 0);
				cloth[i][j].index = particleIndex;
				particleIndex++;
			}
		}

		if (windSystem) {
			cloth[0][0].pinned = true;
			cloth[x - 1][0].pinned = true;
		}

		if (cornerPin) {
			cloth[0][0].pinned = true;
			cloth[0][y - 1].pinned = true;
			cloth[x - 1][0].pinned = true;
			cloth[x - 1][y - 1].pinned = true;
		}

		for (int i = 0; i < length; i++) {
			for (int j = 0; j < width; j++) {
				// Stretch springs
				if (j + 1 < width) {
					springs.add(new Spring(cloth[i][j], cloth[i][j + 1]));
				}
				if (i + 1 < length) {
					springs.add(new Spring(cloth[i][j], cloth[i + 1][j]));
				}

				// Shear springs
				if (i + 1 < length && j + 1 < width) {
					springs.add(new Spring(cloth[i][j], cloth[i + 1][j + 1]));
				}
				if (i + 1 < length && j - 1 >= 0) {
					springs.add(new Spring(cloth[i][j], cloth[i + 1][j - 1]));
				}

				// Bend springs
				if (bendSprings.getValue()) {
					if (j + 2 < width) {
						springs.add(new Spring(cloth[i][j], cloth[i][j + 2]));
					}
					if (i + 2 < length) {
						springs.add(new Spring(cloth[i][j], cloth[i + 2][j]));
					}
				}
			}
		}

		for (int i = 0; i < length - 1; i++) {
			for (int j = 0; j < width - 1; j++) {
				triangles.add(new Triangle(cloth[i][j], cloth[i + 1][j + 1], cloth[i + 1][j]));
				triangles.add(new Triangle(cloth[i][j], cloth[i + 1][j + 1], cloth[i][j + 1]));
			}
		}

		int N = length * width;
		// create matrix and vectors for solve
		CG = new ConjugateGradientMTJ(3 * N);
		CG.setFilter(this);
		A = new DenseMatrix(3 * N, 3 * N);
		dfdx = new DenseMatrix(3 * N, 3 * N);
		dfdv = new DenseMatrix(3 * N, 3 * N);
		deltaxdot = new DenseVector(3 * N);
		b = new DenseVector(3 * N);
		f = new DenseVector(3 * N);
		xdot = new DenseVector(3 * N);

	}

	public Particle[][] getParticles() {
		return cloth;
	}

	public List<Spring> getSprings() {
		return springs;
	}

	public void clearParticles() {
		length = 0;
		width = 0;
		cloth = null;
		springs.clear();
		triangles.clear();
	}

	public void resetParticles() {
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < width; j++) {
				if (cloth[i][j] != null) {
					cloth[i][j].reset();
				}
			}
		}
		time = 0;
	}

	public void init(GLAutoDrawable drawable) {

	}

	/**
	 * Evaluates derivatives for ODE integration.
	 * 
	 * @param t    time
	 * @param p    phase space state
	 * @param dydt to be filled with the derivative
	 */
	@Override
	public void derivs(double t, double[] p, double[] dpdt) {
		setPhaseSpace(p);

		int i = 0;
		for (int j = 0; j < length; j++) {
			for (int k = 0; k < width; k++) {
				Particle p1 = cloth[j][k];
				p1.clearForce();

				p1.f.y += -gravity.getFloatValue() * p1.mass;

				p1.f.x += -viscousDamping.getFloatValue() * p1.v.x;
				p1.f.y += -viscousDamping.getFloatValue() * p1.v.y;
				p1.f.z += -viscousDamping.getFloatValue() * p1.v.z;
			}
		}

		for (Spring spring : springs) {
			spring.apply();
		}

		for (int j = 0; j < length; j++) {
			for (int k = 0; k < width; k++) {
				Particle p1 = cloth[j][k];
				dpdt[i++] = p1.v.x;
				dpdt[i++] = p1.v.y;
				dpdt[i++] = p1.v.z;
				dpdt[i++] = p1.f.x / p1.mass;
				dpdt[i++] = p1.f.y / p1.mass;
				dpdt[i++] = p1.f.z / p1.mass;
			}
		}

	}

	public Integrator integrator = new RK4();

	public double[] state = new double[1];
	public double[] stateOut = new double[1];

	private ConjugateGradientMTJ CG;
	private DenseMatrix A;
	private DenseMatrix dfdx;
	private DenseMatrix dfdv;
	private DenseVector deltaxdot;
	private DenseVector b;
	private DenseVector f;
	private DenseVector xdot;

	public void advanceTime(double elapsed) {
		Spring.k = springStiffness.getValue();
		Spring.c = springDamping.getValue();

		if (cloth == null) {
			return;
		}

		int n = getPhaseSpaceDim();

		if (n != state.length) {
			state = new double[n];
			stateOut = new double[n];
		}

		if (explicit.getValue()) {
			getPhaseSpace(state);
			integrator.step(state, n, time, elapsed, stateOut, this);
			setPhaseSpace(stateOut);
		} else {
			getPhaseSpace(state);
			backwardsEuler(state, n, time, elapsed, stateOut);
			setPhaseSpace(stateOut);
		}
		time = time + elapsed;
		postStepFix();

	}

	private Matrix M;

	private void backwardsEuler(double[] p, int n, double t, double h, double[] pout) {
		int N = length * width;

		getVelocities(xdot);

		for (int i = 0; i < length; i++) {
			for (int j = 0; j < width; j++) {
				Particle p1 = cloth[i][j];
				f.add(p1.index * 3 + 1, -gravity.getFloatValue() * p1.mass);

				if (wind.getValue()) {
					// Wind forces
					f.add(p1.index * 3, 3 * Math.sin(p1.p.x + t));
					f.add(p1.index * 3 + 1, 10 * Math.abs(Math.sin(p1.p.y + t)));
					f.add(p1.index * 3 + 2, 10 * Math.abs(Math.sin(p1.p.z + t)));
				}

				f.add(p1.index * 3, -viscousDamping.getFloatValue() * p1.v.x);
				f.add(p1.index * 3 + 1, -viscousDamping.getFloatValue() * p1.v.y);
				f.add(p1.index * 3 + 2, -viscousDamping.getFloatValue() * p1.v.z);
			}
		}

		for (Spring s : springs) {
			s.addForce(f);
			s.addDfdx(dfdx);
			s.addDfdv(dfdv);
		}

		M = new DenseMatrix(3 * N, 3 * N);
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < width; j++) {
				Particle p1 = cloth[i][j];
				M.set(p1.index * 3, p1.index * 3, p1.mass);
				M.set(p1.index * 3 + 1, p1.index * 3 + 1, p1.mass);
				M.set(p1.index * 3 + 2, p1.index * 3 + 2, p1.mass);
			}
		}

		DenseVector tmp = new DenseVector(3 * N);
		A = new DenseMatrix(3 * N, 3 * N);

		dfdx.mult(xdot, tmp);
		A.set(M.add(dfdv.scale(-h)).add(dfdx.scale(-h * h)));
		b.set(f.add(tmp.scale(h)).scale(h));

		// Last value is iterations
		CG.solve(A, b, deltaxdot, 10);

		for (int i = 0; i < stateOut.length; i += 6) {
			int j = i / 2;
			pout[i] = h * (deltaxdot.get(j) + state[i + 3]) + state[i];
			pout[i + 1] = h * (deltaxdot.get(j + 1) + state[i + 4]) + state[i + 1];
			pout[i + 2] = h * (deltaxdot.get(j + 2) + state[i + 5]) + state[i + 2];

			pout[i + 3] = deltaxdot.get(j) + state[i + 3];
			pout[i + 4] = deltaxdot.get(j + 1) + state[i + 4];
			pout[i + 5] = deltaxdot.get(j + 2) + state[i + 5];
		}
	}

	private void getPhaseSpace(double[] state) {
		int count = 0;
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < width; j++) {
				Particle p = cloth[i][j];
				state[count++] = p.p.x;
				state[count++] = p.p.y;
				state[count++] = p.p.z;

				state[count++] = p.v.x;
				state[count++] = p.v.y;
				state[count++] = p.v.z;
			}
		}
	}

	private void setPhaseSpace(double[] state) {
		int count = 0;
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < width; j++) {
				Particle p = cloth[i][j];
				p.p.x = state[count++];
				p.p.y = state[count++];
				p.p.z = state[count++];

				p.v.x = state[count++];
				p.v.y = state[count++];
				p.v.z = state[count++];
			}
		}
	}

	private void getVelocities(DenseVector xd) {
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < width; j++) {
				Particle p = cloth[i][j];
				int index = p.index * 3;
				if (p.pinned) {
					xd.set(index, 0);
					xd.set(index + 1, 0);
					xd.set(index + 2, 0);
				} else {
					xd.set(index, p.v.x);
					xd.set(index + 1, p.v.y);
					xd.set(index + 2, p.v.z);
				}
			}
		}
	}

	public void postStepFix() {
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < width; j++) {
				Particle p = cloth[i][j];
				if (p.pinned) {
					p.v.set(0, 0, 0);
				}
			}
		}
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < width; j++) {
				Particle p = cloth[i][j];
				// Set height to 0.1 so the triangles still appear above the ground plane
				if (p.p.y <= 0.1) {
					p.p.y = 0.1;
					p.v.x = 0;
					p.v.z = 0;
					p.f.set(0, 0, 0);
				}
			}
		}

		if (collideTable == true) {
			tableCollision();
		}
		if (collideSphere == true) {
			sphereCollision();
		}

	}

	public int getPhaseSpaceDim() {
		return length * width * 6;
	}

	@Override
	public void filter(Vector v) {
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < width; j++) {
				Particle p = cloth[i][j];
				if (!p.pinned)
					continue;
				v.set(p.index * 3 + 0, 0);
				v.set(p.index * 3 + 1, 0);
				v.set(p.index * 3 + 2, 0);
			}
		}
	}

	Point3d loc = new Point3d(5, 10, 4);
	double radius = 6.0;

	private void sphereCollision() {
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < width; j++) {
				Particle p = cloth[i][j];
				if (p.p.distance(loc) < radius) {
					Vector3d dir = new Vector3d();
					dir.sub(p.p, loc);
					dir.normalize();
					dir.scale(radius);
					p.p.add(loc, dir);
					p.v.x = 0;
					p.v.y = 0;
				}
			}
		}
	}

	private void tableCollision() {
		for (int i = 0; i < length; i++) {
			for (int j = 0; j < width; j++) {
				Particle p = cloth[i][j];

				double y = 20;
				double xMin = 5;
				double xMax = 15;

				double zMin = 5;
				double zMax = 15;

				if (p.p.y <= y && p.p.x <= xMax && p.p.x >= xMin && p.p.z <= zMax && p.p.z >= zMin) {
					p.v.scale(-1);
					p.v.normalize();

					double dxMax = Math.abs(p.p.x - xMax);
					double dxMin = Math.abs(p.p.x - xMin);
					double dy = Math.abs(p.p.y - y);
					double dzMax = Math.abs(p.p.z - zMax);
					double dzMin = Math.abs(p.p.z - zMin);

					if (Math.min(dxMax, Math.min(dxMin, Math.min(dy, Math.min(dzMax, dzMin)))) == dxMax) {
						p.p.x = xMax;
					} else if (Math.min(dxMax, Math.min(dxMin, Math.min(dy, Math.min(dzMax, dzMin)))) == dxMin) {
						p.p.x = xMin;
					} else if (Math.min(dxMax, Math.min(dxMin, Math.min(dy, Math.min(dzMax, dzMin)))) == dy) {
						p.p.y = y;
					} else if (Math.min(dxMax, Math.min(dxMin, Math.min(dy, Math.min(dzMax, dzMin)))) == dzMax) {
						p.p.z = zMax;
					} else {
						p.p.z = zMin;
					}
				}
			}
		}
	}

	public double time = 0;

	public void display(GLAutoDrawable drawable) {
		GL2 gl = drawable.getGL().getGL2();

//		gl.glRotatef(-30, 0, 1, 0);
		gl.glTranslatef(0, -25, 0);
//		gl.glTranslatef(0, 0, -120);

		// Draw Particles
		if (showParticles.getValue()) {
			gl.glDisable(GL2.GL_LIGHTING);
			gl.glPointSize(5);
			gl.glBegin(GL.GL_POINTS);
			for (int i = 0; i < length; i++) {
				for (int j = 0; j < width; j++) {
					Particle p = cloth[i][j];
					double alpha = 0.5;
					if (p.pinned) {
						gl.glColor4d(1.0f, 0, 0, 1);
					} else {
						gl.glColor4d(0, 0, 1, alpha);
					}
					gl.glVertex3d(p.p.x, p.p.y, p.p.z);
				}
			}
			gl.glEnd();
			gl.glEnable(GL2.GL_LIGHTING);
		}

		// Draw floor
		gl.glDisable(GL2.GL_LIGHTING);
		gl.glColor4d(1, 1, 1, 0.5);
		gl.glBegin(GL2.GL_QUADS);
		gl.glVertex3d(50, 0, 50);
		gl.glVertex3d(50, 0, -50);
		gl.glVertex3d(-50, 0, -50);
		gl.glVertex3d(-50, 0, 50);
		gl.glEnd();
		gl.glEnable(GL2.GL_LIGHTING);

		// Draw triangle cloth
		if (showTriangles.getValue()) {
			float[] colour = { 0, 1, 0, 1 };
			gl.glMaterialfv(GL.GL_FRONT, GL2.GL_DIFFUSE, colour, 0);
			gl.glBegin(GL.GL_TRIANGLES);
			int k = 0;
			for (Triangle t : triangles) {
				Vector3d v1 = new Vector3d();
				Vector3d v2 = new Vector3d();
				Vector3d norm = new Vector3d();
				v1.sub(t.p2.p, t.p1.p);
				v2.sub(t.p3.p, t.p1.p);
				norm.cross(v1, v2);

				// Fix normal direction for uniform colour on cloth
				if (k % 2 == 1) {
					norm.scale(-1);
				}

				gl.glNormal3d(norm.x, norm.y, norm.z);
				gl.glColor4d(0, 1, 0, 1);
				gl.glVertex3d(t.p1.p.x, t.p1.p.y, t.p1.p.z);
				gl.glVertex3d(t.p2.p.x, t.p2.p.y, t.p2.p.z);
				gl.glVertex3d(t.p3.p.x, t.p3.p.y, t.p3.p.z);
				k++;
			}
			gl.glEnd();
		}

		// Draw springs
		if (showSprings.getValue()) {
			gl.glDisable(GL2.GL_LIGHTING);
			gl.glColor4d(0, 0.5, 0.5, 1);
			gl.glLineWidth(2f);
			gl.glBegin(GL.GL_LINES);
			for (Spring s : springs) {
				gl.glVertex3d(s.p1.p.x, s.p1.p.y, s.p1.p.z);
				gl.glVertex3d(s.p2.p.x, s.p2.p.y, s.p2.p.z);
			}
			gl.glEnd();
			gl.glEnable(GL2.GL_LIGHTING);
		}

	}

	public DoubleParameter gravity = new DoubleParameter("gravity", 9.8, 0.01, 1000);
	public DoubleParameter springStiffness = new DoubleParameter("spring stiffness", 300, 0, 10000);
	public DoubleParameter springDamping = new DoubleParameter("spring damping", 0, 0, 50);
	public DoubleParameter viscousDamping = new DoubleParameter("viscous damping", 0, 0, 10);
	public IntParameter iterations = new IntParameter("iterations", 100, 1, 100);
	/** controls weather explicit or implicit integration is used */
	public BooleanParameter explicit = new BooleanParameter("explicit ([s] to step)", false);
	public BooleanParameter showParticles = new BooleanParameter("show particles [p]", true);
	public BooleanParameter showSprings = new BooleanParameter("show springs [q]", true);
	public BooleanParameter bendSprings = new BooleanParameter("use bend springs (toggle before creating system) [b]",
			false);
	public BooleanParameter showTriangles = new BooleanParameter("show triangles (cloth) [t]", true);
	public BooleanParameter wind = new BooleanParameter("add wind [w]", false);
	public int computeTime;

	@Override
	public JPanel getControls() {
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		vfp.add(gravity.getSliderControls(true));
		vfp.add(springStiffness.getSliderControls(false));
		vfp.add(springDamping.getSliderControls(false));
		vfp.add(viscousDamping.getSliderControls(false));
		vfp.add(iterations.getSliderControls());
		vfp.add(explicit.getControls());
		vfp.add(showParticles.getControls());
		vfp.add(showSprings.getControls());
		vfp.add(showTriangles.getControls());
		vfp.add(bendSprings.getControls());
		vfp.add(wind.getControls());
		return vfp.getPanel();
	}
}
