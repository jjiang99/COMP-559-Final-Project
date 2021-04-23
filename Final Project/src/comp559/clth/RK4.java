package comp559.clth;

public class RK4 implements Integrator {
    
    @Override
    public String getName() {
        return "RK4";
    }
    
	private double[] k1;
	private double[] k2;
	private double[] k3;
	private double[] k4;
	private double[] tmp;
    
    @Override
    public void step(double[] p, int n, double t, double h, double[] pout, Function derivs) {
        // TODO: Objective 6, implement the RK4 integration method
    	// see also efficient memory management suggestion in provided code for the Midpoint method.
    	
		if (k1 == null || k1.length != n) {			
			k1 = new double[n];
			k2 = new double[n];
			k3 = new double[n];
			k4 = new double[n];
			tmp = new double[n];
		}
		
		// k1 = f(x0)
		derivs.derivs(t, p, k1);
		
		// tmp = x0 + h/2 * k1
		for (int i = 0; i < n; i++) {
			tmp[i] = p[i] + h/2.0 * k1[i];
		}
		
		// k2 = f(x0 + h/2 * k1)
		derivs.derivs(t + h, tmp, k2);
		
		// tmp = x0 + h/2 * k2
		for (int i = 0; i < n; i++) {
			tmp[i] = p[i] + h/2.0 * k2[i];
		}
		
		// k3 = f(x0 + h/2 * k2)
		derivs.derivs(t + h, tmp, k3);
		
		// tmp = x0 + h * k3
		for (int i = 0; i < n; i++) {
			tmp[i] = p[i] + h * k3[i];
		}
		
		// k4 = f(x0 + h * k3)
		derivs.derivs(t + h, p, k4);
		
		// pout = x0 + h/6 * (k1 + 2 * k2 + 2 * k3 + k4)
		for (int i = 0; i < n; i++) {
			pout[i] = p[i] + h/6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
		}
    	
    }
}
