package comp559.clth;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

import javax.swing.JButton;
import javax.swing.JPanel;

import com.jogamp.opengl.GLAutoDrawable;

import mintools.parameters.BooleanParameter;
import mintools.parameters.DoubleParameter;
import mintools.parameters.IntParameter;
import mintools.swing.HorizontalFlowPanel;
import mintools.swing.VerticalFlowPanel;
import mintools.viewer.EasyViewer;
import mintools.viewer.Interactor;
import mintools.viewer.SceneGraphNode;

/**
 * Provided code for particle system simulator. This class provides the mouse
 * interface for clicking and dragging particles, and the code to draw the
 * system. When the simulator is running system.advanceTime is called to
 * numerically integrate the system forward.
 * 
 * @author kry
 */
public class FinalProjectApp implements SceneGraphNode, Interactor {

	private EasyViewer ev;

	private Cloth cloth;

	/**
	 * Entry point for application
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		new FinalProjectApp();
	}

	/**
	 * Creates the application / scene instance
	 */
	public FinalProjectApp() {
		cloth = new Cloth();
		ev = new EasyViewer("COMP 559 - Final Project", this, new Dimension(640, 360), new Dimension(640, 480));
		ev.addInteractor(this);
	}

	@Override
	public void init(GLAutoDrawable drawable) {

	}

	@Override
	public void display(GLAutoDrawable drawable) {
		if (run.getValue()) {
			for (int i = 0; i < substeps.getValue(); i++) {
				cloth.advanceTime(stepsize.getValue() / substeps.getValue());
			}
		}
		cloth.display(drawable);
	}

	private BooleanParameter run = new BooleanParameter("simulate", false);
	private DoubleParameter stepsize = new DoubleParameter("step size", 0.05, 1e-5, 1);
	private IntParameter substeps = new IntParameter("sub steps", 1, 1, 100);

	@Override
	public JPanel getControls() {
		VerticalFlowPanel vfp = new VerticalFlowPanel();
		JButton create1 = new JButton("create test system 1");
		vfp.add(create1);
		create1.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				cloth.createSystem(1);
			}
		});

		JButton create2 = new JButton("create test system 2");
		vfp.add(create2);
		create2.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				cloth.createSystem(2);
			}
		});

		JButton create3 = new JButton("create test system 3");
		vfp.add(create3);
		create3.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				cloth.createSystem(3);
			}
		});
		
		JButton create4 = new JButton("create test system 4");
		vfp.add(create4);
		create4.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				cloth.createSystem(4);
			}
		});
		
		JButton create5 = new JButton("create test system 5");
		vfp.add(create5);
		create5.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				cloth.createSystem(5);
			}
		});

		HorizontalFlowPanel hfp1 = new HorizontalFlowPanel();
		JButton res2 = new JButton("1280x720");
		hfp1.add(res2);
		res2.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				ev.glCanvas.setSize(1280, 720);
				ev.frame.setSize(ev.frame.getPreferredSize());
			}
		});
		JButton res1 = new JButton("640x360");
		hfp1.add(res1);
		res1.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				ev.glCanvas.setSize(640, 360);
				ev.frame.setSize(ev.frame.getPreferredSize());
			}
		});
		vfp.add(hfp1.getPanel());

		vfp.add(run.getControls());
		vfp.add(stepsize.getSliderControls(true));
		vfp.add(substeps.getControls());
		vfp.add(cloth.getControls());

		return vfp.getPanel();
	}

	@Override
	public void attach(Component component) {

		component.addKeyListener(new KeyAdapter() {
			@Override
			public void keyPressed(KeyEvent e) {
				if (e.getKeyCode() == KeyEvent.VK_SPACE) {
					run.setValue(!run.getValue());
				} else if (e.getKeyCode() == KeyEvent.VK_S) {
					for (int i = 0; i < substeps.getValue(); i++) {
						cloth.advanceTime(stepsize.getValue() / substeps.getValue());
					}
				} else if (e.getKeyCode() == KeyEvent.VK_R) {
					cloth.resetParticles();
				} else if (e.getKeyCode() == KeyEvent.VK_C) {
					cloth.clearParticles();
				} else if (e.getKeyCode() == KeyEvent.VK_6) {
					cloth.explicit.setValue(false);
				} else if (e.getKeyCode() == KeyEvent.VK_1) {
					cloth.createSystem(1);
				} else if (e.getKeyCode() == KeyEvent.VK_2) {
					cloth.createSystem(2);
				} else if (e.getKeyCode() == KeyEvent.VK_3) {
					cloth.createSystem(3);
				} else if (e.getKeyCode() == KeyEvent.VK_4) {
					cloth.createSystem(4);
				} else if (e.getKeyCode() == KeyEvent.VK_5) {
					cloth.createSystem(5);
				} else if (e.getKeyCode() == KeyEvent.VK_ESCAPE) {
					ev.stop();
				} else if (e.getKeyCode() == KeyEvent.VK_UP) {
					substeps.setValue(substeps.getValue() + 1);
				} else if (e.getKeyCode() == KeyEvent.VK_DOWN) {
					substeps.setValue(substeps.getValue() - 1);
				} else if (e.getKeyCode() == KeyEvent.VK_W) {
					cloth.wind.setValue(!cloth.wind.getValue());
				} else if (e.getKeyCode() == KeyEvent.VK_T) {
					cloth.showTriangles.setValue(!cloth.showTriangles.getValue());
				} else if (e.getKeyCode() == KeyEvent.VK_P) {
					cloth.showParticles.setValue(!cloth.showParticles.getValue());
				} else if (e.getKeyCode() == KeyEvent.VK_Q) {
					cloth.showSprings.setValue(!cloth.showSprings.getValue());
				}
				
				
				if (e.getKeyCode() != KeyEvent.VK_ESCAPE)
					ev.redisplay();
			}
		});
	}
}
