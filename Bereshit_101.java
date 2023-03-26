package simulation;
import simulation.CSVWriter;
import simulation.Moon;

import java.util.ArrayList;
import java.util.Arrays;
import simulation.PDController;
/**
 * This class represents the basic flight controller of the Bereshit space craft.
 * @author ben-moshe
 *
 */
public class Bereshit_101 {
	public static final double WEIGHT_EMP = 165; // kg
	public static final double WEIGHT_FULE = 420; // kg
	public static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE; // kg
// https://davidson.weizmann.ac.il/online/askexpert/%D7%90%D7%99%D7%9A-%D7%9E%D7%98%D7%99%D7%A1%D7%99%D7%9D-%D7%97%D7%9C%D7%9C%D7%99%D7%AA-%D7%9C%D7%99%D7%A8%D7%97
	public static final double MAIN_ENG_F = 430; // N
	public static final double SECOND_ENG_F = 25; // N
	public static final double MAIN_BURN = 0.15; //liter per sec, 12 liter per m'
	public static final double SECOND_BURN = 0.009; //liter per sec 0.6 liter per m'
	public static final double ALL_BURN = MAIN_BURN + 8*SECOND_BURN;
	double vs = 24.8;
	double hs = 932;
	double dist = 181*1000;
	double ang = 58.3; // zero is vertical (as in landing)
	double alt = 13748; // 2:25:40 (as in the simulation) // https://www.youtube.com/watch?v=JJ0VfRL9AMs
	double time = 0;
	double dt =1; // sec
	double acc=0; // Acceleration rate (m/s^2)
	double fuel = 121; //
	double weight = WEIGHT_EMP + fuel;
	double NN = 0.7;

	PDController pdVS = new PDController(0.04,0,0.2);
	PDController pdANG = new PDController(0.04,0.00003,0.2);
	PDController pdHS = new PDController(0.04,0.00003,0.2);
	ArrayList<ArrayList<String>> landingData = new ArrayList<>();
	public static double accMax(double weight) {
		return acc(weight, true,8);
	}
	public static double acc(double weight, boolean main, int seconds) {
		double t = 0;
		if(main) {t += MAIN_ENG_F;}
		t += seconds*SECOND_ENG_F;
		double ans = t/weight;
		return ans;
	}

	public void updateState() {
		double ang_rad = Math.toRadians(ang);
		double h_acc = Math.sin(ang_rad)*acc;
		double v_acc = Math.cos(ang_rad)*acc;
		double vacc = Moon.getAcc(hs);
		time+=dt;
		double dw = dt*ALL_BURN*NN;
		if(fuel>0) {
			fuel -= dw;
			weight = WEIGHT_EMP + fuel;
			acc = NN* accMax(weight);
		}
		else { // ran out of fuel
			acc=0;
		}

		v_acc -= vacc;
		if(hs>0) {hs -= h_acc*dt;}
		dist -= hs*dt;
		vs -= v_acc*dt;
		alt -= dt*vs;
	}

	public void preLanding(){
		double error = 25 - vs;
		ang = 58.3;
		double pidRes= pdVS.update(error,dt);
		NN = (NN-pidRes>0 && NN-pidRes<1) ? NN-pidRes : NN;
		this.updateState();
	}

	public void landing(){
				if(ang>3) {ang-=3;} // rotate to vertical position.
				else {ang =0;}
				double err = 2 - vs;
				double pidRes = pdVS.update(
						err,
						dt
				);
				NN = (NN-pidRes>0 && NN-pidRes<1) ? NN-pidRes : NN;
				if(hs<2) {hs=0;}
				if(alt<125) { // very close to the ground!
					if(vs <=2.5 && hs <=2.5){
						this.updateState();
						return;
					}
					double NNPIDRES = pdHS.update(1-NN,dt);
					NN = (NN-NNPIDRES>0 && NN-NNPIDRES<1) ? NN-NNPIDRES : NN;
					if(vs<5) {
						NNPIDRES = pdHS.update(0.85-NN,dt);
						NN = (NN-NNPIDRES>0 && NN-NNPIDRES<1) ? NN-NNPIDRES : NN;
					} // if it is slow enough - go easy on the brakes
				}
				this.updateState();
	}

	public void startLog() {
		landingData.add(new ArrayList<String>(Arrays.asList("Time","Vertical Speed", "Horizontal Speed", "Distance", "Altitude","Angle", "Weight", "Acceleration", "Fuel")));
	}


	public void log(){
		landingData.add(new ArrayList<String>(Arrays.asList(time+"",vs+"",hs+"",dist+"",alt+"",ang+"",weight+"",acc+"",fuel+"")));
	}

	public static void main(String[] args) {
		ArrayList<ArrayList<String>> landingData = new ArrayList<>();
		System.out.println("Simulating Bereshit's Landing:");
		System.out.println("time, vs, hs, dist, alt, ang,weight,acc");
		Bereshit_101 spaceCraft = new Bereshit_101();
		spaceCraft.startLog();
		// ***** main simulation loop ******
		while(spaceCraft.alt>0) {
			if (spaceCraft.time % 10 == 0 || spaceCraft.alt < 100) {
				System.out.println(spaceCraft.time + "," + spaceCraft.vs + "," + spaceCraft.hs + "," + spaceCraft.dist + "," + spaceCraft.alt + "," + spaceCraft.ang + "," + spaceCraft.weight + "," + spaceCraft.acc + "," + spaceCraft.fuel);
				spaceCraft.log();
			}
			if(spaceCraft.alt>400){
				spaceCraft.preLanding();
			}else{
				spaceCraft.landing();
			}
		}
		CSVWriter writer = new CSVWriter();
		try {
			writer.writeToCSV(spaceCraft.landingData);
		}catch (Exception e){
			e.printStackTrace();
		}
	}
}

