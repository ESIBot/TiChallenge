#ifndef NAVSYS
#define NAVSYS


	
	int navegador(int* goal, int state);
	
	void goToGoal(int* goal);
	
	void avoidObstacles();
	
	void stop();

	void updateOdometry();
	
	void uniToDiff(int* vel_diff);
	
	void setPwm(int* vel_diff);
	

#endif
