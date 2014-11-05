#include "navsys.h"
#include "basics.h"







//********************************************************************************************
//******navegador() -> Función principal. LLeva el robot a la posición indicada***************
//********************************************************************************************

int navegador(int* goal, int state){
	int i = 0;
	int vel_diff[2];
	int objeto_lejano , objeto_cercano , at_goal;


	updateOdometry();
	getDistances();
	applySensorGeometry();


	objeto_lejano = 0;
	objeto_cercano = 0;
	for(i=0;i<NUMSENSORES;i++){
		if(ir_distances[i] < OBJETOLEJANO)
			objeto_lejano = 1;
		if(ir_distances[i] < OBJETOCERCANO)
			objeto_cercano = 1;
	}

	at_goal = 0;
	if(   ( ((position[1]-goal[1]) <= goal_size[1]) || ((position[1]-goal[1]) >= -goal_size[1]) )
	&&    ( ((position[0]-goal[0]) <= goal_size[0]) || ((position[0]-goal[0]) >= -goal_size[0]) )  ){
		at_goal = 1;
	} //La meta la consideramos un rectángulo cuyas medidas se guardan en goal_size


	if( at_goal ){
		state = ATGOAL;
		stop();

	}else if (state == GOTOGOAL){
		if( objeto_cercano){
			state = AVOID;
			avoidObstacles();
		}else{
			state = GOTOGOAL;
			goToGoal(goal);
		}
	}else if(state == AVOID){
		if( objeto_lejano ){
			state = AVOID;
			avoidObstacles();
		}else{
			state = GOTOGOAL;
			goToGoal(goal);
		}
	}

	uniToDiff(vel_diff);
	setPwm(vel_diff);

	return state;
}




//********************************************************************************************
//******goToGoal()
//********************************************************************************************

void goToGoal(int* goal){

	int u_x,u_y;
    int e_k,e_p,e_i,e_d;
    int w;
    int theta_g;


    //Primero calcular el ángulo hacia la meta
    u_x = goal[0]-position[0];
    u_y = goal[1]-position[1];
    theta_g = atan2(u_y,u_x);

    //Calcular error del ángulo y acotarlo entre -pi y pi
    e_k = theta_g-position[2];
    e_k = atan2(sin(e_k),cos(e_k));

    e_p = e_k; //error para la parte proporcional
    e_i = err_gtg[1] + e_k*dt; //error parte integral
    e_d = (e_k-err_gtg[0])/dt; //error parte derivativa

    //Calculo del PID para la velocidad angular
    w = GTGKP*e_p + GTGKI*e_i + GTGKD*e_d;

    //Actualizar el vector de velocidades
    vel_uni[1] = w;
    vel_uni[0] = vel_uni[0];
    //Por el momento la velocidad lineal la mantenemos constante,
    //podríamos por ejemplo frenar poco a poco si nos acercamos a la meta

    //Guardar error para la próxima ejecución
    err_gtg[0] = e_k;
    err_gtg[1] = e_i;



}




//********************************************************************************************
//******avoidObstacles()
//********************************************************************************************


void avoidObstacles(){

	unsigned int i=0;

    int sensor_gains[] = {1, 1, 1, 1, 1, 1};  //Cuanto mas pequeño sea el valor, mas importancia le damos al sensor correspondiente.
    int e_k,e_p,e_i,e_d;
    int w;
    int theta_ao;
    int u_ao_x , u_ao_y;
    int u_x[NUMSENSORES];
    int	u_y[NUMSENSORES];



    u_ao_x = 0;
	u_ao_y = 0;
	for(i=0;i<NUMSENSORES;i++){
	    u_x[i] = sensor_gains[i] * (ir_wf_x[i]-position[0]);
	    u_y[i] = sensor_gains[i] * (ir_wf_y[i]-position[1]);

		u_ao_x += u_x[i];
		u_ao_y += u_y[i];
	}


    theta_ao = atan2(u_ao_y,u_ao_x);// ángulo del vector que apunta al promedio de los obstáculos
    theta_ao += (3.141592);        	// theta_ao debe apuntar a 180º de donde está el obstáculo
    								// Se podría modificar para que apunte a +90 o -90 dependiendo de donde esté la meta

    e_k = theta_ao-position[2];
    e_k = atan2(sin(e_k),cos(e_k));

 	e_p = e_k; //error para la parte proporcional
    e_i = err_ao[1] + e_k*dt; //error parte integral
    e_d = (e_k-err_ao[0])/dt; //error parte derivativa

    w =  AOKP*e_p + AOKI*e_i + AOKD*e_d;

	vel_uni[0] = vel_uni[0]; //En principio mantenemos la velocidad lineal constante
	vel_uni[1] = w;


}


//********************************************************************************************
//******stop()
//********************************************************************************************


void  stop(){
	vel_uni[0]=0;
	vel_uni[1]=0;

}



//********************************************************************************************
//******updateOdometry()
//********************************************************************************************

void  updateOdometry(){

	int d_left,d_right;
	int phi ,d_center ;
	int x_dt,y_dt,theta_dt;

      //Primero calculamos la distancia recorrida por cada rueda
	d_left  = (ticks[0])*DISPERTICK;
    d_right = (ticks[1])*DISPERTICK;

    ticks[0] = 0;
    ticks[1] = 0;

	//Distacia recorrida por el centro y giro realizado
	d_center = (d_right + d_left)/2;
	phi = (d_right - d_left)/L;

	//Calculamos la variación de la posición (x,y,dirección)
	x_dt = d_center*cos(position[2]);
	y_dt = d_center*sin(position[2]);
	theta_dt = phi;

	//Actualizamos la posición
	position[0]+= x_dt;
	position[1]+= y_dt;
	position[2]+= theta_dt;

}



//********************************************************************************************
//******uniToDiff()
//********************************************************************************************

void uniToDiff(int* vel_diff){

  vel_diff[0] = (2*vel_uni[0]-vel_uni[1]*L)/(2*R);
  vel_diff[1] = (2*vel_uni[0]+vel_uni[1]*L)/(2*R);
}

//********************************************************************************************
//******setPWM()
//********************************************************************************************


void setPWM(int* vel_diff){

//	#ifdef POLOLU
	  	TA1CCR1 = vel_diff[0]*MOTORPOLOLU750;
	  	TA1CCR2 = vel_diff[1]*MOTORPOLOLU750;
//	#endif
	/*
		#ifdef MAXON
			TA1CCR1 = vel_diff[0]*MOTORMAXON;
			TA1CCR2 = vel_diff[1]*MOTORMAXON;
		#endif
	*/
}
