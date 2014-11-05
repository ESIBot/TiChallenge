#ifndef BASICS
#define BASICS

	#include <msp430.h>
	#include <math.h>


	//Parámetros físicos del robot
	#define NUMSENSORES	7
	#define L 5 //distancia entre ruedas
	#define R 2.5 //radio de las ruedas
	#define DISPERTICK 0.1

	//Valor para escalar la velocidad de los motores
	#define MOTORPOLOLU750 100

	//Máquina de estados
	#define AVOID 		1
	#define GOTOGOAL	2
	#define ATGOAL		3

	//Histéresis de la maquina de estados
	#define OBJETOLEJANO	25
	#define OBJETOCERCANO	10

	//Constantes de los controladores
	#define AOKP 1
	#define AOKI 0.2
	#define AOKD 0

	#define GTGKP 1
	#define GTGKI 0.2
	#define GTGKD 0


	//Variables globales

	extern int sensor_placement_x[NUMSENSORES];
	extern int sensor_placement_y[NUMSENSORES];
	extern int sensor_placement_theta[NUMSENSORES];


	extern unsigned long ADC_Result[10];
	extern int ir_distances[NUMSENSORES];
	extern int ir_wf_x[NUMSENSORES];
	extern int ir_wf_y[NUMSENSORES];


	extern int ticks[2];
	extern int position[3]; //pos-x pos-y , theta
	extern int vel_uni[2];   //Velocidad del uniciclo v-w
	extern int goal_size[2]; //Iinicializarla a la mitad del tamaño para evitar estar dividiendo en las comparaciones


	extern int err_gtg[2];
	extern int err_ao[2];
	extern int dt; //Tiempo entre ejecuciones de navegador(). Se usa para calcular la integral del error en los controladores


	//Funciones básicas

	void config();

	void applySensorGeometry();

	void getDistances();

#endif
