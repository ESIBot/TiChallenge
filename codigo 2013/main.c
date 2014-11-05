#include "basics.h"
#include "navsys.h"
#include "communication.h"


unsigned long ADC_Result[10];
int ticks[2];

int position[3]; //pos-x pos-y , theta
int vel_uni[2];   //Velocidad del uniciclo v-w

int ir_distances[NUMSENSORES];	//Distancias medidas por los sensores
int ir_wf_x[NUMSENSORES]; 		//Sentido de las agujas del reloj empezando por el de delante. El secundario de delante en última posición
int ir_wf_y[NUMSENSORES];

int err_gtg[2];
int err_ao[2];
int dt; //Tiempo entre ejecuciones de navegador(). Se usa para calcular la integral del error en los controladores



int sensor_placement_x[NUMSENSORES];
int sensor_placement_y[NUMSENSORES];
int sensor_placement_theta[NUMSENSORES];




int main(void) {
   config();


	return 0;
}
