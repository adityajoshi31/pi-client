#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <malloc.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <fstream>
#include <string.h>
#include <stdbool.h>
#include <signal.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

//#include <linux/can.h>
#include <string.h>
#include <arpa/inet.h> //inet_addr
#include <fcntl.h> // for open
// toolbox for databus
#include <iostream> 
#include <sqlite3.h> 
#include <ctime>
#include <curl/curl.h>
#include <modbus/modbus.h>

#define tforder    	2	// order of transfer function
#define coeffnum   	5	// =tforder *2 +1
#define win_size 	20	// window size for parameter identification
#define time_data	1	// (=1/freq_data) sampling time for the data receiving (second)
#define time_ID 	5	// sampling time for the parameter id (second)
#define time_OB 	1	// sampling time for the observer (second)
#define N          	25	// max size of the matrix
#define N_1        	8000	// max size of the matrix
#define ratedcurr	1.36	// 1 C-rate

using namespace std;

// Claim the function used in the program
void print_Modbus_result(); // Print results of Modbus
void print_PI_result();	// Print results of parameter identification
void print_OB_result();	// Print results of observer
void getParams();		// Parameter identification
void printdata();		// print the data in J matrix
void dataprocess();		// build the J matrix
void estimateSOC(int i);	// SOC Observer
void estimateSOH();	// SOH Observer
void celltopack();		// cell-to-pack algorithm
void lookup();			// SOC-VOC lookup table
void readdata();		// read data from RTAC
//void savedata(int time, FILE * file1, FILE * file2, FILE * file3);
						// save data into a file
void coulombcount();	// Coulomb counting
float determinant(float [][25], int);	// Calculate determinant
void cofactor(float [][25], int);		// Calculate cofactor
void transpose(float [][25], float [][25], int);	// Claculate transpose matrix
void ADFilterF();		// Abnormal data filter
int savetodb(int argc, char** argv);	// Save database
void intHandler(int);	// int handler
void  posttoapi(string, string, string, string, string, string );

// Define global variables
// --- Temp flag ---
int tmp0 = 0;
int tmp1 = 0;
int flag = 0;
modbus_t *mb;

// --- Global variables for main function ---
bool userinterupt = true;	// Catch user interuption
int Loop = 0;		// Counter to perform PI and Observer
long counter = 0;
int status = -1;	// -1: discharging, 1: charging
float volt = 0.0;	// terminal votlage
float volt_old = 0.0;	// load votlage (past)
float curr = 0.0;	// load current
float curr_old = 0.0;	// load current (past)

// --- Global variables for PI ---
float curr_win1[N_1];
float volt_win1[N_1];
float out_Y[win_size];	// array for parameter identification
float mat_J[win_size][coeffnum];	// array for parameter identification
float mat_JT[coeffnum][win_size];	// array for parameter identification
float mat_JTJ[N][N];
					// Temp matrix of parameter identificaiton
float mat_JTJI[coeffnum][coeffnum];
					// Inverse matrix
float mat_JTJIJT[coeffnum][win_size];
					// JTJIJT
int det_a;			// det A index (0: cannot inverse)
float R_init = 0.205;   // Initial R at new condition
float R0   = 0.18;	// R_0
float R1   = 0.08;	// R_1
float C1   = 0.25;	// C_1
float b1Qr = 0.0002;	// b_1 / Q_R
float para[coeffnum];	// wrap the identified parameters
int flag_checkVOC = 0;	// The flag for wrong VOC (0: correct esti., 1: wrong esti.)
int counter_checkVOC = 0;	// The counter for wrong VOC (0: correct esti., 1: wrong esti.)
float para_prev[coeffnum];	// the previous identified parameters
float coeff_tfd[coeffnum];
					// coefficient of the identified discrete-time transfer function 

// --- Global variables for OB ---
float Ad[tforder][tforder];	
					// A matrix in observer
float Bd[tforder];	// B matrix in observer
float Cd[tforder];	// C matrix in observer
float pole_c[tforder];	// polse assignments for the observer gain
float L_d[tforder];	// observer gain
float err = 0.0;	// observer error
float estVoc = 0.0;	// estimated V_OC  (x_1 in observer)
float estVrc1= 0.0;	// estimated V_RC1 (x_2 in observer)
float estVt  = 0.0;	// estimated V_T (y observer)
float estSOC = 0.0;	// estimated SOC (V_OC -> lookup table -> SOC)
float estSOC_nxt = 0.0;	// estimated SOC output
float estSOH_R = 99.47; // estimated SOH using internal resistance
float estSOH_C = 99.45; // estimated SOH using available capacity
float estSOH_Coul_tmp = 0.0;
float estSOH_Cap_low = 0.0;
float estSOH_Cap_high = 0.0;
float SOC_ref = 0.0;	// SOC reference
float G_C2P  = 0.0;	// cell-to-pack gain
float estSOC_max = 0.0;	// maximum SOC of the battery cell
float estSOC_pck  = 0;	// estimated SOC (V_OC -> lookup table -> SOC)
float pole_d[tforder];
float coeff1, coeff2, detA;

// --- Global variables for Database ---
struct timespec beg_clk,end_clk;
int complex_count = 0;
int getparams_count = 0;
double sendMessage[5];
char stringMessage[500];
char medString[50];
bool data_acq = false;
char mod_volt_info[15];
char mod_curr_info[15];
char mod_sysstus_info[15];
float sysstus;
int  sysstus_1;
static bool keepRunning = true;

// --- User defined parameters ---
float curr_threshold  = 0.5;	// for regional-awareness parameter id
float volt_cutoff_up  = 4.2;	// upper cut-off voltage
float time_stamp = 1;
int   freq_data = 10;	// frequency of the data receiving (Hz)
int   PI_update = 0;

// --- SOC-VOC profile in float format ---
static int table_SOC[96] = {\
	0,1,2,3,4,5,6,7,8,9,\
	10,11,12,13,14,15,16,17,18,19,\
	20,21,22,23,24,25,26,27,28,29,\
	30,31,32,33,34,35,36,37,38,39,\
	40,41,42,43,44,45,46,47,48,49,\
	50,51,52,53,54,55,56,57,58,59,\
	60,61,62,63,64,65,66,67,68,69,\
	70,71,72,73,74,75,76,77,78,79,\
	80,81,82,83,84,85,86,87,88,89,\
	90,91,92,93,94,95};
static float table_Voc[96] = {\
	593.2206507,616.6383356,619.4795239,620.7771047,621.9883602,623.3294428,624.3347345,625.4964995,627.127467,628.5737801,\
	630.0277104,631.6870844,633.4576514,635.4453603,637.3329539,638.630324,640.3068455,641.5540429,643.0740653,644.3976875,\
	645.3767055,646.2407503,646.9300695,647.6052456,648.3996853,648.8825901,649.4774726,650.1067513,650.3675721,651.3093147,\
	651.8759112,652.3881116,652.9776189,653.6660044,654.1937291,654.8477503,655.5454142,656.1430431,656.6319438,657.007081,\
	657.6932123,658.4803492,659.0073363,659.6490954,660.4153234,660.9573678,662.0656291,662.7823626,663.6296932,664.4212246,\
	665.5118517,666.701003,667.8844765,669.2605237,670.0835483,671.1299897,672.3321758,673.1576855,674.0951614,675.1407521,\
	676.1791059,677.4241253,678.2861719,679.580221,680.8824694,682.3163206,683.5129333,684.4957162,686.0061632,687.2172276,\
	688.5532277,690.4427527,691.8120875,693.4290535,694.9156347,696.5905718,698.3956507,699.780069,701.7789237,703.4818428,\
	705.0767879,706.5135511,708.085543,710.0256509,711.8766277,713.7536371,716.0659041,717.5273089,719.5037425,721.5823655,\
	723.9811916,725.3495741,727.0746845,729.3990723,731.9135693,734.1813936};


volatile sig_atomic_t done = 0;

void sig_term_handler(int signum) {
	done = 1;
}

void catch_sigterm() {
	static struct sigaction _sigact;

	memset(&_sigact, 0, sizeof(_sigact));
	_sigact.sa_handler = sig_term_handler;
	_sigact.sa_flags = SA_SIGINFO;

	sigaction(SIGTERM, &_sigact, NULL);
}

// Main function
int main(void){
	// Catch SIGTERM so we can exit gracefully!
	catch_sigterm();

	// --- Open files for data saving ---
	clock_gettime(CLOCK_MONOTONIC_RAW,&beg_clk);

	// --- Catch interuption ---
	while(!done){
		sleep(3);
		// --- Counter ---
		Loop = Loop + 1;
		if (Loop >= 1001){
			Loop = Loop - 1000;	// clear counter
		}

		// Co-Estimation algorithm
		// --- Read data from the array ---
		printf("Read data through Modbus and perform SOC/SOH estimation.\n");
//		printf("- Reading data through Modbus...\n");
		readdata();
//		ADFilterF();			// Abnormal data filter
		print_Modbus_result();	// print
		// --- Coulomb counting: SOC reference ---
//		coulombcount();			// Coulomb counting
		// --- Parameter Identification ---
		if (Loop % time_ID==0){
//			printf("-- Performing the parameter identificaiton. --\n");
			dataprocess();		// build the J matrix
			getParams();		// parameter identification
//			print_PI_result();	// print result
		}
		// --- Observer ---
		if (Loop % time_OB==0){
//			printf("-- Performing the SOC/SOH estimation. --\n");
			estimateSOC(Loop);	// observer
			lookup();			// SOC lock-up
//			celltopack();		// cell-to-pack mapping
			estimateSOH();		// SOH estimation
			print_OB_result();	// print result
		}
		// --- Save data into a database ---
		int argc;
		char** argv;
		savetodb(argc, argv);
/*		// --- open text files ---
		FILE * file1 = fopen("out_data.txt", "w");
		FILE * file2 = fopen("out_est_SOC.txt", "w");
		FILE * file3 = fopen("out_id_para.txt", "w");
		// --- Save data into files ---
		savedata(Loop,file1,file2,file3);
		// --- close text files ---
		fclose(file1);
		fclose(file2);
		fclose(file3);
*/
	} 
	return 0;
}
     
void readdata(){
	
	uint16_t volt_p[2];
	uint16_t curr_p[2];

	int read1, read2;

	mb = modbus_new_tcp("10.61.78.98",502);
	if(modbus_connect(mb)==-1){
		printf("Error connecting to Modbus!\n");
	}
	read1=modbus_read_registers(mb, 256, 2, volt_p);
        printf("Reading port 256: %d\n",read1);
        volt=modbus_get_float_cdab(volt_p);
	read2=modbus_read_registers(mb, 266, 2, curr_p);
        printf("Reading port 266: %d\n",read2);
        curr=modbus_get_float_cdab(curr_p);
	modbus_close(mb);
  	modbus_free(mb);
/*
	// init variables
	FILE *myfile;
	char modbuscmd[50];
	/// Get voltage data
	// use string copy and system command to get data.
	strcpy(modbuscmd, "modbus read 10.61.78.98 %MF256 1 > v.txt");
	system(modbuscmd);
	system(NULL);
	// read data
	myfile = fopen("v.txt","r");
	fscanf(myfile, "%s         %f",mod_volt_info,&volt);
	fclose(myfile);
	/// Get current data
	// use string copy and system command to get data.
	strcpy(modbuscmd, "modbus read 10.61.78.98 %MF266 1 > c.txt");
	system(modbuscmd);
	system(NULL);
	// read data
	myfile = fopen("c.txt","r");
	fscanf(myfile, "%s         %f",mod_curr_info,&curr);
	fclose(myfile);
	/// Get system status
	// use string copy and system command to get data.
	strcpy(modbuscmd, "modbus read 10.61.78.98 %MF229 1 > sysstus.txt");
	system(modbuscmd);
	system(NULL);
	// read data
	myfile = fopen("sysstus.txt","r");
	fscanf(myfile, "%s         %f",mod_sysstus_info,&sysstus);
	fclose(myfile);
	sysstus_1 = (int) round(sysstus);
*/
}

// Abnormal data filters
void ADFilterF(){
	// filter chronic abnormal data
	if (sysstus == 0.0){
		curr = 0;
	}
/*	if (fabs(volt - volt_old) < curr_old * R0){
		curr = 0;
	}
*/
	// filter acute abnormal data
	// find NaN
	if (curr != curr || volt != volt){
		curr = curr_old;
		volt = volt_old;
	}
	// update previous data samples
	curr_old = curr;
	volt_old = volt;
}

// cell-to-pack algorithm
void celltopack(){
	// update the battery cell max SOC when any battery cell in the bnattery pack is fully charged
	// Step 1: Find the battery cell which has the lowest SOC
	//   The cell with min capacity has lowest voltage when the battery pack is fully charged.
	//   (Assume that all the cells has similar capacity)
	//   If would be safe to keep searching for the battery cell with the lowest voltage.
	if (volt >= volt_cutoff_up){
		estSOC_max = estSOC;
	}
	// Step 2: Update the cell-to-pack gain
	// G_C2P = Q_pack / Q_cell,i
	//       = 100% / (SOC_max,cell,i - 0%)
	G_C2P = 1 / estSOC_max;
	// Step 3: Estimate the battery pack SOC
	estSOC_pck = estSOC * G_C2P;
}

// Coulomb counting
void coulombcount(){
	SOC_ref += curr * time_data / 5165.66;
}

// SOC-VOC lookup table
void lookup(void){
	int i = 0; // index of look-up table
	float slope = 0.0;
	// find the index
	while (estVoc > table_Voc[i] && i < 100){
	    i++;
	}
	// find the slope
	slope  = (table_SOC[i] - table_SOC[i-1])/(table_Voc[i] - table_Voc[i-1]);
	estSOC = table_SOC[i-1] + slope * (estVoc - table_Voc[i-1]);
	// saturation
	if(estVoc >= table_Voc[95]){
	    estSOC = table_SOC[95];
	}else if(estVoc <= table_Voc[0]){
	    estSOC = table_SOC[0];
	}
}

/* SOH Estimation */
void estimateSOH(){
	// Method 1: Estimate SOH using the internal resistance
	estSOH_R = 100- ((R0+R1-R_init) / R_init);
	// saturation
	if (estSOH_R > 100){
		estSOH_R = 100;
	}else if (estSOH_R < 0){
		estSOH_R = 0;
	}
	// Method 2: Estimate SOH using the capacity fo the battery
	if (curr >= 20){
		if (estSOC >=20){
			estSOH_Cap_low = estSOC;	// estimated SOC
			estSOH_Coul_tmp = estSOH_Coul_tmp + curr * time_stamp / 3600;	// Coulomb counting
		}else if(estSOC >= 80 && estSOC <= 100){
			estSOH_Cap_high = estSOC;	// estimated SOC
		}
		if (estSOH_Cap_low > 0 && estSOH_Cap_high > 0 && estSOH_Coul_tmp != 0){
			estSOH_C = fabs(estSOH_Coul_tmp / (estSOH_Cap_high - estSOH_Cap_low));
		}
	}else{	// reset
		estSOH_Coul_tmp = 0;
	}
}

// build the J matrix
void dataprocess(){
	// moves the data in the moving window
	for (int i = 0; i <= win_size-2; i++){
		out_Y[i] = out_Y[i+1];
		for (int j = 0; j <= coeffnum-1 ; j++)
			mat_J[i][j] = mat_J[i+1][j];
	}
	// assign the new data
	out_Y[win_size-1]    =  volt;
	mat_J[win_size-1][0] = -1 * out_Y[win_size-2];
	mat_J[win_size-1][1] = -1 * out_Y[win_size-3];
	mat_J[win_size-1][2] = curr;
	mat_J[win_size-1][3] = mat_J[win_size-2][2];
	mat_J[win_size-1][4] = mat_J[win_size-3][2];
}

// Parameter identification
void getParams(){
// parameter identification
// regional-awareness
if ((mat_J[10][3]-mat_J[9][3]) >= curr_threshold){
	float detA = 0.0;
	PI_update = 1;
	// --- theta(k)=(J(k)^T .* J(k))^-1 .* J(k) * y(k) ---
	// --- Step 1: Transpose the matrix ---
	for (int i = 0; i <= win_size-1; i++){
		for (int j = 0; j <= coeffnum-1 ; j++)
		mat_JT[j][i] = mat_J[i][j];
	}
	// --- Step 2: mat_JTJ = J^T .* J ---
	for (int i = 0; i <= coeffnum-1 ; i++){
		for (int k = 0; k <= coeffnum-1 ; k++){
		for (int j = 0; j <= win_size-1; j++){
			mat_JTJ[i][k] += mat_JT[i][j] * mat_J[j][k];
		}}
	}
	// --- Step 3: mat_JTJI = (J^T .* J)^-1 ---
	detA = determinant(mat_JTJ, coeffnum);
	if (detA == 0)
		printf("\nInverse of Entered Matrix is not possible\n");
	else
	    cofactor(mat_JTJ, coeffnum);
	// --- Step 4: mat_JTJIJ = (J^T.*J)^-1*J ---
	for (int i = 0; i <= coeffnum-1 ; i++){
		for (int k = 0; k <= win_size-1 ; k++){
		for (int j = 0; j <= coeffnum-1; j++){
		    mat_JTJIJT[i][k] += mat_JTJI[i][j] * mat_JT[j][k];
		}}
	}
	// --- Step 5: para1 = mat_JTJIJY = (J^T.*J)^-1*J*Y ---
	for (int i = 0; i <= coeffnum-1; i++){
		for (int j = 0; j <= win_size-1 ; j++)
		coeff_tfd[i] = mat_JTJIJT[i][j] * out_Y[j];
	}
	// --- Step 6: Pull the coefficients out ---
	float a1 = coeff_tfd[0];
	float a2 = coeff_tfd[1];
	float c0 = coeff_tfd[2];
	float c1 = coeff_tfd[3];
	float c2 = coeff_tfd[4];
	// --- Step 7: convert from discrete tf to continuous tf ---
	float n0 = (c0-c1+c2)*time_ID*time_ID/4.0;
	float n1 = (c0-c2)*time_ID;
	float n2 = c0+c1+c2;
	float d0 = (1-a1+a2)*time_ID*time_ID/4.0;
	float d1 = (1-a2)*time_ID;
	float d2 = (1+a1+a2);
	// --- Step 8: Normalizing the first coefficient in the denominator to 1 ---
	d1 = d1/d0;
	d2 = d2/d0;
	n0 = n0/d0;
	n1 = n1/d0;
	n2 = n2/d0;
	//  Step 9 - Convert to Pole-Zero format //
	float gain,detnum,detden,zeros[tforder],poles[tforder];
	gain = n0;
	detnum = pow((n1/n0),2) - 4*n2/n0;
	detden = pow(d1,2) - 4*d2;
	if(detnum >= 0){
		zeros[0] = (-n1/n0 + sqrt(detnum))/2;
		zeros[1] = (-n1/n0 - sqrt(detnum))/2;
	}
	if(detden >=0){
		poles[0] = (-d1 + sqrt(detden))/2;
		poles[1] = (-d1 - sqrt(detden))/2;
	}
	float num[tforder+1];
	num[0] = n0;
	num[1] = n1;
	num[2] = n2;
	float den[tforder+1];
	den[0] = 1;
	den[1] = d1;
	den[2] = d2;
	float num12[tforder+1];
	for(int i = 0; i <= tforder; i++){
	    num12[i] = num[i] - gain*den[i];
	}
	// --- Step 10: Pull the parameters out ---
	R0 = gain;
	float RC = 1.0/poles[0];
	b1Qr = ((num12[1])*(poles[1]) + (num12[2]))/((poles[1]) - (poles[0]));
	C1 = (poles[0]-poles[1])/((num12[1])*(poles[0] ) + (num12[2]));
	R1 = RC/C1;
	// --- Step 11: data processing ---
	R0 = fabs(R0);
	R1 = fabs(R1);
	C1 = fabs(C1);
	b1Qr = fabs(b1Qr);
	para[0] = R0;
	para[1] = R1;
	para[2] = C1;
	para[3] = b1Qr;
}else if(PI_update == 0){
//	Set initial values
//	printf("*** PI is NOT performed since lack of information. ***\n");
	para[0] = R0;
	para[1] = R1;
	para[2] = C1;
	para[3] = b1Qr;
}

// Use the previous para if VOC is estimated very wrong
if(flag_checkVOC == 1){	// Para is ID wrong, use the previous parameter
		para[0] = para_prev[0];
		para[1] = para_prev[1];
		para[2] = para_prev[2];
		para[3] = para_prev[3];
}else{					// Store the correctly ID parameter to para_prev after 3 iterations
	counter_checkVOC = counter_checkVOC + 1;
	if(counter_checkVOC >= 3){
		para_prev[0] = para[0];
		para_prev[1] = para[1];
		para_prev[2] = para[2];
		para_prev[3] = para[3];
	}else{
		counter_checkVOC = 0;	// reset register
	}
}

}

// Observer
void estimateSOC(int i1){
	// Output the SOC_cell (x^(k))
	lookup();
	float Ad_alpha[tforder][tforder];
	float CA[tforder];
	float N_matI[tforder][tforder];
	float N_mat[tforder][tforder];
	float AA[tforder][tforder];
	float a1,a2,alpha1,alpha2;
	float exp_operands[4], x;
	
	// update the SOC of next time stamp
	// update the state-space matrix
	Ad[0][0] = 1;
	Ad[0][1] = 0;	Ad[1][0] = 0;
	//	Ad[1][1] = exp(-time_OB/R1/C1);	// e^(-Ts/RC)
	Bd[0] = time_OB * b1Qr + time_OB / C1;
	Bd[1] = time_OB * b1Qr + (1-Ad[1][1]) * R1;
	Cd[0] = 1;
	Cd[1] = 1;

	// Pole placement using Ackermanns formula:
	// 	L_d =  phi(Ad) * N_matI * [0 0....1]^T
	// Step 1: Assign the poles: pole_c
	if(PI_update == 0){
		L_d[0] = 0.9;
		L_d[1] = 0.3;
	}else{
		pole_c[0] = -0.05;
		pole_c[1] = -3*Ad[1][1]/R1/C1;
	// Step 2: Convert to Discrete time
	//	pole_d[0] = exp(pole_c[0]*time_OB);
	//	pole_d[1] = exp(pole_c[1]*time_OB);
	exp_operands[0] = -time_OB / R1 / C1;	// Ad[1][1]
	exp_operands[1] = pole_c[0] * time_OB;	// pole_d[0]
	exp_operands[2] = pole_c[1] * time_OB;	// pole_d[1]
	exp_operands[3] = 0.0f;
	for (int i = 0; i < 4; i++)
	{
		x = exp_operands[i];
		x = 1.0f + x / 256.0f;
		x *= x; x *= x; x *= x; x *= x;
		x *= x; x *= x; x *= x; x *= x;
		exp_operands[i] = x;
	}
	Ad[1][1] = exp_operands[0];	// Ad[1][1]
		// Step 3: Calculate coefficients for equation phi(Ad)
	alpha1 = -(pole_d[0] + pole_d[1]);	
	alpha2 = (pole_d[0])*(pole_d[1]);
	// Step 4: Calculate Cd*Ad = CA
	CA[0] = Cd[0]*Ad[0][0] + Cd[1]*Ad[1][0];
	CA[1] = Cd[0]*Ad[1][0] + Cd[1]*Ad[1][1];
	// Step 5: N_mat = [Cd Cd*Ad ...]^T
	N_mat[0][0] = Cd[0];
	N_mat[0][1] = Cd[1];
	N_mat[1][0] = CA[0];
	N_mat[1][1] = CA[1];  
	// Step 6: N_matI = N_mat^-1
	float den;
	float detN_mat;
	detN_mat = N_mat[0][0] * N_mat[1][1] - N_mat[0][1] * N_mat[1][0];
	if(detN_mat == 0)
		printf("Inverse not Possible in observer computation");
	else{
		den = 1/detN_mat;
		N_matI[0][0] =  N_mat[1][1] * den;
		N_matI[0][1] = -N_mat[0][1] * den;
		N_matI[1][0] = -N_mat[1][0] * den;
		N_matI[1][1] =  N_mat[0][0] * den;
	}
	// Step 7: Calculate Ad^2 =  A * A
	for (int i = 0; i < tforder; i++){
		for (int j = 0; j < tforder; j++){
		AA[i][j] = 0;
		for (int k = 0; k < tforder; k++)
			AA[i][j] += Ad[i][k] * Ad[k][j];
		}
	}
	// Step 8: alpha1 * Ad = Ad_alpha
	for (int i = 0; i < tforder; i++){
		for (int j = 0; j < tforder; j++){
		Ad_alpha[i][j] = alpha1 * Ad[i][j];
		}
	}
	// Step 9: AA = Ad^2 + alpha1 * Ad + alpha2 * I
	for (int i = 0; i < tforder; i++){
		for (int j = 0; j < tforder; j++){
		if(i == j)
			AA[i][j] = AA[i][j] + Ad_alpha[i][j] + alpha2;
		else
			AA[i][j] = AA[i][j] + Ad_alpha[i][j] + 0;			
		}
	}	
	// Step 10: L1 = phi(A) * [C CA]^-1  = AA * N_matI
	float L1[tforder][tforder];
	for (int i = 0; i < tforder; i++){
		for (int j = 0; j < tforder; j++){
		L1[i][j] = 0;
		for (int k = 0; k < tforder; k++)
		L1[i][j] += AA[i][k]*N_matI[k][j];
		}
	}
	// Step 11: observer gain L_d =  phi(Ad)*N_matI*[0 0 ... 1]^T
	L_d[0] = L1[0][1];
	L_d[1] = L1[1][1];
	}
	// --- y^(k)   = C * x^(k) + D * u(k) ---
	estVt = Cd[0]*estVoc + Cd[1]*estVrc1 + R0*curr;
	err = volt - estVt;
	// --- xhat = [SOC, V_RC1, V_RC2] ---
	// --- Update x^(k+1) ---
	// --- x^(k+1) = A * x^(k) + B * u(k) + L * (y - V_T) ---
	estVoc  = Ad[0][0] * estVoc  + Bd[0] * curr;
	estVrc1 = Ad[1][1] * estVrc1 + Bd[1] * curr;
	
	// Adapting OB gain
//	estVoc  += L_d[0] * err * fabs(curr / ratedcurr);
//	estVrc1 += L_d[1] * err * fabs(curr / ratedcurr);
	estVoc  += L_d[0] * err;
	estVrc1 += L_d[1] * err;

	// Check whether the estimated VOC is far from the terminal voltage (greater than 100 Volt diff.)
	if(fabs(estVoc - volt) >= 100){
		flag_checkVOC = 1;
		estVoc = volt;		// overwrite the estimated voltage by the terminal voltage
	}else{
		flag_checkVOC = 0;
	}
}

/*For calculating Determinant of the Matrix */
float determinant(float a[25][25], int k){
	float s = 1, det = 0, b[25][25];
	int i, j, m, n, c;
	if (k == 1){
		return (a[0][0]);
	}else{
		det = 0;
		for (c = 0; c < k; c++){
		m = 0;
		n = 0;
		for (i = 0;i < k; i++){
			for (j = 0 ;j < k; j++){
				b[i][j] = 0;
				if (i != 0 && j != c){
					b[m][n] = a[i][j];
				if (n < (k - 2))
					n++;
				else{
					n = 0;
					m++;
		}}}}det = det + s * (a[0][c] * determinant(b, k - 1));
		s = -1 * s;
	}}return (det);
}
void cofactor(float num[25][25], int f){
	float b[25][25], fac[25][25];
	int p, q, m, n, i, j;
	for (q = 0;q < f; q++){
		for (p = 0;p < f; p++){
		m = 0;
		n = 0;
		for (i = 0;i < f; i++){
			for (j = 0;j < f; j++){
			if (i != q && j != p){
				b[m][n] = num[i][j];
				if (n < (f - 2))
				n++;
				else{
				n = 0;
				m++;
			}}}
		} fac[q][p] = pow(-1, q + p) * determinant(b, f - 1);
	}} transpose(num, fac, f);
}

/*Finding transpose of matrix*/
void transpose(float num[25][25], float fac[25][25], int r){
	int i, j;
	float b[25][25], d;
	for (i = 0;i < r; i++){
		for (j = 0;j < r; j++){
			b[i][j] = fac[j][i];
	}} d = determinant(num, r);
	for (i = 0;i < r; i++){
		for (j = 0;j < r; j++){
			mat_JTJI[i][j] = b[i][j] / d;
	}}
}

// print the data in J matrix
void printdata(){
	// check the Y matrix
	printf("the data in the Y matrix is:\n");
	for (int i = 0; i <= win_size-1; i++){
		printf("\t%.2f", out_Y[i]);
	}	printf("\n\n");
	// check the J matrix
	printf("the data in the J matrix is:\n");
	for (int i = 0; i <= win_size-1; i++){
		for (int j = 0; j <= coeffnum-1 ; j++){
			printf("\t%.2f", mat_J[i][j]);
		}	printf("\n");
	}	printf("\n");
	// check the JT matrix
	printf("the data in the JT matrix is:\n");
	for (int i = 0; i <= coeffnum-1; i++){
		for (int j = 0; j <= win_size-1 ; j++){
			printf("\t%.2f", mat_JT[i][j]);
		}	printf("\n");
	}	printf("\n");
	// check the JTJ matrix
	printf("the JTJ matrix is:\n");
	for (int i = 0; i <= coeffnum-1; i++){
		for (int j = 0; j <= coeffnum-1 ; j++)
			printf("\t%.2f", mat_JTJ[i][j]);
		printf("\n");
	}	printf("\n");
	// check the JTJI matrix
	printf("The JTJI matrix is:\n");
	for (int i = 0; i <= coeffnum-1; i++){
		for (int j = 0; j <= coeffnum-1 ; j++)
			printf("\t%.2f", mat_JTJI[i][j]);
		printf("\n");
	}	printf("\n");
	// check the JTJI matrix
	printf("The JTJIJT matrix is:\n");
	for (int i = 0; i <= coeffnum-1; i++){
		for (int j = 0; j <= win_size-1 ; j++)
			printf("\t%.2f", mat_JTJIJT[i][j]);
		printf("\n");
	}	printf("\n");
	// check the identifed coefficietns of tf_d
	printf("the coefficient of the discrete time tf:\n");
	for (int i = 0; i <= coeffnum-1; i++){
		printf("%.2f\t", coeff_tfd[i]);
	}	printf("\n");
}

// Print results of parameter identification
void print_PI_result(){
	// print the identified parameters
	printf("the identified para using ARX:\n");
	printf("R_0\t\tR_1\t\tC_1\t\tRC_1\t\tb1/QR\n");
	for (int i = 0; i<= coeffnum-1; i++){
		printf("%.10f\t", para[i]);
		}printf("\n");
}

// Print results of Modbus
void print_Modbus_result(){
	// print the result
//	printf("Data from Modbus are:\n");
	printf("- Current at %s is %.2f amp.\n", mod_curr_info,curr);
	printf("- Voltage at %s is %.2f volt.\n",mod_volt_info,volt);
//	printf("- System status is %d.\n",mod_sysstus_info,sysstus_1);
}

// Print results of observer
void print_OB_result(){
	// Print the estimated battery cell VOC
	printf("= Estd batt VOC is %.2f Volt.\n", estVoc);
	// Print the estimated battery cell SOC
	printf("= Estd batt SOC is %.2f percent.\n", estSOC);
	// Print the estimated battery SOH
	printf("= Estd batt SOH is %.2f percent (using Resistance). \n", estSOH_R);
	printf("= Estd batt SOH is %.2f percent (using Capacity).", estSOH_C);
	printf("\n"); printf("\n");
}

// Save data into a file
void savedata(int t, FILE * file1, FILE * file2, FILE * file3){
	float error = (estSOC - SOC_ref);
	float estSOC1 = estSOC;
	float SOC_ref1 = SOC_ref;
	clock_gettime(CLOCK_MONOTONIC_RAW,&end_clk);
	float interval = (end_clk.tv_sec -beg_clk.tv_sec)*1000 + (end_clk.tv_nsec - beg_clk.tv_nsec)/1000000;
	time_stamp = interval/1000;
	char array1[200] ="time_stamp		      estSoC      	SoC_ref      	SoC_err";
	char array2[200] ="time_stamp	      		R0	  	R1	      	 C1	 	R1C1     	 b1Qr";
	fprintf(file1, "%f\t%f\t%f\t%f\n", interval, volt, curr, 0.0);	
	if(t == 0){
		//fprintf(file2, "%s\n\n",array1);
		fprintf(file3, "%s\n\n",array2);	
	}
	else{
		fprintf(file2, "%6.3f\t\t%f\t\t%f\t\t%f\t\t%f\n", time_stamp, estSOC1, estSOH_R, estSOH_C,0.0);
		fprintf(file3, "%6.3f\t\t%f\t\t%f\t\t%f\t\t%f\t\t%f\n",time_stamp, R0, R1, C1, R1*C1, b1Qr);
	}
}
/*
// Call back function
static int callback(void* data, int argc, char** argv, char** azColName){
	int i;
	fprintf(stderr, "%s: ", (const char*)data);
	for (i = 0; i < argc; i++){
		printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
	}
	printf("\n");
	return 0;
}
*/

void  posttoapi(string current, string estSoc, string estSohC, string estSohR, string voltage, string cdate){
	CURL *curl;
	CURLcode res;
	
	curl_global_init(CURL_GLOBAL_ALL);
        string cdata="";
	cdata = "{ \"bmsNodeBmsNodeName\": \"Butlet Farms Node 1\", \"bmsNodeId\": 1, ";
	printf("debug: %s", current.c_str());
	cdata += " \"current\": ";
	cdata += "\"" + current + "\"";
	cdata += ", \"estSoc\": ";
	cdata += "\"" + estSoc + "\"";
	cdata += ", \"estSohR\": ";
	cdata += "\"" + estSohR + "\"";
	cdata += ", \"estSohC\": ";
	cdata += "\"" + estSohC + "\"";
	cdata += ", \"eventTimestamp\": ";
	cdata += "\"" + cdate + "\"";
	cdata += ", \"voltage\": ";
	cdata += "\"" + voltage + "\"";
	cdata += "}";

	char token[] = "Authorization: Bearer eyJhbGciOiJIUzUxMiJ9.eyJzdWIiOiJhZG1pbiIsImF1dGgiOiJST0xFX0FETUlOLFJPTEVfVVNFUiIsImV4cCI6MTYwNDU0NjA3NX0.M6XMlC2BBMQJDrh96VtWktzwjrdKe68mcNUle3qTFxA19hksRTvmq9x6LNOK_kH4jDRUDVpQ1EaKUPVFJXWR9Q";
	printf("DATA %s\n", cdata.c_str());	

		


	/* get a curl handle */ 
	curl = curl_easy_init();
	    if(curl) {
		struct curl_slist *chunk = NULL;
		chunk = curl_slist_append(chunk, token);
		chunk = curl_slist_append(chunk, "Content-Type: application/json");
		curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
	        /* First set the URL that is about to receive our POST. This URL can
		 *        just as well be a https:// URL if that is what should receive the
		 *               data. */ 
	        curl_easy_setopt(curl, CURLOPT_URL, "https://api.smart-battery-gauge.xyz/api/bms-node-events");
		/* Now specify the POST data */ 
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, cdata.c_str());
		     
	       /* Perform the request, res will get the return code */ 
	        res = curl_easy_perform(curl);
	    	/* Check for errors */ 
	 	if(res != CURLE_OK)
	          fprintf(stderr, "curl_easy_perform() failed: %s\n",
	        curl_easy_strerror(res));
				     
	       /* always cleanup */ 
	        curl_easy_cleanup(curl);
	  }
	      curl_global_cleanup();
	       // return 0;
}



// Save data as a database
int savetodb(int argc, char **argv){
	sqlite3* DB;
	char* messaggeError;
	int exit = sqlite3_open("testDB.db", &DB);
	string v1,v2,v3,v4,v5,v6;

	if (exit){
		std::cerr << "Error occured while opening database. " << sqlite3_errmsg(DB) << std::endl;
		return (-1);
	}else{
//		std::cout << "Saving data..." << std::endl;
	}
	v1 = to_string(volt);
	v2 = to_string(curr);
	v3 = to_string(estSOC);
	v4 = to_string(estSOH_R);
	v5 = to_string(estSOH_C);
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];
	time (&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer,sizeof(buffer), "%H:%M:%S",timeinfo);
	std::string currenttime(buffer);
	strftime(buffer,sizeof(buffer), "%Y-%m-%d",timeinfo);
	std::string currentdate(buffer);


	string i1 = "INSERT INTO BMSDB2 (DATE,TIMESTAMP, VOLTAGE,CURRENT,estSOC,estSOH_R,estSOH_C) VALUES (";
	string i2 = ")";
	string i3 = ",";
	string i4 = "'";
	string i5 = i1+i4+currentdate+i4+i3+i4+currenttime+i4+i3+v1+i3+v2+i3+v3+i3+v4+i3+v5+i2;

	exit = sqlite3_exec(DB, i5.c_str(), NULL, 0, &messaggeError);

	if (exit != SQLITE_OK){ 
		std::cerr << "\nError occured while saving database." << std::endl; 
		sqlite3_free(messaggeError); 
	}else{
//		cout << i5;
		string i6 = "At "+currenttime+": ";
		cout << i6;
		std::cout << "Database saved successfully." << std::endl; 
		printf("v1: %s; v2: %s, currentdate: %s", v1.c_str(), v2.c_str(), currentdate.c_str());
		string ts = currentdate + "T"+currenttime+"-04:00";
		posttoapi( v2, v3, v5, v4, v1, ts);
	}
	sqlite3_close(DB); 
	printf("--------------------------\n\n");
	return (0); 
}



