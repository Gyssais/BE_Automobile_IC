//Date : 15 février 2016
//Auteur : A. Boyer (INSA de Toulouse)
//Projet : Demo_Cluster_v1 : programme de démo mettant en valeur les fonctionnalités du Cluster Visteon et du microcontrôleur MPC5645S.
//Séquençage d'une machine à état par les timers.
//Timer 0 : base de temps pour les moteurs pas à pas des afficheurs à aiguilles
//Timer 1 : séquençage machine à état de la démo
//Timer 2 : clignotant des warnings (période 1 s)

//Séquencement :
//Init = initialisation de la position des aiguilles, allumage des voyants et backlight, DCU en mode test
//Phase 1 = les aiguilles vitesse et tour minute montent rapidement vers une position 200 km/h et 4000 tr/min. On reste dans cet état 
//          pendant 3 s avant de passer à l'état 2. Les clignotants clignotent toutes les secondes.
//Phase 2 = les aiguilles vitesse et tour minute descendent lentement vers une position 90 km/h et 2000 tr/min. On reste dans cet état 
//          pendant 6 s avant de passer à l'état 3. Les clignotants clignotent toutes les secondes.
//Phase 3 = les aiguilles vitesse et tour minute montent rapidement vers une position 200 km/h et 4000 tr/min.  On reste dans cet état 
//          pendant 6 s avant de passer à l'état 2. Les clignotants clignotent toutes les secondes.
//
//          Durant les phases 1, 2 et 3, l'écran LCD est en mode normal. Il est découpé en 3 parties, entourées par un liseret noir (Layers
//          4, 5 et 6) placés en haut à gauche, en haut à droite et en bas.
//			Dans le cadre en haut à gauche, on affiche l'heure avec le curseur hardware clignotant toutes les 1 sec.
//			Dans le cadre en haut à droite, on affiche la langue en cours avec un drapeau français (construction du drapeau).
//			Dans le cadre en bas, on affiche le logo de l'INSA, au format 4 bpp (on charge une image en mémoire)
//			L'ensemble des éléments graphiques sont chargés en mémoire Graphic SRAM.
//			On change l'heure indiquée sur le layer1 toutes les secondes (cadencé par Timer2).



#include "MPC5645S.h"
#include "IntcInterrupts.h"
#include "port.h"
#include "LCD_Graph_Manager.h"



//Variables et constantes pour le contrôle des jauges à aiguilles

typedef enum
{
	Init = 0,
    Phase1,
    Phase2,
    Phase3,
    Phase4,
    Phase5
}
State;

#define SENS_HORAIRE 0
#define SENS_ANTIHORAIRE 1
#define DONTMOVE 2

#define	MOTEUR0  0
#define	MOTEUR1  1
#define	MOTEUR2  2
#define	MOTEUR3  3

#define TIMER0	0
#define TIMER1	1
#define TIMER2	2
#define TIMER3	3

struct Moteur {
 unsigned int voie0;
 unsigned int voie1;
 unsigned int voie2;
 unsigned int voie3;
 unsigned int state_MOTOR_PWM;
 unsigned int sens_rotation;
 unsigned int position;
};


//MOTEUR0 : Indicateur de vitesse rapport (5/3° par full step)
//MOTEUR1 : Compte tour rapport (5/3° par full step)
//MOTEUR2 : Jauge de niveau de carburant (1.5° par full step)
//MOTEUR3 : Indicateur de température (1.5° par full step)

int init_finished = 0;
int indice = 0;
int etat_actuel = Init;

struct Moteur Moteur0;
struct Moteur Moteur1;
struct Moteur Moteur2;
struct Moteur Moteur3;

struct Moteur *pointeur_moteur0 = &Moteur0;
struct Moteur *pointeur_moteur1 = &Moteur1;
struct Moteur *pointeur_moteur2 = &Moteur2;
struct Moteur *pointeur_moteur3 = &Moteur3;

//état des warnings
int State_Warning = 0;

//chiffres indiquées sur le layer1 pour indiquer l'heure (format Min1Min2 : Sec1Sec2)
uint8_t Min1 = 0;
uint8_t Min2 = 0;
uint8_t Sec1 = 0;
uint8_t Sec2 = 0;

/*========================================================================*/
/*                          PROTOTYPES                                    */
/*========================================================================*/
void DISABLE_WATCHDOG();
void MC_MODE_INIT_PLL();
void enableIrq();
int Config_port_input(unsigned int port);
int Config_port_output(unsigned int port);
int Config_port_output_logical_inverted(unsigned int port);
int Set_GPIO(unsigned int port);
int Reset_GPIO(unsigned int port);
int Invert_GPIO(unsigned int port);
int Config_port_AF(unsigned int port, unsigned int AF_num);
uint8_t Read_GPIO(unsigned int port);
void step_to(unsigned int position, struct Moteur* moteur);
void init_struct_moteur(unsigned int num_motor, struct Moteur* moteur);
void motor_init(struct Moteur* moteur);
void init_position(struct Moteur* moteur);
void Init_PIT(void);
void Set_Counter_Value_PIT(uint8_t timer, uint32_t TimeOut);
void Start_PIT(uint8_t timer);
void Disable_PIT(uint8_t timer);
void Autorize_IT_PIT(uint8_t timer);
void Clear_Flag_PIT(uint8_t timer);
void Set_LEDS_Cluster(void);
void InitLayers(void);
void CONFIG_DCU_IO(void);
void CONFIG_DCU(void);
void activ_DCU_Test(void);
void activ_DCU_Normal(void);
void InitCursor();
void PIT_CH0_ISR(void);
void PIT_CH1_ISR(void);
void PIT_CH2_ISR(void);


//désactivation du watchdog du microcontrôleur
void DISABLE_WATCHDOG()
{
	SWT.SR.R = 0x0000c520; /* Write keys to clear soft lock bit */ 
	SWT.SR.R = 0x0000d928; 
	SWT.CR.R = 0x8000010A; /* disable WEN */
}

//on configure l'entrée en mode RUN0, utilisation de la PLL0 (division par 2 pour générer l'horloge système, pas de division de l'horloge système vers les périph
//on utilise PLL0 comme auxiliary clock 0 = clock source du DCU. On ne divise pas cette clock auxiliaire.
void MC_MODE_INIT_PLL()
{
	ME.MER.R = 0x0000001D;          /* Enable DRUN, RUN0, SAFE, RESET modes */
	
	  //config de la PLL
	  //NDIV = 50, IDF = '0000'--> 1, 0DF = '00' --> 2, clk_in = 8 MHz
	  //PLL freq = clk_in*NDIV/(IDF*ODF) = 200 MHz, Fvco = clk_in*NDIV/IDF = 400 MHz (la fréquence du VCO doit rester entre 256 et 512 MHz)
	  CGM.FMPLL[0].CR.B.IDF = 0;
	  CGM.FMPLL[0].CR.B.ODF = 0;
	  CGM.FMPLL[0].CR.B.NDIV = 50;
	  CGM.FMPLL[0].CR.B.EN_PLL_SW = 1; //activation PLL
	
	CGM.OCDS_SC.B.SELCTL=2; //sélection de la PLL principale divisée par 2 comme horloge système
	
	/* Enbable all peripheral clocks */
	//registres SC_DC (p 218) : On active les diviseurs de sysclk pour les 4 sets de periph mais division  = 1. 
	//voir p 2013 - table 8.1 pour les 4 sets de périph.
	CGM.SC_DC[0].R = 0x80808080;
	CGM.SC_DC[1].R = 0x80808080;
	CGM.SC_DC[2].R = 0x80808080;
	CGM.SC_DC[3].R = 0x80808080;
	
	//Auxiliary clock 0 --> pour le DCU3.
	CGM.AC0_SC.B.SELCTL = 0x03; //PLL0 comme source d'horloge.
	CGM.AC0_DC.B.DE0 = 1; //activation du diviseur
	CGM.AC0_DC.B.DIV0 = 0; //Division par 1;
	
	//remarque : pour savoir si le DCU3 est actif, regardez le bit S_DCU3 du registre ME.ME_PS1.
	
	
	ME.RUN[0].R = 0x001F0064;     /* enable fxosc and FMPLL0, FMPLL0 est l'horloge système */ 
	/* Peri. Cfg. 1 settings: only run in RUN0 mode
	  Only RUNPC[0] mode configuration is defined. Only this configuration will be used.*/ 
	ME.RUNPC[0].R = 0x00000010;  //les périph fonctionnent uniquement en mode RUN0
	
    //Voir Table 29.2 p 1067 datasheet MPC5645S pour obtenir les numéros associés à chaque périphérique (à priori, les mêmes que MPC5604B).
    ME.PCTL[68].R = 0x00;  //SIUL use the configuration of RunPC[0]  
    ME.PCTL[92].R = 0x00;  //PIT0 use the configuration of RunPC[0]
    ME.PCTL[55].R = 0x00;  //DCU3 use the configuration of RunPC[0]
    ME.PCTL[63].R = 0x00;  //TCON use the configuration of RunPC[0]
	
	ME.MCTL.R = 0x40005AF0;         /* Enter RUN0 Mode & Key */
	ME.MCTL.R = 0x4000A50F;         /* Enter RUN0 Mode & Inverted Key */  
	//la transition de mode exige ces 2 instructions avec la clé la 1e fois, puis la clé inversée. 									
	while (ME.GS.B.S_MTRANS) {}      
	while(ME.GS.B.S_CURRENTMODE != 4) {}  
} 


int Config_port_input(unsigned int port) 
{
	int error=-1;
	SIU.PCR[port].B.WPE = 0;
	SIU.PCR[port].B.WPS = 0;
	SIU.PCR[port].B.ODE= 0;
	SIU.PCR[port].B.PA = 0;
	SIU.PCR[port].B.OBE = 0;
	SIU.PCR[port].B.IBE = 1;
	error = 0;
	return error;
}


//Configuration of PORT as an output GPIO (no alternate function)
int Config_port_output(unsigned int port) 
{
    int error =-1;
    SIU.PCR[port].B.WPE = 1;
	SIU.PCR[port].B.WPS = 0;
	SIU.PCR[port].B.ODE= 0;
    SIU.PCR[port].B.SRC = 1;  //1.8 V low-power DDR full speed
	SIU.PCR[port].B.PA = 0;
	SIU.PCR[port].B.OBE = 1;
	SIU.PCR[port].B.IBE = 0;
	error = 0;
	return error;
}

//Configuration of PORT as an output GPIO, with inverted logic (no alternate function)
int Config_port_output_logical_inverted(unsigned int port)
{
	int error=-1;
	SIU.PCR[port].B.WPE = 1;
	SIU.PCR[port].B.WPS = 0;
	SIU.PCR[port].B.ODE= 1; 
	SIU.PCR[port].B.SRC = 1;  //1.8 V low-power DDR full speed
	SIU.PCR[port].B.PA = 0;
	SIU.PCR[port].B.OBE = 0;
	SIU.PCR[port].B.IBE = 0;
	error = 0;
	return error;
}

//Configuration of PORT in alternate function number AF_num
int Config_port_AF(unsigned int port, unsigned int AF_num) 
{
    int error =-1;
	if (1 <= AF_num && 3 >= AF_num)
	{
		SIU.PCR[port].B.PA = AF_num;
		error = 0;
	}
	else
	{
		error = 1;
	}
	return error;
}

// Set an output PORT
int Set_GPIO(unsigned int port)
{
	int error=-1;
	SIU.GPDO[port].B.PDO = 1;
	error = 0;
	return error;
}

// Reset an output PORT
int Reset_GPIO(unsigned int port)
{
	int error=-1;
	SIU.GPDO[port].B.PDO = 0;
	error = 0;
	return error;
}

// Invert the state of an output PORT
int Invert_GPIO(unsigned int port)
{
	int error=-1;
	SIU.GPDO[port].B.PDO = ~SIU.GPDO[port].B.PDO;
	error = 0;
	return error;
}

// Read the content of an input PORT
uint8_t Read_GPIO(unsigned int port)
{
	return SIU.GPDI[port].B.PDI;	
}


void init_struct_moteur(unsigned int num_motor, struct Moteur* moteur)
{
	//step motor output pads on PORT D
	(*moteur).voie0 = PD0+num_motor*4;
	(*moteur).voie1 = PD1+num_motor*4;
	(*moteur).voie2 = PD2+num_motor*4;
	(*moteur).voie3 = PD3+num_motor*4;
	(*moteur).state_MOTOR_PWM = 0;
	(*moteur).position = 0;
}

void motor_init(struct Moteur* moteur)
{
	Config_port_output((*moteur).voie0);
	Config_port_output((*moteur).voie1);
	Config_port_output((*moteur).voie2);
	Config_port_output((*moteur).voie3);
}


void init_position(struct Moteur* moteur)
{
	switch((*moteur).state_MOTOR_PWM)
	{
		case 0:
			Set_GPIO((*moteur).voie2);
			Reset_GPIO((*moteur).voie3);
			Set_GPIO((*moteur).voie0);
			Reset_GPIO((*moteur).voie1);
			(*moteur).state_MOTOR_PWM++;
			break;
					
		case 1:
			Reset_GPIO((*moteur).voie2);
			Set_GPIO((*moteur).voie3);
			Set_GPIO((*moteur).voie0);
			Reset_GPIO((*moteur).voie1);
			(*moteur).state_MOTOR_PWM++;
			break;
					
		case 2:
			Reset_GPIO((*moteur).voie2);
			Set_GPIO((*moteur).voie3);
			Reset_GPIO((*moteur).voie0);
			Set_GPIO((*moteur).voie1);
			(*moteur).state_MOTOR_PWM++;
			break;
					
		case 3:
			Set_GPIO((*moteur).voie2);
			Reset_GPIO((*moteur).voie3);
			Reset_GPIO((*moteur).voie0);
			Set_GPIO((*moteur).voie1);
			(*moteur).state_MOTOR_PWM++;
			break;
					
		default: 
			Reset_GPIO((*moteur).voie0);
			Reset_GPIO((*moteur).voie1);
			Reset_GPIO((*moteur).voie2);
			Reset_GPIO((*moteur).voie3);
			(*moteur).state_MOTOR_PWM =0;
	}

	if((*moteur).state_MOTOR_PWM>= 4)
	{
		(*moteur).state_MOTOR_PWM =0;
	}	
}

void step_to(unsigned int position, struct Moteur* moteur)
{
    if(moteur->position > position)
    {
    	(*moteur).sens_rotation = SENS_ANTIHORAIRE;
    }
    else if(moteur->position < position)
    {	
    	(*moteur).sens_rotation = SENS_HORAIRE;
    }
    else
    {
    	(*moteur).sens_rotation = DONTMOVE;
    }
    
	if( moteur->sens_rotation == SENS_HORAIRE)
			
		{	
			switch((*moteur).state_MOTOR_PWM)
			{
			case 0:
				Set_GPIO((*moteur).voie0); //PD0 si moteur0, PD4 si moteur1, PD8 si moteur2, PD12 si moteur3
				Reset_GPIO((*moteur).voie1);
				Set_GPIO((*moteur).voie2);
				Reset_GPIO((*moteur).voie3);
				(*moteur).state_MOTOR_PWM++;
				
			break;
			
			case 1:
				Reset_GPIO((*moteur).voie0);
				Set_GPIO((*moteur).voie1);
				Set_GPIO((*moteur).voie2);
				Reset_GPIO((*moteur).voie3);
				(*moteur).state_MOTOR_PWM++;
			break;
			
			case 2:
				Reset_GPIO((*moteur).voie0);
				Set_GPIO((*moteur).voie1);
				Reset_GPIO((*moteur).voie2);
				Set_GPIO((*moteur).voie3);
				(*moteur).state_MOTOR_PWM++;
			break;
			
			case 3:
				Set_GPIO((*moteur).voie0);
				Reset_GPIO((*moteur).voie1);
				Reset_GPIO((*moteur).voie2);
				Set_GPIO((*moteur).voie3);
				(*moteur).state_MOTOR_PWM++;
				moteur->position = (moteur->position)+1;
			break;
			
			default: 
				Reset_GPIO((*moteur).voie0);
				Reset_GPIO((*moteur).voie1);
				Reset_GPIO((*moteur).voie2);
				Reset_GPIO((*moteur).voie3);
				(*moteur).state_MOTOR_PWM =0;
			}
		
		}
		
		else if ( moteur->sens_rotation == SENS_ANTIHORAIRE)
		
		{
			switch((*moteur).state_MOTOR_PWM)
					{
					case 0:
						Set_GPIO((*moteur).voie2);
						Reset_GPIO((*moteur).voie3);
						Set_GPIO((*moteur).voie0);
						Reset_GPIO((*moteur).voie1);
						(*moteur).state_MOTOR_PWM++;
					break;
					
					case 1:
						Reset_GPIO((*moteur).voie2);
						Set_GPIO((*moteur).voie3);
						Set_GPIO((*moteur).voie0);
						Reset_GPIO((*moteur).voie1);
						(*moteur).state_MOTOR_PWM++;
					break;
					
					case 2:
						Reset_GPIO((*moteur).voie2);
						Set_GPIO((*moteur).voie3);
						Reset_GPIO((*moteur).voie0);
						Set_GPIO((*moteur).voie1);
						(*moteur).state_MOTOR_PWM++;
					break;
					
					case 3:
						Set_GPIO((*moteur).voie2);
						Reset_GPIO((*moteur).voie3);
						Reset_GPIO((*moteur).voie0);
						Set_GPIO((*moteur).voie1);
						(*moteur).state_MOTOR_PWM++;
						moteur->position = (moteur->position)-1;
					break;
					
					default: 
						Reset_GPIO((*moteur).voie0);
						Reset_GPIO((*moteur).voie1);
						Reset_GPIO((*moteur).voie2);
						Reset_GPIO((*moteur).voie3);
						(*moteur).state_MOTOR_PWM =0;
		}
					
		if((*moteur).state_MOTOR_PWM>= 4)
			{
			(*moteur).state_MOTOR_PWM =0;
			}	
		}
}

void Init_PIT(void)
{
	PIT.MCR.R = 0x00000001;	/* Enable PIT and configure to stop in debug mode */
	PIT.MCR.B.MDIS = 1; 	/*disable PIT module */
}

void Set_Counter_Value_PIT(uint8_t timer, uint32_t TimeOut) //count down and generate interrupt
{
	PIT.CH[timer].LDVAL.R = TimeOut; 	
}

void Start_PIT(uint8_t timer) 
{ 	
	PIT.CH[timer].TCTRL.B.TEN = 1;
	PIT.MCR.B.MDIS = 0; /*enable PIT module */
}

void Disable_PIT(uint8_t timer) 
{ 	
	PIT.CH[timer].TCTRL.B.TEN = 0;
}

void Autorize_IT_PIT(uint8_t timer) 
{ 	
	PIT.CH[timer].TCTRL.B.TIE = 1;
}

void Clear_Flag_PIT(uint8_t timer)
{
	//raz du flag interruption PITx
	PIT.CH[timer].TFLG.B.TIF = 1;
}

//Initialisation et allumage des voyants + rétroéclairage tableau de bord au démarrage
void Set_LEDS_Cluster()
{
	  //config des sorties voyants + rétroéclairage tableaude bord et aiguilles
	  
	  Config_port_output(PC1);
	  Config_port_output(PC2);
	  Config_port_output(PC3);
	  Config_port_output(PK11);  
	  Config_port_output(PK4);
	  Config_port_output(PK5);
	  Config_port_output(PK2);
	  Config_port_output(PK3);
	  Config_port_output(PK6);
	  Config_port_output(PK10);
	  Config_port_output(PF5);
	  Config_port_output(PF1);
	  Config_port_output(PF3);
	  Config_port_output(PF4);
	  Config_port_output(PF5);
	  Config_port_output(PF6);
	  Config_port_output(PM7);
	  Config_port_output(PB7);
	  //Pour PJ3 et PJ7
	  Config_port_output_logical_inverted(PJ3);
	  Config_port_output_logical_inverted(PJ7); 
	  
	  //allumage voyants 
	  //voyant airbag (PC1 et PB7)
	  Set_GPIO(PC1);
	  Set_GPIO(PB7);
	  
	  //voyant feu de route (PK11)
	  Set_GPIO(PK11);
	  
	  //voyants cligno droite et gauche (PK6 + PC2 et PC3)
	  Set_GPIO(PK6);
	  Set_GPIO(PC2);
	  Set_GPIO(PC3);
	  
	  //voyants ESP (PK4 + PC3)
	  Set_GPIO(PK4);
	  
	  //voyants défaillance des freins (PK3 + PC3) et voyants ceintures non bouclées (PK3 + PC2)
	  Set_GPIO(PK3);
	  
	  //voyants autodiagnostic (PK5 + PC2)
	  Set_GPIO(PK5);
	  
	  //voyants feu de position (PM7 + PC3)
	  Set_GPIO(PM7);
	  
	  //voyants insuffisance batterie (PF3 + PC2)
	  Set_GPIO(PF3);
	  
	  
	  //Allumage retroéclairage tableau de bord
	  Set_GPIO(PJ3);
	  //Allumage retroéclairage des aiguilles
	  Set_GPIO(PJ7);
	  
	  
}


//configuration des I/O du DCU
void CONFIG_DCU_IO()
{
	//signaux à configurer en sortie:
	//DCU_PCLK: PG11 (OBE à activer), Fast Pad
	//DCU_VSYNC et DCU_HSYNC: PG8 et PG9
	//DCU_TAG: non utilisé
	//DCU_DE: PG10
	//DCU_R[7:2]: PA[7:2]
	//DCU_G[7:0]: PA[15:10]
	//DCU_B[7:2]: PG[7:2]
	//pour tous, fonction alternative 1 PA[1:0] = 1 dans les registres PCR
	
	//il faut aussi configurer le port PH5 en sortie et le mettre à  1 car elle commande la pin Reset de l'afficheur
	//On met aussi à '1' PG0 qui commande PWM_BL (commande du backlight)
	
	//config du port A (2 à 7):
	Config_port_AF(PA2,1);
	Config_port_AF(PA3,1);
	Config_port_AF(PA4,1);
	Config_port_AF(PA5,1);
	Config_port_AF(PA6,1);
	Config_port_AF(PA7,1);
	
	//config du port A (10 à 15):
	Config_port_AF(PA10,1);
	Config_port_AF(PA11,1);
	Config_port_AF(PA12,1);
	Config_port_AF(PA13,1);
	Config_port_AF(PA14,1);
	Config_port_AF(PA15,1);
	
	
	//config du port G (2 à 10), hormis PG[11] = PCLK
	Config_port_AF(PG2,1);
	Config_port_AF(PG3,1);
	Config_port_AF(PG4,1);
	Config_port_AF(PG5,1);
	Config_port_AF(PG6,1);
	Config_port_AF(PG7,1);
	Config_port_AF(PG8,1);
	Config_port_AF(PG9,1);
	Config_port_AF(PG10,1);

	//PG[11} = PCLK, fonction alternative 1 + OBE = 1
	//Config_port_AF(PG11,1);
	//Config_port_output(PG11);
	SIU.PCR[97].R = 0x600; //fonction alternative 1 + OBE = 1
	
	//PH[5] = reset afficheur, en mode sortie et à '1'
	Config_port_output(PH5);
	Set_GPIO(PH5);

	
	//PG0 = Backlight, en mode sortie et à '1'
	Config_port_output(PG0);
	Set_GPIO(PG0);
	
	//PG12 = alim 3.3 V du LCD, en mode sortie et à '1'
	Config_port_output(PG12);
	Set_GPIO(PG12);
}


//configuration du DCU3 en mode de test
void CONFIG_DCU()
{
	//activation du TCON
	//To enable TTL mode, set TCON_CTRL1[RSDS_MODE]=’0’, TCON_CTRL1[TCON_BYPASS]=’0’ and TCON_CTRL1[TCON_EN]=’1’.
	TCON.CTRL1.B.RSDS_MODE = 0; 
	TCON.CTRL1.B.TCON_BYPASS = 1; //En fait, si on met Bypass à 0, on n'arrive pas à sortir les signaux HSYN, VSYN et DE !!!
	TCON.CTRL1.B.TCON_EN = 0;  //pour l'instant, on désactive le TCON, qui semble inutile. 
	
	
	//config taille de l'écran (480 x 272 px) --> DISP_SIZE
	DCU.DISP_SIZE.B.DELTA_Y = RESO_VERT; //résolution verticale en nombre de pixels
	DCU.DISP_SIZE.B.DELTA_X = RESO_HOR; //résolution horizontale en multiple de 16 pixels
	
	//configuration des paramètres temporels
	//DIV_RATIO --> pour fixer la clock pixel. Horloge système = 100 MHz. Horloge pixel à 10 MHz --> facteur de division = 9 
	//--> DIV_RATIO = 9
	DCU.DIV_RATIO.R = 0x00000009;
	//paramètres de synchro horizontale : HSYN_PARA (FP_H, BP_H, and PW_H)
	DCU.HSYN_PARA.B.PW_H = 5; //l'impulsion sur DCU_HSYNC dure 5 pixel clocks (on a mesure une impulsion nég de 500 ns).
	//Impulsion nég sur DCU_HSYNC toutes les 58.1 µs. Il reste donc 58.1-0.5-480*0.1 = 9.6 µs pour BP_H et FP_H.
	//On prend BP_H = FP_H = 4.8 µs soit 48 pixel clocks.
	DCU.HSYN_PARA.B.BP_H = 48;
	DCU.HSYN_PARA.B.FP_H = 48;
	//paramètres de synchro verticale : VSYN_PARA (FP_V, BP_V, and PW_V)
	DCU.VSYN_PARA.B.PW_V = 2; //l'impulsion sur DCU_HSYNC dure 116.2 µs soit 2 cycles horizontaux.
	//Impulsion nég sur DCU_VSYNC toutes les 20 ms. Il reste donc 20e3-2*58.1-272*58.11 = 4080.6 µs, soit 70 cycles horiz pour BP_V et FP_V.
	//On prend BP_V = FP_V = 11 cyncles horizontaux.
	DCU.VSYN_PARA.B.BP_V = 39;
	DCU.VSYN_PARA.B.FP_V = 31;
	//Polarité des signaux de synchro : négative pour tous les signaux (HSYNC, VSYNC, DE)
	DCU.SYN_POL.B.INV_PXCK = 0; //display samples on falling edges
	DCU.SYN_POL.B.NEG = 0; //pas de négation des données pixels
	DCU.SYN_POL.B.BP_VS = 0; //do not bypass VSYNC
	DCU.SYN_POL.B.BP_HS = 0; //do not bypass HSYNC
	DCU.SYN_POL.B.INV_VS = 1; //VSYNC active low
	DCU.SYN_POL.B.INV_HS = 1; //HSYNC active low
	
	//Rq : les tests montrent en mode test que cela fonctionne queque soit INV_VS, INV_HS,INV_PXCK.
	
	//Background color : fond de couleur blanc-gris
	DCU.BGND.B.BGND_R = 0xAA;
	DCU.BGND.B.BGND_G = 0xAA;
	DCU.BGND.B.BGND_B = 0xAA;
}


//initialisation des paramètres des layers graphiques
void InitLayers()
{
	//Initialisation du layer1
	DCU.LAYER[0].CTRLDESCL1.B.HEIGHT = Layer1_H;
	DCU.LAYER[0].CTRLDESCL1.B.WIDTH = Layer1_W;
	DCU.LAYER[0].CTRLDESCL2.B.POSX = Layer1_posX;
	DCU.LAYER[0].CTRLDESCL2.B.POSY = Layer1_posY;
	DCU.LAYER[0].CTRLDESCL3.R = Adress1;
		
	DCU.LAYER[0].CTRLDESCL4.B.EN = 1;
	DCU.LAYER[0].CTRLDESCL4.B.DATA_SEL = 0; //données en mémoire
	DCU.LAYER[0].CTRLDESCL4.B.TRANS = 0xFF; //pas de selction de pixels où sera gérée la transparence. 
	DCU.LAYER[0].CTRLDESCL4.B.BPP = 0; //format 1 bpp. Couleur défini dans la CLUT (noir ou blanc)
	DCU.LAYER[0].CTRLDESCL4.B.LUOFFS = 0; //valeurs situées au début de la CLUT
	DCU.LAYER[0].CTRLDESCL4.B.AB = 2; //blend the whole frame  
	
	//Initialisation du layer2
	DCU.LAYER[1].CTRLDESCL1.B.HEIGHT = Layer2_H;
	DCU.LAYER[1].CTRLDESCL1.B.WIDTH = Layer2_W;
	DCU.LAYER[1].CTRLDESCL2.B.POSX = Layer2_posX;
	DCU.LAYER[1].CTRLDESCL2.B.POSY = Layer2_posY;
	DCU.LAYER[1].CTRLDESCL3.R = Adress2;
			
	DCU.LAYER[1].CTRLDESCL4.B.EN = 1;
	DCU.LAYER[1].CTRLDESCL4.B.DATA_SEL = 0; //données en mémoire
	DCU.LAYER[1].CTRLDESCL4.B.TRANS = 0xFF; //pas de selction de pixels où sera gérée la transparence. 
	DCU.LAYER[1].CTRLDESCL4.B.BPP = 1; //format 2 bpp. Couleur défini dans la CLUT (4 couleurs)
	DCU.LAYER[1].CTRLDESCL4.B.LUOFFS = 2; //valeurs situées à offset = 2 % début de la CLUT
	DCU.LAYER[1].CTRLDESCL4.B.AB = 2; //blend the whole frame 
	
	//Initialisation du layer3
	DCU.LAYER[2].CTRLDESCL1.B.HEIGHT = Layer3_H;
	DCU.LAYER[2].CTRLDESCL1.B.WIDTH = Layer3_W;
	DCU.LAYER[2].CTRLDESCL2.B.POSX = Layer3_posX;
	DCU.LAYER[2].CTRLDESCL2.B.POSY = Layer3_posY;
	DCU.LAYER[2].CTRLDESCL3.R = Adress3;
			
	DCU.LAYER[2].CTRLDESCL4.B.EN = 1;
	DCU.LAYER[2].CTRLDESCL4.B.DATA_SEL = 0; //données en mémoire
	DCU.LAYER[2].CTRLDESCL4.B.TRANS = 0xFF; //pas de selction de pixels où sera gérée la transparence. 
	DCU.LAYER[2].CTRLDESCL4.B.BPP = 2; //format 4 bpp. Couleur défini dans la CLUT (4 couleurs)
	DCU.LAYER[2].CTRLDESCL4.B.LUOFFS = 6; //valeurs situées à offset = 6 % début de la CLUT
	DCU.LAYER[2].CTRLDESCL4.B.AB = 2; //blend the whole frame 	
	
	//Initialisation du layer4 (cadre en haut à gauche, simple liseré noir en bord de cadre, le reste est transparent)
	DCU.LAYER[3].CTRLDESCL1.B.HEIGHT = Layer4_H;
	DCU.LAYER[3].CTRLDESCL1.B.WIDTH = Layer4_W;
	DCU.LAYER[3].CTRLDESCL2.B.POSX = Layer4_posX;
	DCU.LAYER[3].CTRLDESCL2.B.POSY = Layer4_posY;
	DCU.LAYER[3].CTRLDESCL3.R = Adress4;
		
	DCU.LAYER[3].CTRLDESCL4.B.EN = 1;
	DCU.LAYER[3].CTRLDESCL4.B.DATA_SEL = 0; //données en mémoire
	DCU.LAYER[3].CTRLDESCL4.B.TRANS = 0xFF; //pas de selction de pixels où sera gérée la transparence. 
	DCU.LAYER[3].CTRLDESCL4.B.BPP = 0; //format 1 bpp --> liseré en noir. Couleur défini dans la CLUT.
	DCU.LAYER[3].CTRLDESCL4.B.LUOFFS = 0; //valeurs situées au début de la CLUT
	DCU.LAYER[3].CTRLDESCL4.B.AB = 2; //blend the whole frame	
	
	//Initialisation du layer5 (cadre en haut à droite, simple liseré noir en bord de cadre, le reste est transparent)
	DCU.LAYER[4].CTRLDESCL1.B.HEIGHT = Layer5_H;
	DCU.LAYER[4].CTRLDESCL1.B.WIDTH = Layer5_W;
	DCU.LAYER[4].CTRLDESCL2.B.POSX = Layer5_posX;
	DCU.LAYER[4].CTRLDESCL2.B.POSY = Layer5_posY;
	DCU.LAYER[4].CTRLDESCL3.R = Adress5;
			
	DCU.LAYER[4].CTRLDESCL4.B.EN = 1;
	DCU.LAYER[4].CTRLDESCL4.B.DATA_SEL = 0; //données en mémoire
	DCU.LAYER[4].CTRLDESCL4.B.TRANS = 0xFF; //pas de selction de pixels où sera gérée la transparence. 
	DCU.LAYER[4].CTRLDESCL4.B.BPP = 0; //format 1 bpp --> liseré en noir. Couleur défini dans la CLUT.
	DCU.LAYER[4].CTRLDESCL4.B.LUOFFS = 0; //valeurs situées au début de la CLUT
	DCU.LAYER[4].CTRLDESCL4.B.AB = 2; //blend the whole frame	
	
	//Initialisation du layer6 (cadre en bas, simple liseré noir en bord de cadre, le reste est transparent)
	DCU.LAYER[5].CTRLDESCL1.B.HEIGHT = Layer6_H;
	DCU.LAYER[5].CTRLDESCL1.B.WIDTH = Layer6_W;
	DCU.LAYER[5].CTRLDESCL2.B.POSX = Layer6_posX;
	DCU.LAYER[5].CTRLDESCL2.B.POSY = Layer6_posY;
	DCU.LAYER[5].CTRLDESCL3.R = Adress6;
				
	DCU.LAYER[5].CTRLDESCL4.B.EN = 1;
	DCU.LAYER[5].CTRLDESCL4.B.DATA_SEL = 0; //données en mémoire
	DCU.LAYER[5].CTRLDESCL4.B.TRANS = 0xFF; //pas de selction de pixels où sera gérée la transparence. 
	DCU.LAYER[5].CTRLDESCL4.B.BPP = 0; //format 1 bpp --> liseré en noir. Couleur défini dans la CLUT.
	DCU.LAYER[5].CTRLDESCL4.B.LUOFFS = 0; //valeurs situées au début de la CLUT
	DCU.LAYER[5].CTRLDESCL4.B.AB = 2; //blend the whole frame   
}

//initialisation du curseur
void InitCursor() {
	
	DCU.CTRLDESCCURSOR1.B.HEIGHT = Cursor_H;
	DCU.CTRLDESCCURSOR1.B.WIDTH = Cursor_W;
	DCU.CTRLDESCCURSOR2.B.POSX = Cursor_posX;
	DCU.CTRLDESCCURSOR2.B.POSY = Cursor_posY;
	DCU.CTRLDESCCURSOR3.B.CURSOR_DEFAULT_COLOR = 0x000000; //au format RGB888 --> noir
	DCU.CTRLDESCCURSOR3.B.CUR_EN = 1; //activation du curseur
	DCU.CTRLDESCCURSOR4.B.EN_BLINK = 1; //autorisation du blink mode
	DCU.CTRLDESCCURSOR4.B.HWC_BLINK_ON = 50; //durée de la période de blink_on (en nombre de trame. On rappelle qu'on a 50 trames par seconde)
	DCU.CTRLDESCCURSOR4.B.HWC_BLINK_OFF = 50; 
}

//activation du DCU
void activ_DCU_Test()
{
	//DCU_mode : mode normal
	DCU.DCU_MODE.B.DCU_MODE = 3; //mode test
	DCU.DCU_MODE.B.RASTER_EN = 1; //démarrage du balayage de pixel

}

//activation du DCU
void activ_DCU_Normal()
{
	//DCU_mode : mode normal
	DCU.DCU_MODE.B.DCU_MODE = 1; //mode normal
	DCU.DCU_MODE.B.RASTER_EN = 1; //démarrage du balayage de pixel
}


//autorisation des interruptions masquables
void enableIrq(void) {
  INTC.CPR.B.PRI = 0;          /* Single Core: Lower INTC's current priority */
  asm("wrteei 1");	    	   /* Enable external interrupts */
  
  //autorisation des interruptions liées aux timers 0 et 1
  INTC_InstallINTCInterruptHandler(PIT_CH0_ISR,59,2);//function, PSR, priority
  INTC_InstallINTCInterruptHandler(PIT_CH1_ISR,60,3);//function, PSR, priority
  INTC_InstallINTCInterruptHandler(PIT_CH2_ISR,61,3);//function, PSR, priority  
} 


/*========================================================================*/
/*                            MAIN                                        */
/*========================================================================*/

int main(void) {
  volatile int i = 0;
  //Timer 0 : cadence d'un step pour les moteurs pas à pas des afficheurs à aiguille
  //on fait tourner les moteurs pas à pas d'un pas toutes les 4 ms --> full step en 16 ms.
  uint32_t TimeOut_TIMER0 = 400000; //t = 4 ms --> full step = 16 ms : POUR LE DEPLACEMENT DES AIGUILLES 
  
  //Timer 1 : séquençage de la machine état de la démo
  uint32_t TimeOut_TIMER1 = 300000000; //t=3s POUR INITIALISER LA POSITION DES AIGUILLES, on revient en butée.
  
  //Timer 2 : période d'allumage/extinction des warnings. Démarrage après l'initialisation
  uint32_t TimeOut_TIMER2 = 50000000; //t = 0.5 s

  DISABLE_WATCHDOG();
  MC_MODE_INIT_PLL();   //mode Run0, utilisation de la PLL principale tournant à 200 MHz --> horloge système = 100 MHz.
  
  Set_LEDS_Cluster();
  
  //initialisation des timers : 0 pour les aiguilles, 1 pour l'initialisation de la machine à état, 2 pour le clignotement des warnings
  Init_PIT(); 
  Set_Counter_Value_PIT(TIMER0, TimeOut_TIMER0);
  Set_Counter_Value_PIT(TIMER1, TimeOut_TIMER1);
  Set_Counter_Value_PIT(TIMER2, TimeOut_TIMER2); //lancement des warnings
  	
  //initialisation des moteurs pas à pas associés aux afficheurs à aiguille (pads, positions à 0)
  init_struct_moteur(MOTEUR0, pointeur_moteur0);
  init_struct_moteur(MOTEUR1, pointeur_moteur1);
  init_struct_moteur(MOTEUR2, pointeur_moteur2);
  init_struct_moteur(MOTEUR3, pointeur_moteur3);  
  motor_init(pointeur_moteur0);
  motor_init(pointeur_moteur1);
  motor_init(pointeur_moteur2);
  motor_init(pointeur_moteur3);
  
  //intiialisation DCU (en mode de test)
  InitLayerMemory();
  InitLayers();
  CONFIG_DCU();
  CONFIG_DCU_IO();
  InitCursor();
  activ_DCU_Test();
  	  	
  enableIrq();	/*Autorisation des interruptions masquables du programme, priorité = 0*/ 

  Autorize_IT_PIT(TIMER0);
  Clear_Flag_PIT(TIMER0);
  Start_PIT(TIMER0);
    
  Autorize_IT_PIT(TIMER1);
  Clear_Flag_PIT(TIMER1);
  Start_PIT(TIMER1);

  /* Loop forever */
  for (;;) {
    i++;
  }
}


/*========================================================================*/
/*                          ISR FUNCTIONS                                 */
/*========================================================================*/

//interruption pour le pilotage des moteurs pas à pas des jauges à aiguilles
void PIT_CH0_ISR(void)
{
	Clear_Flag_PIT(TIMER0);
	
	switch(etat_actuel)
	{
	//démarrage : intialisation de la position des aiguilles
	case Init:
		
		init_position(pointeur_moteur0); //position à 0km/h
		init_position(pointeur_moteur1);
		init_position(pointeur_moteur2);
		init_position(pointeur_moteur3);
		
		if(init_finished == 1)
		{	
			(*pointeur_moteur0).position=0;
			(*pointeur_moteur1).position=0;
			(*pointeur_moteur2).position=0;
			(*pointeur_moteur3).position=0;
			init_finished =0;
			etat_actuel = Phase1;
		}
		
	break;
	
	//phase 1 : les aiguilles sont montées rapidement vers des positions données depuis la position 0 (1 full step en 20 ms)
	case Phase1:

		step_to(120,pointeur_moteur0); //200km/h équivalent à 200° depuis 0km/h, soit 200/(5/3) = 120 full steps  
		step_to(108,pointeur_moteur1); //régime moteur = 4000 tr/min équivalent à 180° depuis 0tr/min, soit 180/(5/3) = 108 full steps  
		step_to(30,pointeur_moteur2);  //au mlieu de la jauge --> +45° donc 45°/1.5 = 30 full steps
		step_to(30,pointeur_moteur3);  //au mlieu de la jauge --> +45° donc 45°/1.5 = 30 full steps

	break;
	
	//phase 2 : les aiguilles sont descendues lentement vers des positions données (1 full step en 60 ms) depuis la position de la phase 1 ou 3
	case Phase2:
		step_to(54,pointeur_moteur0); //90km/h équivalent à 90°, soit 90/(5/3) = 54 full steps  
		step_to(54,pointeur_moteur1); //régime moteur = 2000 tr/min équivalent à 90° depuis 0tr/min, soit 90/(5/3) = 54 full steps 	

	break;
	
	//phase 3 : les aiguilles sont montées rapidement vers des positions données (1 full step en 20 ms) depuis la position de la phase 2
	case Phase3:
		step_to(120,pointeur_moteur0); //200km/h équivalent à 200° depuis 0km/h, soit 200/(5/3) = 120 full steps   
		step_to(108,pointeur_moteur1); //régime moteur = 4000 tr/min équivalent à 180° depuis 0tr/min, soit 180/(5/3) = 108 full steps 	

	break;
	
	case Phase4:
//
	break;
	
	case Phase5:
	break;	
	
	default:
		Disable_PIT(TIMER0);
		Disable_PIT(TIMER1);
	break;
	}
}

//Interruption pour le séquençage de la machine à état de la démo
void PIT_CH1_ISR(void)
{
	uint32_t TimeOut_TIMER1_PHASE1 = 300000000; //3 secondes
	uint32_t TimeOut_TIMER1_PHASE2 = 500000000; //5 secondes
	uint32_t TimeOut_TIMER1_PHASE3 = 300000000; //3 secondes
	
	uint32_t TimeOut_TIMER0_PHASE1 = 400000; //1 full step = 16 ms
	uint32_t TimeOut_TIMER0_PHASE2 = 1250000;  //1 full step = 50 ms
	uint32_t TimeOut_TIMER0_PHASE3 = 500000;  //1 full step = 20 ms
	
	Clear_Flag_PIT(TIMER1);
	Disable_PIT(TIMER1);
	
	if(etat_actuel == Init)
	{
	init_finished = 1;
	//démarrage clignotement des warnings, cadencé par le TIMER2
	Autorize_IT_PIT(TIMER2);
	Clear_Flag_PIT(TIMER2);
	Start_PIT(TIMER2);
	
	//sortie du mode Test du DCU et entrée dans le mode normal
	DCU.DCU_MODE.B.RASTER_EN = 0; //arrêt du balayage de pixel
	activ_DCU_Normal();
	
	//passage dans la phase 1
	Disable_PIT(TIMER0);
	Set_Counter_Value_PIT(TIMER0,TimeOut_TIMER0_PHASE1);
	Start_PIT(TIMER0);
	//changement de la période de Time Out du Timer 1 : passage à 3 secondes
	Set_Counter_Value_PIT(TIMER1,TimeOut_TIMER1_PHASE1);
	Start_PIT(TIMER1);
	}
	
	else if (etat_actuel == Phase1) { 
		etat_actuel = Phase2;
		Disable_PIT(TIMER0);
		Set_Counter_Value_PIT(TIMER0,TimeOut_TIMER0_PHASE2);
		Start_PIT(TIMER0);
		Set_Counter_Value_PIT(TIMER1,TimeOut_TIMER1_PHASE2);
		Start_PIT(TIMER1);
	}
	else if (etat_actuel == Phase2) { 
		etat_actuel = Phase3;
		Disable_PIT(TIMER0);
		Set_Counter_Value_PIT(TIMER0,TimeOut_TIMER0_PHASE3);
		Start_PIT(TIMER0);
		Set_Counter_Value_PIT(TIMER1,TimeOut_TIMER1_PHASE3);
		Start_PIT(TIMER1);
	}
	else if (etat_actuel == Phase3) { 
		etat_actuel = Phase2;
		Disable_PIT(TIMER0);
		Set_Counter_Value_PIT(TIMER0,TimeOut_TIMER0_PHASE2);
		Start_PIT(TIMER0);
		Set_Counter_Value_PIT(TIMER1,TimeOut_TIMER1_PHASE2);
		Start_PIT(TIMER1);
	}
	else {
	}	
	
}


//Interruption pour le contrôle des clignotants des warnings (période 1 s)
//En outre, on change l'heure indiquée sur l'écran LCD toutes les secondes
void PIT_CH2_ISR(void)
{
	Clear_Flag_PIT(TIMER2);
	
	if(State_Warning == 0)
	{
		Reset_GPIO(PK6);
		State_Warning = 1;
		
		Sec2++; 
		if (Sec2 == 10) {
			Sec1++;
			Sec2 = 0;
		}
		if (Sec1 == 6) {
			Sec1 = 0;
			Min2++;
		}
		if (Min2 == 10) {
			Min2 = 0;
			Min1++;
		}
		if (Min1 == 6) {
			Min1 = 0;
		}		
		Change_Time_LCD(Min1,Min2,Sec1,Sec2);
	}
	else {
		Set_GPIO(PK6);
		State_Warning = 0;
	}		
}



