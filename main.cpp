#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <mbed.h>
#include "VL53L0X.h"

//#define stop(x) ThisThread::sleep_for(x)
#define BLINKING_RATE     500ms
#define wait(x) ThisThread::sleep_for(x)

#define M1_STEP p30
#define M1_DIR p29
#define EN p28

#define POT p18

#define ANGLE_MAX 9
#define ANGLE_MIN -9
#define STEP_ANGLE 0.225
 
// VALEURS DU PID
#define Kp -0.1
#define Ki 0.000
#define Kd 0.000

float target_speed = 2;
float Sp = 160;
int mesure = 5000;
float erreur;
bool flag = false;

const int VITESSE_ANGULAIRE_MAX_DEG_SEC = 720;

// =========================================================================
// -- Fonctions
void manualspin(void);
float degree_to_delay(int degrees_persecond);
void stepper1(void);
float tension_degree(uint16_t V_in);
void init_capteur_distance(void);
float calcul_PID(int mesure);
void motorDrive(float PID);
void controle_moteur(void);


    // =========================================================================
// -- Objets
DigitalOut m1_step(M1_STEP);
DigitalOut m1_dir(M1_DIR);
DigitalOut ENABLED(EN);
DigitalOut led(LED1);

DigitalOut MS1(p5);
DigitalOut MS2(p6);
DigitalOut MS3(p7);

I2C         i2c(p9, p10);
VL53L0X     capteur_distance(&i2c);
DigitalOut  capteur_shutdown(p8);

AnalogIn speed(POT);

Timeout time1;
// =========================================================================


// =========================================================================
// -- Variables
float degree_tension;
float angle = 0;
float derniere_erreur;
int last_target;
// =========================================================================


//================================================ THIS IS MAIN
int main() 
{
    printf("------------------Reset------------------");
    m1_step = 0; // position initiale
    m1_dir = 1;
    init_capteur_distance();
    while(mesure>1000 || mesure<-1000){
        mesure = capteur_distance.getRangeMillimeters();
    }
    MS1 = 1;
    MS2 = 1;
    MS3 = 0;
    time1.attach(&controle_moteur, degree_to_delay(target_speed)); 

    while(true)
    {
        mesure = capteur_distance.getRangeMillimeters();
        

        printf("\n\r Ang: %7.2f | Mes: %d mm | Err: %7.2f mm | PID: %8.2f | Spd: %7.2f  |",angle,mesure,erreur,  calcul_PID(mesure), target_speed);
         
    }
}

float degree_to_delay(int degrees_persecond){
    return(1/((degrees_persecond/1.8)*2)); // transforme les degrees par secondes en temps haut et bas pour la bonne fréquence de rotation
}

float vitesse_angulaire_sortie_pid;

void controle_moteur(void)
{
    if(angle <= ANGLE_MIN){
           if (target_speed<=0){
                target_speed = 0;
            }
        }else if(angle >= ANGLE_MAX){
            if (target_speed>=0){
                target_speed = 0;
           }
        } 

    if (target_speed != 0) {
       // choix direction en fonction du signe de la vitesse_angulaire_sortie_pid
        if(target_speed < 0){
            m1_dir = 0;
        }else if(target_speed > 0){
            m1_dir = 1;
        }
        
        m1_step = 1;
        wait_us(2);
        m1_step = 0;
        wait_us(2);

        if(m1_dir == 0){
            angle -= STEP_ANGLE;
        }else if(m1_dir == 1){
            angle += STEP_ANGLE;
        }

        //angle ++ ou angle-- en fonction de la direction
        
        //delai en fonction de la vitesse
        time1.attach(&controle_moteur, degree_to_delay(target_speed)); 
    }else{
        //delai =  calculer le delai 
        time1.attach(&controle_moteur, degree_to_delay(1)); 
    }

}
float I= 0;
float calcul_PID(int mesure){
    erreur = Sp-mesure;
    float P = Kp * erreur; 
     I = I + (Ki * erreur); 
    float D = Kd*(erreur - derniere_erreur);
    derniere_erreur = erreur;
    float PID = P;

        target_speed = PID;

    return PID;

}



void init_capteur_distance(void){
    capteur_shutdown = 1;  //turn VL53L0X on
    capteur_distance.init();
    capteur_distance.setModeContinuous();
    capteur_distance.startContinuous();
}


