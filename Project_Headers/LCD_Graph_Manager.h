/*
 * LCD_Graph_Manager.h
 *
 *  Created on: Feb 15, 2016
 *      Author: Alexandre Boyer
 *      Gestion des éléments graphiques à afficher sur l'écran LCD
 */

#ifndef LCD_GRAPH_MANAGER_H_
#define LCD_GRAPH_MANAGER_H_

//paramètres écran LCD
#define RESO_VERT 272
#define RESO_HOR 30;

//Info sur les caractères et images stockées en mémoire
//Caractères alpha numériques (taille en pixels)
#define W_Char 32
#define H_Char 32
#define PixelPerBit_Char 32 //32 pixel par mot de 32 bits --> 1 bpp

//Image INSA (taille en pixels)
#define W_Image_INSA 384
#define H_Image_INSA 83
#define PixelPerWord_Image_INSA 8 //8 pixels par mot de 32 bits --> 4 bpp

#define PixelPerWord_2bpp 16 //16 pixels par mot de 32 bits --> 2 bpp

#define PixelPerWord_Cadre 32 //32 pixels par mot de 32 bits --> 1 bpp


//Définition des espaces mémoires des différentes couches graphiques. Attention à bien déterminer l'espace mémoire occupée 
//par une couche pour éviter toute erreur d'affichage. Veiller à ce que le début des adresses des layers soient des multiples de 64 !
//mémoire graphique SRAM : de 0x60000000 à 0x600FFFFF
//On range dans l'ordre :
//- Layer 4
//- Layer 5
//- Layer 6
//- Layer 1
//- Layer 2
//- Layer 3
//- Image INSA
//- Caractères alphanumériques


//On met l'ensemble des symboles graphiques qui seront affichés sur Layer1. Il est centré sur le cadre en haut à gauche (Layer4).
//Format 1 bpp --> (160*32 px /8 (1bpp) = 640 octets)
//attention, le contenu de la mémoire doit démarrer à une adresse multiple de 64 !
#define Layer1_W 160  //multiple de 32 car format 1 bpp
#define Layer1_H 32
#define Layer1_posX 37 
#define Layer1_posY 14  
#define Adress1 0x60003BC0  //offset de 12420 octets (0XC21)  % Layer 6


//Dans layer2, on va faire apparaitre le drapeau français dans le cadre en haut à droite
//Puisqu'il y a 3 couleurs, on n'a besoin que d'un format 2bpp.
//On veut un drapeau de width = 3*16 px = 48px et height = 40 px
//--> 48*40 px/ 4 px/octets = 480 octets --> 120 mots de 32 bits.
//attention, le contenu de la mémoire doit démarrer à une adresse multiple de 64 !
#define Layer2_W 48  //multiple de 16 car format 2 bpp
#define Layer2_H 40
#define Layer2_posX 339
#define Layer2_posY 10
#define Adress2 0x60003E40  //offset de 256 octets (0X100)  % Layer 1 


//Dans layer3, on va faire apparaitre le logo INSA dans le cadre du bas
//L'image est au format 4bpp. --> 384*83 px/ 2 px/octets = 15936 octets --> 3984 mots de 32 bits.
//attention, le contenu de la mémoire doit démarrer à une adresse multiple de 64 !
#define Layer3_W 384  //multiple de 8 car format 4 bpp
#define Layer3_H 83
#define Layer3_posX 48
#define Layer3_posY 122
#define Adress3 0x60004040  //offset de 480 octets (0X1E0)  % Layer 2.


//cadre en haut à gauche (224*50 px --> 11200 bits nécessaires (format 1bpp) = 1400 octets. 
//Mis à l'adresse 0 de la SRAM graphique.
#define Layer4_W 224 //multiple de 32 car format 1 bpp (7*32)
#define Layer4_H 50
#define Layer4_posX 5
#define Layer4_posY 5
#define Adress4 0x60000000  
#define Adress_Layer4 *(uint32_t *) Adress4  

//cadre en haut à droite (224*50 px --> 11200 bits nécessaires (format 1bpp) = 1400 octets. 
//attention, le contenu de la mémoire doit démarrer à une adresse multiple de 64 !
#define Layer5_W 224 //multiple de 32 car format 1 bpp (7*32)
#define Layer5_H 50
#define Layer5_posX 251
#define Layer5_posY 5
#define Adress5  0x60000580  //offset de 1408 octets (0X580) % Layer 4
#define Adress_Layer5 *(uint32_t *) Adress5  

//cadre en en bas (480*207 px --> 99360 bits nécessaires (format 1bpp) =  12420 octets (12420 est un multiple de 32)
//attention, le contenu de la mémoire doit démarrer à une adresse multiple de 64 !
#define Layer6_W 480 //multiple de 32 car format 1 bpp (15*32)
#define Layer6_H 207
#define Layer6_posX 0
#define Layer6_posY 60
#define Adress6 0x60000B00 //offset de 1408 octets (0X580)  % Layer 5
#define Adress_Layer6 *(uint32_t *) Adress6 

//adresse de l'image INSA (384*83 pixels encodés en 4 bpp)
#define Adr_Image_INSA 0x60007E80

//adresse des caractères alphanumériques (32*32 pixels encodés en 1 bpp)
#define Adr_Char_0 0x600FDBE0
#define Adr_Char_1 0x600FDCE0
#define Adr_Char_2 0x600FDDE0
#define Adr_Char_3 0x600FDEE0
#define Adr_Char_4 0x600FDFE0
#define Adr_Char_5 0x600FE0E0
#define Adr_Char_6 0x600FE1E0
#define Adr_Char_7 0x600FE2E0
#define Adr_Char_8 0x600FE3E0
#define Adr_Char_9 0x600FE4E0

//Définition des symboles curseur hardware et CLUT

//curseur graphique, situé entre 0x0400 – 0x07FF % début de l'adresse du DCU = 0xFFE5C000
#define MemStart_Cursor 0xFFE5C400  //début de l'adresse mémoire curseur
#define Adress_Cursor *(uint32_t *) MemStart_Cursor
#define Cursor_W 32 //largeur du curseur en pixels
#define Cursor_H 32  //hauteur du curseur en pixels
#define Cursor_posX 101
#define Cursor_posY 14
#define PixelPerWord_Cursor 32 //32 pixels par mot de 32 bits --> 1 bpp

//CLUT
#define Start_CLUT 0xFFE5E000
#define Adress_Start_CLUT *(uint32_t *) Start_CLUT 

void InitLayerMemory(void);
void write_Time_Char(uint32_t * Pointer_Layer, uint32_t W_car, uint32_t H_car, uint8_t PixelPerWord, uint8_t CursorActivate, uint32_t * Pointer_CharMin1, uint32_t * Pointer_CharMin2, uint32_t * Pointer_CharSec1, uint32_t * Pointer_CharSec2);
void Change_Time_LCD(uint8_t M1, uint8_t M2, uint8_t S1, uint8_t S2);

#endif /* LCD_GRAPH_MANAGER_H_ */
