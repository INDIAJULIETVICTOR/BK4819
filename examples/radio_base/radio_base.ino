/*
 * Project Name: Radio Firmware
 * File: radio_base.c
 *
 * Copyright (C) 2024 Fabrizio Palumbo (IU0IJV)
 * 
 * This program is distributed under the terms of the MIT license.
 * You can obtain a copy of the license at:
 * https://opensource.org/licenses/MIT
 *
 * DESCRIPTION:
 * Implementation of the control system for the experimental radio module based on the Beken BK4819.
 *
 * AUTHOR: Fabrizio Palumbo
 * CREATION DATE: October 27, 2024
 *
 * CONTACT: t.me/IU0IJV
 *
 * NOTES:
 * - This implementation requires the following libraries:
 *
 *   - BK4819
 *   - IcomSYM
 *   - TaskScheduler
 * 
 * - Verify correct SPI pin configuration before use.
 */


#include <BK4819.h>
#include <TaskScheduler.h>
#include <IcomSim.h>

#define PIN_PTT A0
#define PIN_S1 3
#define PIN_S2 4
#define PIN_LED_TX 5
#define PIN_LED_RX 6
#define PIN_MUTE 7
#define PIN_PB_MONITOR 8
#define PIN_IRQ_BEKEN 9

// #include <SoftwareSerial.h>

//--------------------------------------------------------- Creazione dell'oggetto Scheduler
Scheduler runner;

//--------------------------------------------------------- Funzioni dei task
void keyboard();
void sercomm();
void interrupt();
void smeter();

//--------------------------------------------------------- Definizione dei task
Task irq  (1, TASK_FOREVER, &interrupt);
Task keyb (50, TASK_FOREVER, &keyboard);                         // task che gestisce una pulsantiera
Task serc (10, TASK_FOREVER, &sercomm);
Task rssi (500, TASK_FOREVER, &smeter);

//--------------------------------------------------------- Definizione del chip beken
BK4819 beken(10, 11, 12, 13);                                   // Passa i pin CS, MOSI, MISO, e SCK

IcomSim radio(Serial);                                          // usa la seriale di sistema USB

// #define RX_PIN 10                                            // pin usati da softwareserial
// #define TX_PIN 11

// SoftwareSerial mySerial(RX_PIN, TX_PIN);
// IcomSim radio(mySerial);                                     // Usa SoftwareSerial per la comunicazione seriale

//--------------------------------------------------------- Definizione variabili
uint32_t frequenza = 74025UL * 100;  
uint32_t step      = 1250UL;

volatile int posizione = 0;  // Posizione dell'encoder
volatile bool direzione;      // Direzione di rotazione

bool mute = true;
bool monitor = false;
bool Txon = false;
bool statoPrecedente = false;
bool toggleState = false;   
bool lastButtonState = HIGH;  

//=============================================================================================
//
//=============================================================================================
void setup() 
{
    Serial.begin(19200); 
    // mySerial.begin(9600);                                    // Inizializza SoftwareSerial

    pinMode(PIN_S1, INPUT);
    pinMode(PIN_S2, INPUT);

    pinMode(PIN_IRQ_BEKEN, INPUT_PULLUP);                       // Ingresso Linea IRQ del beken ( GPIO 4 )
    pinMode(PIN_PB_MONITOR, INPUT_PULLUP);                      // Pulsante MONITOR

    pinMode(PIN_MUTE, OUTPUT);                                  // abilitazione audio MUTE
    pinMode(PIN_LED_RX, OUTPUT);                                // Led Verde stato RX
    pinMode(PIN_LED_TX, OUTPUT);                                // Led Rosso stato TX

    digitalWrite(PIN_LED_TX, LOW);                              // led TX Spento
    digitalWrite(PIN_LED_RX, LOW);                              // led RX Spento
    digitalWrite(PIN_MUTE, HIGH);                               // Mute ON

    beken.BK4819_Init();                                        // Inizializza il dispositivo BK4819

    beken.BK4819_Set_AF(AF_FM);		                              // attiva demodulazione FM
    beken.BK4819_Set_Frequency ( frequenza );                   // imposta frequenza
    beken.BK4819_Set_Filter_Bandwidth(BK4819_FILTER_BW_10k);	  // imposta BW e filtri audio
    beken.BK4819_Squelch_Mode (RSSI);                           // tipo squelch
    beken.BK4819_Set_Squelch (130,136, 127,127, 127, 127);      // setup Squelch
    beken.BK4819_IRQ_Set ( Squelch_Lost | Squelch_Found );      // definizione interrupt
    beken.BK4819_RX_TurnOn();	                                  // accensione modulo radio
    beken.BK4819_Set_AGC_Gain(AGC_MAN, 26);                     // imposta AGC
    beken.BK4819_Enable_Mic(31);                                // mic Gain max = 31

  //--------------------------------------------------------- Aggiunta dei task allo scheduler
  runner.addTask(keyb);
  runner.addTask(serc);
  runner.addTask(irq);
  runner.addTask(rssi);

  //--------------------------------------------------------- Avvio dei task
  keyb.enable();
  serc.enable();
  irq.enable();
  //rssi.enable();

  attachInterrupt(digitalPinToInterrupt(PIN_S1), leggiEncoder, CHANGE); // Interruzione su cambiamento su pin A
}


//=============================================================================================
//
//=============================================================================================
void mute_audio ( bool stato )
{
  if (stato)
  {
    digitalWrite(PIN_MUTE, HIGH);                               // Mute Audio attivo
    digitalWrite(PIN_LED_RX, LOW);                              // LED verde spento
  }
  else
  {
    digitalWrite(PIN_MUTE, LOW);                                // Mute Audio OFF
    digitalWrite(PIN_LED_RX, HIGH);                             // LED Verde acceso
  }
  mute = stato;
}


//=============================================================================================
//
//=============================================================================================
void loop() 
{
  runner.execute();
}

//=============================================================================================
//
//=============================================================================================
void interrupt ( void )
{
  if ( digitalRead(PIN_IRQ_BEKEN))
  {
    const uint16_t irqtype = beken.BK4819_Check_Irq_type();
    if(!monitor)
    {
      if (irqtype & Squelch_Lost) mute_audio(false);
      if (irqtype & Squelch_Found) mute_audio(true);
    }  
  }
}

//=============================================================================================
//
//=============================================================================================
void keyboard ( void )
{
  //--------------------------------------------------------- Lettura PTT
  int ValoreAn = analogRead(PIN_PTT);                     // Legge il valore dalla porta A0
   
  if (ValoreAn < 10 && !statoPrecedente) 
  {
                                                          // Attiva la trasmissione 
      digitalWrite(PIN_LED_TX, HIGH);                     // LED Rosso acceso
      Txon = true;
      statoPrecedente = true;

      beken.BK4819_Prepare_Transmit();
  }
  else if (ValoreAn > 150 && statoPrecedente) 
  {
                                                          // Disattiva la trasmissione e attiva la ricezione
      digitalWrite(PIN_LED_TX, LOW);                      // LED Rosso spento
      Txon = false;
      statoPrecedente = false;

      // Attiva la ricezione
      beken.BK4819_RX_TurnOn();
  }

  //--------------------------------------------------------- Lettura pulsante Monitor
  bool currentButtonState = digitalRead(PIN_PB_MONITOR);

  if (currentButtonState == LOW  && lastButtonState == HIGH )
  {
    monitor = mute;
    mute_audio(!mute);
  }

  //--------------------------------------------------------- Aggiorna lo stato precedente del pulsante
  lastButtonState = currentButtonState;
}


//=============================================================================================
//
//=============================================================================================
void sercomm( void )
{
  radio.processCIVCommand();

  switch( radio.isChanged())
  {
    case COMMAND_SET_FREQUENCY:
      {
        uint32_t frequenza = radio.getFrequency()/100;

        digitalWrite(PIN_MUTE, HIGH); 
        beken.BK4819_Set_Frequency(frequenza);
        digitalWrite(PIN_MUTE, mute); 
      }
      break;

    case COMMAND_SET_SQUELCH:
      {
        uint8_t value = radio.getSquelch();
        beken.BK4819_Set_Squelch(value, value+4, 0,0,0,0);
        // beken.BK4819_RX_TurnOn();
      }
      break;  

    case COMMAND_SET_MODE:
      {
        // MODE_AM = 0x00   # Codice per AM
        // MODE_FM = 0x01   # Codice per FM
        // MODE_SSB = 0x02  # Codice per SSB

        uint8_t mode = radio.getMode();
        switch(mode)
        {
          case 0:   // # Commuta in AM
            beken.BK4819_Set_AF(AF_AM);
            break;

          case 1:   // # Commuta in FM
            beken.BK4819_Set_AF(AF_FM);
            break;

          case 2:   // # Commuta in DSB
            beken.BK4819_Set_AF(AF_DSB);
            break;  
        }
      }
      break;  
  }
}


//=============================================================================================
//
//=============================================================================================
void smeter( void )
{
  int16_t rssi = beken.BK4819_Get_RSSI();


}


//=============================================================================================
//
//=============================================================================================
void leggiEncoder()
{
    static int ultimoValoreA = LOW; // Memorizza l'ultimo stato di PIN_S1 (fase A)
    int valoreA = digitalRead(PIN_S1); // Legge il valore attuale di PIN_S1
    int valoreB = digitalRead(PIN_S2); // Legge il valore attuale di PIN_S2

    // Verifica una transizione da LOW a HIGH o da HIGH a LOW su PIN_S1
    if (valoreA != ultimoValoreA)
    {
        // Solo quando PIN_S1 cambia stato si determina la direzione
        if (valoreA == HIGH)
        {
            // Transizione rilevata: incrementa o decrementa in base a PIN_S2
            if (valoreB == LOW)
            {
                // Rotazione in senso orario
                frequenza += step;
                direzione = false;
            }
            else
            {
                // Rotazione in senso antiorario
                frequenza -= step;
                direzione = true;
            }

            Serial.print("frequenza = ");
            Serial.println((uint32_t) frequenza);
        }
    }

    // Aggiorna lo stato precedente di PIN_S1
    ultimoValoreA = valoreA;
}



