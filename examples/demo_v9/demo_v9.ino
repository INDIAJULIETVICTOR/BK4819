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
#include <IcomSim.h>
#include <avr/wdt.h>


#define PIN_PTT A0
#define PIN_SQL A1
#define PIN_RFGAIN A2

#define PIN_IRQ_BEKEN 2

#define PIN_LED_TX 5
#define PIN_LED_RX 6

#define PIN_MUTE1 7
#define PIN_MUTE2 A5

#define PIN_PB_ENCODER 8
#define PIN_ENCODER_S1 3
#define PIN_ENCODER_S2 4


// #include <SoftwareSerial.h>

//--------------------------------------------------------- Creazione dell'oggetto Scheduler
// Scheduler runner;

//--------------------------------------------------------- Funzioni dei task
void controlli();
void sercomm();
void interrupt_beken();
void smeter();
void check_control();

//--------------------------------------------------------- Definizione variabili
VfoData_t Vfo[2];

uint8_t VfoNum = 0;

volatile int posizione = 0;  // Posizione dell'encoder
volatile bool direzione;      // Direzione di rotazione

bool mute = true;
bool monitor = false;
bool Txon = false;
bool statoPrecedente = false;
bool toggleState = false;   

bool PBLongPress = false;
bool PBShortPress = false;

//--------------------------------------------------------- Definizione del chip beken
BK4819 beken(10, 11, 12, 13);                                   // Passa i pin CS, MOSI, MISO, e SCK


//--------------------------------------------------------- Definizione librerie Icom CI-V
IcomSim radio(Serial);                                          // usa la seriale di sistema USB

long oldPosition  = -999;

// #define RX_PIN 10                                            // pin usati da softwareserial
// #define TX_PIN 11

// SoftwareSerial mySerial(RX_PIN, TX_PIN);
// IcomSim radio(mySerial);                                     // Usa SoftwareSerial per la comunicazione seriale

//=============================================================================================
//
//=============================================================================================
void setup() 
{
    Vfo[0].scn_port = 10;
    Vfo[1].scn_port = 9;
    Vfo[0].mute_port = PIN_MUTE1;
    Vfo[1].mute_port = PIN_MUTE2;

    Vfo[VfoNum].Frequency = 74025UL * 1000;  
    Vfo[VfoNum].step      = 12500UL;
    Vfo[VfoNum].Sql       = 0;
    Vfo[VfoNum].Mode      = AF_FM;
    Vfo[VfoNum].Gain      = 0;
    Vfo[VfoNum].AGC       = AGC_MAN;

    Serial.begin(115200); 
    // mySerial.begin(9600);                                    // Inizializza SoftwareSerial

    wdt_enable(WDTO_2S);                                        // Imposta il watchdog per 2 secondi

    beken.BK4819_SCN_select(Vfo[VfoNum].scn_port);

    radio.Initialize(&Vfo[0],&Vfo[1]);

    pinMode(PIN_PTT, INPUT_PULLUP);                             // porte analogiche
    pinMode(PIN_SQL, INPUT_PULLUP);
    pinMode(PIN_RFGAIN, INPUT_PULLUP);

    pinMode(PIN_ENCODER_S1, INPUT);
    pinMode(PIN_ENCODER_S2, INPUT);

    pinMode(PIN_IRQ_BEKEN, INPUT_PULLUP);                       // Ingresso Linea IRQ del beken ( GPIO 4 )
    pinMode(PIN_PB_ENCODER, INPUT_PULLUP);                      // Pulsante ENCODER

    pinMode(PIN_MUTE1, OUTPUT);                                 // abilitazione audio MUTE primo modulo
    pinMode(PIN_MUTE2, OUTPUT);                                 // abilitazione audio MUTE secondo modulo
    
    pinMode(PIN_LED_RX, OUTPUT);                                // Led Verde stato RX
    pinMode(PIN_LED_TX, OUTPUT);                                // Led Rosso stato TX

    digitalWrite(PIN_LED_TX, LOW);                              // led TX Spento
    digitalWrite(PIN_LED_RX, LOW);                              // led RX Spento

    digitalWrite(PIN_MUTE1, HIGH);                              // Mute ON
    digitalWrite(PIN_MUTE2, HIGH);                              // Mute ON

    beken.BK4819_Init();                                        // Inizializza il dispositivo BK4819

    beken.BK4819_Set_AF(AF_FM,true);		                              // attiva demodulazione FM
    beken.BK4819_Set_Frequency ( Vfo[VfoNum].Frequency,true );        // imposta frequenza
    beken.BK4819_Set_Filter_Bandwidth(BK4819_FILTER_BW_6k,true);	    // imposta BW e filtri audio
    beken.BK4819_Squelch_Mode (RSSI,true);                            // tipo squelch
    //beken.BK4819_Set_Squelch (130,136, 127,127, 127, 127,true );      // setup Squelch
    beken.BK4819_IRQ_Set ( Squelch_Lost | Squelch_Found,true );       // definizione interrupt
    beken.BK4819_RX_TurnOn(true);	                                    // accensione modulo radio
    //beken.BK4819_Set_AGC_Gain(Vfo[VfoNum].AGC, Vfo[VfoNum].Gain,true);               // imposta AGC e RFGain
    beken.BK4819_Enable_Mic(31,true);                                 // mic Gain max = 31
    beken.BK4819_Set_TxDeviation ( 5 ,true);                          // deviazione TX 0 = disabilitata 1 = Min 10 = max
    beken.BK4819_Set_Power_TX(0,true);                                // tx disabilitata        

  //--------------------------------------------------------- configurazione interrupt
  //attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_S1), check_encoder, CHANGE);       // Interruzione su cambiamento su pin A
  //attachInterrupt(digitalPinToInterrupt(PIN_IRQ_BEKEN), interrupt_beken, RISING);   // Interruzione su cambiamento irq del beken

  radio.send_frequency(Vfo[VfoNum].Frequency, CIV_ADDRESS_RADIO, CIV_ADDRESS_COMPUTER);
  update_rfgain(true);
  update_rfgain(true);
  update_squelch(true);
  update_squelch(true);
}

//=============================================================================================
//
//=============================================================================================
unsigned long counter = 0;
unsigned long prevmillis = 0;

void loop() 
{
  check_encoder();

  interrupt_beken();

  beken.BK4819_processRegisterWriteQueue();

    if(millis()>prevmillis) 
    { 
      prevmillis = millis(); 
      ++counter;

      radio.processSerialQueue();

      // Esegui i controlli utente ogni 50 ms
      if (counter % 50 == 0) controlli(); 

      // trasmetti lo stato della radio ogni 100 mS
      if (counter % 100 == 0) send_status(); 
      
      // trasmetti lo smeter ogni 250 ms 
      if (counter % 250 == 0) smeter(); 

      // controlla la comunicazione seriale ogni 200 ms
      if (counter % 200 == 0) sercomm(); 

      // controlla l'interrupt del beken ogni 300 ms
      if (counter % 300 == 0) checkirq(); 
    }
    wdt_reset();
}

//=============================================================================================
//
//=============================================================================================
void mute_audio ( bool stato )
{
  if (stato)
  {
    digitalWrite(Vfo[VfoNum].mute_port, HIGH);              // Mute Audio attivo
    digitalWrite(PIN_LED_RX, LOW);                          // LED verde spento
  }
  else
  {
    digitalWrite(Vfo[VfoNum].mute_port, LOW);               // Mute Audio OFF
    digitalWrite(PIN_LED_RX, HIGH);                         // LED Verde acceso
  }
  mute = stato;
}

//=============================================================================================
//
//=============================================================================================
unsigned long irqHighStartTime = 0;
bool irqHighDetected = false;

void checkirq(void) 
{
    if (digitalRead(PIN_IRQ_BEKEN) == HIGH) 
    {
        if (!irqHighDetected) 
        {
            // Segnale alto appena rilevato, registra l'inizio del tempo
            irqHighStartTime = millis();
            irqHighDetected = true;
        } 
        else 
        {
            // Controlla se il segnale è stato alto per più di 20 ms
            if (millis() - irqHighStartTime > 20) 
            {
                beken.BK4819_Clear_Interrupt(false);
                irqHighDetected = false; // Resetta il flag dopo aver gestito l'interrupt
            }
        }
    } 
    else 
    {
        // Se il segnale è basso, resetta il flag
        irqHighDetected = false;
    }
}


//=============================================================================================
//
//=============================================================================================
void controlli ( void )
{
  // --------------------------------------------------------- Lettura PTT
  int ValoreAn = analogRead(PIN_PTT);                     // Legge il valore dalla porta A0

  if (ValoreAn < 130 && !statoPrecedente) 
  {
                                                          // Attiva la trasmissione 
      digitalWrite(PIN_LED_TX, HIGH);                     // LED Rosso acceso
      Vfo[VfoNum].Flag.bits.tx = 1;
      Txon = true;
      statoPrecedente = true;

      beken.BK4819_Set_Power_TX(Vfo[VfoNum].txp, false);  // uscita a zero dB 0 = tx OFF 1 = min 15 = Max
      beken.BK4819_Prepare_Transmit(false);
  }
  else if (ValoreAn > 140 && statoPrecedente) 
  {
                                                          // Disattiva la trasmissione e attiva la ricezione
      digitalWrite(PIN_LED_TX, LOW);                      // LED Rosso spento
      Vfo[VfoNum].Flag.bits.tx = 0;
      Txon = false;
      statoPrecedente = false;

      // Attiva la ricezione
      beken.BK4819_RX_TurnOn(false);
  }

  // //--------------------------------------------------------- Lettura pulsante Monitor
  // bool currentButtonState = digitalRead(PIN_PB_ENCODER);

  // if (currentButtonState == LOW  && lastButtonState == HIGH )
  // {
  //   beken.BK4819_Clear_Interrupt();
  //   monitor = mute;
  //   mute_audio(!mute);
  // }

  Encoder_button();

  //--------------------------------------------------------- aggiornamento squelch analogico
  update_squelch(false);
  //--------------------------------------------------------- aggiornamento rfgain analogico
  update_rfgain(false);

  //--------------------------------------------------------- Aggiorna lo stato precedente del pulsante
  //lastButtonState = currentButtonState;
}


//=============================================================================================
//
//=============================================================================================

int lastButtonState = HIGH;         // Stato precedente del pulsante
bool onetime = false;
unsigned long lastTime = 0;

void Encoder_button()
{
  int stato = digitalRead(PIN_PB_ENCODER);

  if (stato != lastButtonState)
  {
    if (stato == LOW)  lastTime = millis(); 
    else PBShortPress = true;
    
    onetime = false;                // azzera flag se c'e' una variazione
  }

  if ( stato == LOW )               // pulsante premuto
  {
    if ((onetime == false) && (millis() - lastTime) > 1500UL)
    {
      onetime = true;
      PBLongPress = true;
    }
  }  

  lastButtonState = stato;
}

//=============================================================================================
//
//=============================================================================================

volatile int ultimoValoreA = HIGH;                  // Memorizza l'ultimo stato di PIN_S1 (fase A)

void check_encoder() 
{
  int valoreA = digitalRead(PIN_ENCODER_S1);  // Legge il valore attuale di PIN_S1
  int valoreB = digitalRead(PIN_ENCODER_S2);  // Legge il valore attuale di PIN_S2

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
        Vfo[VfoNum].Frequency += Vfo[VfoNum].step;
      } 
      else 
      {
        // Rotazione in senso antiorario
        Vfo[VfoNum].Frequency -= Vfo[VfoNum].step;
      }

      beken.BK4819_Set_Frequency(Vfo[VfoNum].Frequency, false);
      radio.send_frequency(Vfo[VfoNum].Frequency, CIV_ADDRESS_RADIO, CIV_ADDRESS_COMPUTER);
    }
    // Aggiorna lo stato precedente di PIN_S1
    ultimoValoreA = valoreA;
  }
}

int read_analog_stable(int pin, int num_samples = 5) 
{
    long total = 0;
    for (int i = 0; i < num_samples; i++) 
    {
        total += analogRead(pin);
        delay(1); // Breve pausa per evitare letture troppo rapide
    }
    return total / num_samples;
}

//=============================================================================================
//
//=============================================================================================
uint8_t previous_squelch_level = 0;  // Memorizza il valore precedente

void update_squelch(bool force) 
{
    // Leggi il valore attuale dello squelch dall'ingresso analogico A1
    int analog_value = read_analog_stable(PIN_SQL);
    // int analog_value = analogRead(PIN_SQL);

    // Mappa il valore da 0-1023 a 0-255
    Vfo[VfoNum].Sql = map(analog_value, 0, 1023, 0, 255);

    // Calcola la variazione percentuale
    float variation = abs(Vfo[VfoNum].Sql - previous_squelch_level) / 255.0 * 100;

    // Se la variazione supera il 1%, aggiorna il valore
    if ((variation > 1.0) || force) 
    {
        beken.BK4819_Set_Squelch(Vfo[VfoNum].Sql, Vfo[VfoNum].Sql + 4, 0, 0, 0, 0, false);
        radio.send_command(COMMAND_GET_SQUELCH, Vfo[VfoNum].Sql, CIV_ADDRESS_RADIO, CIV_ADDRESS_COMPUTER);

        previous_squelch_level = Vfo[VfoNum].Sql;  // Aggiorna il valore precedente
        // radio.Debug_Print("%d\r\n",Vfo[VfoNum].Sql);
    }
}

//=============================================================================================
//
//=============================================================================================
uint8_t previous_gain = 0;  // Memorizza il valore precedente

void update_rfgain(bool force) 
{
    // Leggi il valore attuale del RFGain dall'ingresso analogico A2
    int analog_value = read_analog_stable(PIN_RFGAIN);
    // int analog_value = analogRead(PIN_RFGAIN);

    // Mappa il valore da 0-1023 a 0-31
    Vfo[VfoNum].Gain = map(analog_value, 0, 1023, 0, 31);

    // Calcola la variazione percentuale
    float variation = abs(Vfo[VfoNum].Gain - previous_gain) / 31.0 * 100;

    // Se la variazione supera il 1%, aggiorna il valore
    if ((variation > 1.0)|| force) 
    {
        beken.BK4819_Set_AGC_Gain(AGC_MAN, Vfo[VfoNum].Gain , false);
        radio.send_command(COMMAND_GET_RFGAIN, Vfo[VfoNum].Gain, CIV_ADDRESS_RADIO, CIV_ADDRESS_COMPUTER);

        previous_gain = Vfo[VfoNum].Gain;  // Aggiorna il valore precedente
    }
}

//=============================================================================================
//
//=============================================================================================
void sercomm(void)
{
  radio.processCIVCommand();

  uint16_t changedFlags = radio.isChanged();

  //--------------------------------------------------------- Controlla se la frequenza è cambiata
  if (changedFlags & FLAG_FREQUENCY_CHANGED)
  {
    digitalWrite(Vfo[VfoNum].mute_port, HIGH); 
    beken.BK4819_Set_Frequency(Vfo[VfoNum].Frequency, false);
    digitalWrite(Vfo[VfoNum].mute_port, mute); 
  }

  //--------------------------------------------------------- Controlla se lo squelch è cambiato
  if (changedFlags & FLAG_SQL_CHANGED)
  {
    beken.BK4819_Clear_Interrupt(false);
    beken.BK4819_Set_Squelch(Vfo[VfoNum].Sql, Vfo[VfoNum].Sql + 4, 0, 0, 0, 0, false);
  }

  //--------------------------------------------------------- Controlla se la modalità è cambiata
  if (changedFlags & FLAG_MODE_CHANGED)
  {
    switch (Vfo[VfoNum].Mode)
    {
      case 0:   // Commuta in AM
        beken.BK4819_Set_AF(AF_AM, false);
        break;

      case 1:   // Commuta in FM
        beken.BK4819_Set_AF(AF_FM, false);
        break;

      case 2:   // Commuta in DSB
        beken.BK4819_Set_AF(AF_DSB, false);
        break;
    }
  }

  //--------------------------------------------------------- Controlla se il guadagno è cambiato
  if (changedFlags & FLAG_GAIN_CHANGED)
  {
    beken.BK4819_Set_AGC_Gain(AGC_MAN, Vfo[VfoNum].Gain, false); 
  }

  //--------------------------------------------------------- Controlla se la Bandwith e' cambiata
  if (changedFlags & FLAG_BW_CHANGED)
  {
    beken.BK4819_Set_Filter_Bandwidth(Vfo[VfoNum].bw, false);	    // imposta BW e filtri audio
  }

  //--------------------------------------------------------- Controlla se la potenza e' cambiata
  if (changedFlags & FLAG_TXP_CHANGED)
  {
    if(Txon) beken.BK4819_Set_Power_TX(Vfo[VfoNum].txp, false);    // solo se e' in trasmissione
  }


  //--------------------------------------------------------- Controlla il pulsante monitor
  if (changedFlags & FLAG_MONITOR_CHANGED)
  {
    monitor = mute;
    mute_audio(!mute);
    Vfo[VfoNum].Flag.bits.monitor = monitor;
  }
}

//=============================================================================================
//
//=============================================================================================
void smeter( void )
{
  uint16_t rssi = beken.BK4819_Read_Register(0x67);
  radio.send_rssi(rssi, CIV_ADDRESS_RADIO, CIV_ADDRESS_COMPUTER);
}

//=============================================================================================
//
//=============================================================================================
void send_status( void )
{
  radio.send_status(VfoNum,CIV_ADDRESS_RADIO, CIV_ADDRESS_COMPUTER);
}

//=============================================================================================
//
//=============================================================================================
void interrupt_beken ( void )
{
    const uint16_t irqtype = beken.BK4819_Check_Irq_type(true);
    if(!monitor)
    {
      if (irqtype & Squelch_Lost) {Vfo[VfoNum].Flag.bits.rx = 1;  mute_audio(false); }
      if (irqtype & Squelch_Found){Vfo[VfoNum].Flag.bits.rx = 0;  mute_audio(true);  }
    }  
}








