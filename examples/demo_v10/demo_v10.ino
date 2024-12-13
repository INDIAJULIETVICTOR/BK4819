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
#include <Adafruit_MCP4725.h>
#include <Wire.h>



#define PIN_PTT A0
#define PIN_SQL A1
//#define PIN_RFGAIN A2

#define PIN_IRQ_BEKEN 2

//#define PIN_LED_TX 5
//#define PIN_LED_RX 6

#define PIN_MUTE1 7

#define PIN_PB_ENCODER 8
#define PIN_ENCODER_S1 3
#define PIN_ENCODER_S2 4

#define PIN_PB1 A7
#define PIN_PB2 A6
#define PIN_PB3 A5
#define PIN_PB4 A4

#define DAC_ADDRESS 0x60      // Indirizzo DAC MCP4725
#define EEPROM_ADDRESS 0x50   // Indirizzo I2C per 24C02 (0x50)

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


bool LastPB1 = false;
bool LastPB2 = false;
bool LastPB3 = false;
bool LastPB4 = false;

bool PBLongPress = false;
bool PBShortPress = false;

//--------------------------------------------------------- Definizione del chip beken
BK4819 beken(10, 11, 12, 13);                                   // Passa i pin CS, MOSI, MISO, e SCK

//Adafruit_MCP4725 dac;
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
    Vfo[1].mute_port = PIN_MUTE1;

    Vfo[VfoNum].Frequency = 74025UL * 1000;  
    Vfo[VfoNum].step      = 12500UL;
    Vfo[VfoNum].Sql       = 0;
    Vfo[VfoNum].Mode      = AF_FM;
    Vfo[VfoNum].Gain      = 18;
    Vfo[VfoNum].AGC       = AGC_MAN;

    Serial.begin(115200); 
    // mySerial.begin(9600);                                          // Inizializza SoftwareSerial

    Wire.begin();                                                     // inizializza la comunicazione i2c

    //wdt_enable(WDTO_2S);                                            // Imposta il watchdog per 2 secondi

    beken.BK4819_SCN_select(Vfo[VfoNum].scn_port);

    radio.Initialize(&Vfo[0],&Vfo[1]);

    pinMode(PIN_PTT, INPUT_PULLUP);                                   // porte analogiche
    pinMode(PIN_SQL, INPUT_PULLUP);
    //pinMode(PIN_RFGAIN, INPUT_PULLUP);

    pinMode(PIN_ENCODER_S1, INPUT);
    pinMode(PIN_ENCODER_S2, INPUT);

    pinMode(PIN_IRQ_BEKEN, INPUT_PULLUP);                             // Ingresso Linea IRQ del beken ( GPIO 4 )
    pinMode(PIN_PB_ENCODER, INPUT_PULLUP);                            // Pulsante ENCODER

    pinMode(PIN_PB1, INPUT);
    pinMode(PIN_PB2, INPUT);
    pinMode(PIN_PB3, INPUT);
    pinMode(PIN_PB4, INPUT);            

    pinMode(PIN_MUTE1, OUTPUT);                                       // abilitazione audio MUTE primo modulo
    digitalWrite(PIN_MUTE1, HIGH);                                    // Mute ON
 

    beken.BK4819_Init();                                              // Inizializza il dispositivo BK4819

    beken.BK4819_Set_GPIO_Output(3,true);                             // accende il led operativo

    beken.BK4819_Set_Modulation(MODE_FM, true);
    beken.BK4819_Set_Frequency ( Vfo[VfoNum].Frequency,true );        // imposta frequenza
    beken.BK4819_Set_Filter_Bandwidth(BK4819_FILTER_BW_6k,true);	    // imposta BW e filtri audio
    beken.BK4819_Squelch_Mode (RSSI,true);                            // tipo squelch
     beken.BK4819_IRQ_Set ( Squelch_Lost | Squelch_Found,true );       // definizione interrupt
    beken.BK4819_RX_TurnOn(true);	                                    // accensione modulo radio
    beken.BK4819_Set_AGC_Gain(Vfo[VfoNum].AGC, Vfo[VfoNum].Gain,true);               // imposta AGC e RFGain
    beken.BK4819_Enable_Mic(31,true);                                 // mic Gain max = 31
    beken.BK4819_Set_TxDeviation ( 5 ,true);                          // deviazione TX 0 = disabilitata 1 = Min 10 = max
    beken.BK4819_Set_Power_TX(0,true);                                // tx disabilitata        

  //--------------------------------------------------------- configurazione interrupt
  //attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_S1), check_encoder, CHANGE);       // Interruzione su cambiamento su pin A
  //attachInterrupt(digitalPinToInterrupt(PIN_IRQ_BEKEN), interrupt_beken, RISING);   // Interruzione su cambiamento irq del beken
 
  // if (!dac.begin(DAC_ADDRESS)) 
  // {  
  //   radio.Debug_Print("%s\r\n","Errore nella connessione al DAC!");
  // }
  // else dac.setVoltage(3192, true);                                        // Il secondo parametro è false, quindi il DAC non entrerà in modalità "permanente"

  radio.send_frequency(COMMAND_GET_FREQUENCY, Vfo[VfoNum].Frequency, CIV_ADDRESS_RADIO, CIV_ADDRESS_COMPUTER);
  //update_rfgain(true);
  //update_rfgain(true);
  radio.send_command(COMMAND_GET_RFGAIN, Vfo[VfoNum].Gain, CIV_ADDRESS_RADIO, CIV_ADDRESS_COMPUTER);
  radio.send_command(COMMAND_GET_RFGAIN, Vfo[VfoNum].Gain, CIV_ADDRESS_RADIO, CIV_ADDRESS_COMPUTER);
  update_squelch(true);
  update_squelch(true);

  //radio.Debug_Print("%s\r\n","Scansione I2C in corso...");
  
  // for (byte i = 8; i < 120; i++) 
  // {
  //   Wire.beginTransmission(i);
  //   byte error = Wire.endTransmission();
  //   if (error == 0) radio.Debug_Print("Dispositivo trovato all'indirizzo %x\r\n",i);
  // }

  //eepromWrite(0x10, 42);  // Scrivi il valore 42 all'indirizzo 0x10
  //delay(100);

  // Esempio di lettura dalla EEPROM
  //byte data = eepromRead(0x10);  // Leggi il valore dall'indirizzo 0x10
  //radio.Debug_Print("Eeprom read = %d\r\n",data);
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
      //if (counter % 300 == 0) checkirq(); 
    }
    wdt_reset();
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
      //digitalWrite(PIN_LED_TX, HIGH);                   // LED Rosso acceso
      beken.BK4819_Set_GPIO_Output(0,false);              // SPEGNE led Verde
      beken.BK4819_Set_GPIO_Output(1,true);               // accende led Roxxo (Tx)
      Vfo[VfoNum].Flag.bits.tx = 1;
      Txon = true;
      statoPrecedente = true;

      beken.BK4819_Set_Power_TX(Vfo[VfoNum].txp, false);  // uscita a zero dB 0 = tx OFF 1 = min 15 = Max
      beken.BK4819_Prepare_Transmit(false);
  }
  else if (ValoreAn > 140 && statoPrecedente) 
  {
                                                          // Disattiva la trasmissione e attiva la ricezione
      beken.BK4819_Set_GPIO_Output(1,false);              // spegne il LED Tx
      //digitalWrite(PIN_LED_TX, LOW);                    // LED Rosso spento
      Vfo[VfoNum].Flag.bits.tx = 0;
      Txon = false;
      statoPrecedente = false;

      // Attiva la ricezione
      beken.BK4819_RX_TurnOn(false);
  }

  // //--------------------------------------------------------- Lettura pulsante Monitor
  bool currentPB4 = digitalRead(PIN_PB4);

  if (currentPB4 == LOW && !LastPB4)
  {
    mute_audio(monitor);
    monitor = !monitor;
    Vfo[VfoNum].Flag.bits.monitor = monitor;
    LastPB4 = true;
  }
  //--------------------------------------------------------- Aggiorna lo stato precedente del pulsante
   if (currentPB4 == HIGH) LastPB4 = false;

  Encoder_button();

  //--------------------------------------------------------- aggiornamento squelch analogico
  update_squelch(false);
  //--------------------------------------------------------- aggiornamento rfgain analogico
  //update_rfgain(false);


}




//=============================================================================================
//
//=============================================================================================
void mute_audio ( bool stato )
{
  if (stato)
  {
    digitalWrite(Vfo[VfoNum].mute_port, HIGH);              // Mute Audio attivo
    //digitalWrite(PIN_LED_RX, LOW);                        // LED verde spento
    beken.BK4819_Set_GPIO_Output(0,false);                  // accende il led operativo
    Vfo[VfoNum].Flag.bits.rx = 0;
  }
  else
  {
    digitalWrite(Vfo[VfoNum].mute_port, LOW);               // Mute Audio OFF
    //digitalWrite(PIN_LED_RX, HIGH);                       // LED Verde acceso
    beken.BK4819_Set_GPIO_Output(0,true);                   // accende il led operativo
    Vfo[VfoNum].Flag.bits.rx = 1;
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

int lastButtonState = HIGH;         // Stato precedente del pulsante
bool onetime = false;
unsigned long lastTime = 0;

void Encoder_button()
{
  int stato = digitalRead(PIN_PB_ENCODER);

  if (stato != lastButtonState)
  {
    if (stato == LOW)  lastTime = millis(); 
    else PBShortPress = !PBShortPress;
    
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

bool s1_low_detected = false;
bool s2_low_detected = false;
bool s1_first = false;
bool s2_first = false;

void check_encoder() 
{
  int S1 = digitalRead(PIN_ENCODER_S1);  // Legge il valore attuale di PIN_S1
  int S2 = digitalRead(PIN_ENCODER_S2);  // Legge il valore attuale di PIN_S2

  // Resetta i flag se entrambi i pin sono HIGH
  if (S1 == HIGH && S2 == HIGH) 
  {
    s1_low_detected = false;
    s2_low_detected = false;
    s1_first = false;
    s2_first = false;
    return;
  } 

  // Verifica se S1 è passato da HIGH a LOW
  if (S1 == LOW && !s1_low_detected && S2 == HIGH && !s2_first) 
  {
    s1_low_detected = true;
    s1_first = true;
  }

  // Verifica se S2 è passato da HIGH a LOW
  if (S2 == LOW && !s2_low_detected && S1 == HIGH && !s1_first) 
  {
    s2_low_detected = true;
    s2_first = true;
  }


  if (s1_first && S2==LOW && !s2_low_detected )
  {
    // Rotazione in senso orario
    if (PBShortPress) Vfo[VfoNum].Frequency = check_frequency( Vfo[VfoNum].Frequency, +1000000); 
    else              Vfo[VfoNum].Frequency = check_frequency( Vfo[VfoNum].Frequency, Vfo[VfoNum].step);
    s2_low_detected = true;
    digitalWrite(Vfo[VfoNum].mute_port, HIGH);
    beken.BK4819_Set_Frequency(Vfo[VfoNum].Frequency, true);
    radio.send_frequency( COMMAND_GET_FREQUENCY, Vfo[VfoNum].Frequency, CIV_ADDRESS_RADIO, CIV_ADDRESS_COMPUTER);
    digitalWrite(Vfo[VfoNum].mute_port, mute);
    return;
  }

  if (s2_first && S1==LOW && !s1_low_detected )
  {
    // Rotazione in senso antiorario
    if (PBShortPress) Vfo[VfoNum].Frequency = check_frequency( Vfo[VfoNum].Frequency, -1000000); 
    else              Vfo[VfoNum].Frequency = check_frequency( Vfo[VfoNum].Frequency, -Vfo[VfoNum].step);
    s1_low_detected = true;
    digitalWrite(Vfo[VfoNum].mute_port, HIGH);
    beken.BK4819_Set_Frequency(Vfo[VfoNum].Frequency, true);
    radio.send_frequency( COMMAND_GET_FREQUENCY, Vfo[VfoNum].Frequency, CIV_ADDRESS_RADIO, CIV_ADDRESS_COMPUTER);
    digitalWrite(Vfo[VfoNum].mute_port, mute);
    return;
  }
}

//=============================================================================================
//
//=============================================================================================
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
uint8_t map_with_clamp(int value, int in_min, int in_max, int out_min, int out_max)
{
    // Mappa il valore alla scala desiderata
    long mapped_value = (long)(value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    // Assicura che il valore sia sempre compreso tra out_min e out_max
    if (mapped_value < out_min) mapped_value = out_min;
    if (mapped_value > out_max) mapped_value = out_max;

    return (uint8_t)mapped_value;
}




uint8_t previous_squelch_level = 0;       // Memorizza il valore precedente
const uint8_t hysteresis_threshold1 = 5; // Soglia di isteresi
const uint8_t filter_weight2 = 2;         // Peso del filtro per la media mobile (maggiore = più lento, minore = più reattivo)

void update_squelch(bool force) 
{
    // Leggi il valore attuale dello squelch dall'ingresso analogico A1
    int analog_value = read_analog_stable(PIN_SQL);

    // Usa la funzione di mappatura migliorata
    uint8_t current_squelch_level = map_with_clamp(analog_value, 0, 1023, 0, 255);

    // Calcola la differenza assoluta tra il valore attuale e quello precedente
    uint8_t difference = abs(current_squelch_level - previous_squelch_level);

    // Se la differenza supera la soglia di isteresi o se il parametro 'force' è vero, aggiorna il valore
    if (difference >= hysteresis_threshold1 || force) 
    {
        // Aggiorna il livello di squelch nel VFO corrente
        Vfo[VfoNum].Sql = current_squelch_level;

        // Imposta il livello di squelch nel chip BK4819
        beken.BK4819_Set_Squelch(Vfo[VfoNum].Sql, Vfo[VfoNum].Sql + 4, 0, 0, 0, 0, false);

        // Invia il comando di aggiornamento dello squelch
        radio.send_command(COMMAND_GET_SQUELCH, Vfo[VfoNum].Sql, CIV_ADDRESS_RADIO, CIV_ADDRESS_COMPUTER);

        // Aggiorna il valore precedente
        previous_squelch_level = current_squelch_level;
    }
}




//=============================================================================================
//
//=============================================================================================
// uint8_t previous_gain = 0;                // Memorizza il valore precedente
// const uint8_t hysteresis_threshold2 = 1; // Soglia di isteresi
// const uint8_t filter_weight = 4;         // Peso del filtro per la media mobile (maggiore = più lento, minore = più reattivo)

// void update_rfgain(bool force) 
// {
//     // Leggi il valore attuale del RFGain dall'ingresso analogico A2
//     int analog_value = read_analog_stable(PIN_RFGAIN);

//     // Usa la funzione di mappatura migliorata
//     uint8_t current_gain = map_with_clamp(analog_value, 0, 1023, 0, 31);

//     // Calcola la differenza assoluta tra il valore attuale e quello precedente
//     uint8_t difference = abs(current_gain - previous_gain);

//     // Se la differenza supera la soglia di isteresi o se il parametro 'force' è vero, aggiorna il valore
//     if (difference >= hysteresis_threshold2 || force) 
//     {
//         // Aggiorna il livello di guadagno nel VFO corrente
//         Vfo[VfoNum].Gain = current_gain;

//         // Imposta il livello di guadagno nel chip BK4819
//         beken.BK4819_Set_AGC_Gain(AGC_MAN, Vfo[VfoNum].Gain, false);

//         // Invia il comando di aggiornamento del guadagno RF
//         radio.send_command(COMMAND_GET_RFGAIN, Vfo[VfoNum].Gain, CIV_ADDRESS_RADIO, CIV_ADDRESS_COMPUTER);

//         // Aggiorna il valore precedente
//         previous_gain = current_gain;
//     }
// }


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
    beken.BK4819_Set_Modulation(Vfo[VfoNum].Mode, false);
  }

  //--------------------------------------------------------- Controlla se il guadagno è cambiato
  if (changedFlags & FLAG_GAIN_CHANGED)
  {
    //uint16_t value = 400 + (uint16_t)Vfo[VfoNum].Gain * 100;
    //dac.setVoltage(value, false);
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
    
    mute_audio(monitor);
    monitor = !monitor;
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
  if (digitalRead(PIN_IRQ_BEKEN) == HIGH) 
  {
    const uint16_t irqtype = beken.BK4819_Check_Irq_type(true);
    if(!monitor)
    {
      if (irqtype & Squelch_Lost)  mute_audio(false); 
      if (irqtype & Squelch_Found) mute_audio(true); 
    } 
  }
}

//=============================================================================================
//
//=============================================================================================
uint32_t check_frequency ( uint32_t frequency, int32_t increment )
{
  if(((frequency + increment)>=15000000) && ((frequency + increment) < 1300000000)) return (frequency + increment);

  return frequency;
}


// Funzione per scrivere un byte nella EEPROM
void eepromWrite(int addr, byte data) 
{
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(addr >> 8));  // invia i primi 8 bit dell'indirizzo
  Wire.write((int)(addr & 0xFF));  // invia i secondi 8 bit dell'indirizzo
  Wire.write(data);  // invia il dato
  Wire.endTransmission();
  delay(5);  // Ritardo per assicurarsi che la scrittura sia completata
}

// Funzione per leggere un byte dalla EEPROM
byte eepromRead(int addr) 
{
  byte receivedData = 0xFF;  // Variabile per il dato ricevuto

  // Inizializza la comunicazione I2C per scrivere l'indirizzo di memoria
  Wire.beginTransmission(EEPROM_ADDRESS);
  Wire.write((int)(addr >> 8));  // invia i primi 8 bit dell'indirizzo
  Wire.write((int)(addr & 0xFF));  // invia i secondi 8 bit dell'indirizzo
  Wire.endTransmission();  // Termina la trasmissione dell'indirizzo

  // Aggiungi un breve ritardo per consentire alla EEPROM di elaborare
  delay(10);  

  // Ora che abbiamo inviato l'indirizzo, possiamo fare una lettura
  Wire.requestFrom(EEPROM_ADDRESS, 1);  // Richiede un byte di dati
  if (Wire.available()) {
    receivedData = Wire.read();  // Legge il dato dalla EEPROM
  }

  return receivedData;
}






