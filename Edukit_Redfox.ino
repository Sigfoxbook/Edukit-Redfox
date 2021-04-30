//************************************************
// Programa default da placa EDUKit Versão 1.0   *
// Se comunica com Monitor Serial para Debug     *
// As práticas 1 a 4 utilizam esse programa      *
// Enviando as variáveis Contador e Temperatura  *
// Toda vez que o botão SW1 da placa EDUKit for  * 
// pressionada o Contador é incrementado de 1  e *
// a Temperatura é incrementada de 0,33          *
//************************************************
#define RESET 4     // Reset no HT32SX
#define TX 2        // Serial TX 
#define RX 3        // Serial RX
#define LED 13      // Led no Arduino Nano
#define SW1 5       // Botão na Placa EDUKIt

#include <SoftwareSerial.h>       // Biblioteca de Serial por Software
SoftwareSerial serial_HT(RX, TX); // Cria Serial paca conectar no HT32SX

int Contador;            // Variável Contador
                         // Toda vez que SW1 for acionado adiciona 1 ao 
                         // Contador 
int16_t Temperatura;     // Variável Temperatura unidade 0,01 Graus 
                         // Centigrados Representamos com numero 
                         // inteiro múltiplo com múltiplo de 100
                         // Toda Vez que SW1 for acionado adiciona 
                         // 33 (equivalente a 0,33 graus) a Temperatura

void setup() {             // Setup inicial
digitalWrite(RESET, HIGH); // ativa Reset  
pinMode(RESET, OUTPUT);    // configura o pino digital RESET como saida.
Serial.begin(9600);        // Abre Serial de Hardware em 9600 bps
                           // Conectada no Monitor Serial (PC)
serial_HT.begin(9600);     // Abre Serial de Software em 9600 bps
                           // Conectada no HT32SX
                           
//Escreve no Monitor Serial Menu de opções de comandos
Serial.println("**************************************************");
Serial.println("* Menu de opções de comandos, tecle de 1 a 5     *");
Serial.println("* 1 -  RESET HT32SX por comando AT               *");
Serial.println("* 2 -  RESET HT32SX por hardware                 *");
Serial.println("* 3 -  Configura para operar em RCZ2             *");
Serial.println("* 4 -  Envia dados (UPLink)                      *");
Serial.println("* 5 -  Envia dados  e aguarda  (Downlink)        *");
Serial.println("*                                                *");
Serial.println("* OBS nas opções 4 e 5 aguardar 35 segundos ou   *");
Serial.println("* até o LED Apagar para envio de novos comandos  *");
Serial.println("**************************************************");
reset_HT();                // Gera o Reset de hardware no HT32SX
delay(5000);               //delay 5 segundos
Serial.println("Vai enviar Configuração AT+CFGRCZ=2"); 
serial_HT.print("AT+CFGRCZ=2;"); // Configura HT32SX para Região RCZ2

pinMode(SW1, INPUT_PULLUP);
pinMode(LED, OUTPUT);




} // Fim da Configuração.


void loop() {            // Loop principal



char buf[36];            // Buffer usado para conversão de String.
char c;                  // Variável auxiliar
// Inicializa variáveis

Contador = 0;
Temperatura = 1000;

    while(true){
    if (serial_HT.available()) {
      c = serial_HT.read(); 
      Serial.print(c);
    } // serial HT
    if (Serial.available()) {
      c = Serial.read();

      switch (c) {
        case '1':
           Serial.println("Vai Executar AT+RESET;");
           serial_HT.print("AT+RESET;"); 
           break;

        case '2':
           Serial.println("Vai Executar RESET de Hardware");
           reset_HT();
           break;

        case '3':
           Serial.println("Vai enviar Configuração AT+CFGRCZ=2");
           serial_HT.print("AT+CFGRCZ=2;");
           break;

        case '4':
           Serial.println("Vai enviar Mensagem SIGFOX");
           sprintf(buf, "AT+SEND=0:%02x%04x;", Contador, Temperatura);
           Serial.println (buf);
           serial_HT.print (buf);
           delay_msg();
           break;

        case '5':
           Serial.println("Vai enviar Mensagem SIGFOX e esperar Downlink");
           sprintf(buf, "AT+SEND=1:%02x%04x;", Contador, Temperatura);
           Serial.println (buf);
           serial_HT.print (buf);
           delay_msg();
           break;
        
      } // Fim do switch
     }// serial PC

int sensorVal = digitalRead(5);
  //print out the value of the pushbutton
  //Serial.println(sensorVal);
  if (sensorVal == HIGH) {  // Botão não pressionado
    digitalWrite(LED, LOW);
  } else {                  // Botão pressionado
    digitalWrite(LED, HIGH);
    Contador++;
    Temperatura += 33;   // Soma 0,33 na Temperatura Final
    Serial.println("Botão Pressionado, vai enviar Mensagem SIGFOX");
    sprintf(buf, "AT+SEND=0:%02x%04x;", Contador, Temperatura);
    Serial.println (buf);
    serial_HT.print (buf);  
    delay_msg();
    while (!digitalRead(SW1))
     {
      delay(10);
     } // fim do while botão   
     }
      
    }// fim do while()   
}// fim do Loop()

//Função de RESET do HT32SX por Hardware
void reset_HT() {          
  digitalWrite(RESET, HIGH); // ativa Reset
  delay(1000);               // espera por um segundo
  digitalWrite(RESET, LOW);  // libera Reset
  delay(100);               // espera 100 milisegundos       
}                           // fim da função reset_HT

// Função de Espera para envio de novo comando
// Monitora Serial TX do HT32SX e envia dados recebidos pela 
// Serial do Monitor Serial.


int Char2Int( char c ){
 int x;
 if (c >= '0' && c <='9' ) {
 x = c - '0';
 }// fim do if
 else if (c >= 'a' && c <='f' ){
   x = (c - 'a') + 10;
  } // fim do else if
  else return(-1);
  return(x);
      
} // Fim da Char2Int

void delay_msg() // Fica esperando Mensagens finaliza em 45 Segundos.
     { 
      int  Temp_H;
      int  Temp_L;
      int  Cont; 
      char c;
      int x;
      int xPos;
      char buf_aux[60];        // Buffer usado para conversão de String.
      
       //String resposta[60];  
       digitalWrite(LED, HIGH);
       Serial.println("Aguarde 45 segundos ou até o led apagar...");
       uint16_t i;
       
       for ( i =0; i<45000; i++)
        {
         if (serial_HT.available()) 
         {
          String resposta = serial_HT.readString(); 
          Serial.print(resposta);
          if(resposta.indexOf("Customer resp:") > 0) { // Downlink
            // exemplo de resposta:  
            // Customer resp: {0x64,0xf,0xa0,0x78,0x90,0xab,0xcd,0xef}
            xPos = resposta.indexOf('x');          // Parse Primeiro x
            c = resposta[xPos+1];
            Cont = Char2Int(c);
            c = resposta[xPos+2];
            if( c != ',' ){
              x = Char2Int(c);        
              Cont *=16;
              Cont +=x;
             }

            xPos = resposta.indexOf('x', xPos+1);  // Parse Segundo x
            c = resposta[xPos+1];
            Temp_H = Char2Int(c);
            c = resposta[xPos+2];
            if( c != ',' ){
              x = Char2Int(c);
              Temp_H *=16;
              Temp_H +=x;
             }

           xPos = resposta.indexOf('x', xPos+1); // Parse Terceiro x
            c = resposta[xPos+1];
            Temp_L = Char2Int(c);
            c = resposta[xPos+2];
            if( c != ',' ){
              x = Char2Int(c);
              Temp_L *=16;
              Temp_L +=x;
             }

             Contador = Cont;
             Temperatura = Temp_H;
             Temperatura *=256;
             Temperatura += Temp_L;

             sprintf(buf_aux, "Valor Recebido: Contador=0x%02x Temperatura=0x%04x;", Contador, Temperatura); 
             Serial.println (buf_aux);
             
            }// fim if resposta

        
          if(resposta.indexOf("AT_cmd_Waiting...") > 0) {   
            i = 50000; // Modulo finalizou operação pode receber novo comando.
          }
         } // serial HT
         delay(1);
       } // fim do for 45000
       digitalWrite(LED, LOW);     // Apaga o Led, 
       Serial.println("Ponto para receber novo comando");
              
     }// fim função delay_msg
 
  
