#include <WiFi.h> /* Header para uso das funcionalidades de wi-fi do ESP32 */
#include <PubSubClient.h>  /*  Header para uso da biblioteca PubSubClient */
 
#define TOPICO_SUBSCRIBE "ESP32_SUBSCRIBE"   
/* Tópico MQTT para envio de informações do ESP32 para broker MQTT */
#define TOPICO_PUBLISH   "ESP32_PUBLISH"  
#define TOPICO_TEMP_PUBLISH   "ESP32_TEMP"  
#define TOPICO_HUM_PUBLISH   "ESP32_HUMIDADE"  
#define TOPICO_CO2_PUBLISH   "ESP32_CO2"  

#define ID_MQTT  "Cliente_MQTT"     
/*  Variáveis e constantes globais */
/* SSID / nome da rede WI-FI que deseja se conectar */
const char* SSID = "Meuparaiso2"; 
/*  Senha da rede WI-FI que deseja se conectar */
const char* PASSWORD = "c60812520"; 
  
/* URL do broker MQTT que deseja utilizar */
const char* BROKER_MQTT = "broker.hivemq.com"; 
/* Porta do Broker MQTT */
int BROKER_PORT = 1883;
 
 
/* Variáveis e objetos globais */
WiFiClient espClient;
PubSubClient MQTT(espClient);
  
//Prototypes
void init_serial(void);
void init_wifi(void);
void init_mqtt(void);
void reconnect_wifi(void); 
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void verifica_conexoes_wifi_mqtt(void); 

//Sensores
#include <DHT11.h>
#include <MQUnifiedsensor.h>
DHT11 dht11(4);
#define placa "ESP-32"
#define Voltage_Resolution 3.3
#define pin 13
#define type "MQ-135"
#define ADC_Bit_Resolution 12
#define RatioMQ135CleanAir 3.6

double CO2 = 0;
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

/*

 *  Implementações das funções
 */
void setup() {

  MQ135.setRegressionMethod(1); // _PPM = a*ratio^b
  MQ135.setA(110.47);
  MQ135.setB(-2.862);

  // Configure the ecuation values to get CO2 concentration
  MQ135.init();

  // Start the serial port
  Serial.begin(9600);

  // Calibrate the sensor
  float calcR0 = 0;
  for (int i = 1; i <= 5; i++) {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0 / 5);
  Serial.println(" done!");

  // Check for calibration errors
  if (isinf(calcR0)) {
    Serial.println("Warning: Connection issue found, R0 is infinite (Open circuit detected). Please check your wiring and supply.");
    while (1) {}
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin with short circuit to ground). Please check your wiring and supply.");
    while (1) {}
  }
  init_wifi();
  init_mqtt();
  
}
  
/* Função: inicializa comunicação serial com baudrate 115200 (para fins de monitorar no terminal serial 
*          o que está acontecendo.
* Parâmetros: nenhum
* Retorno: nenhum
*/
void init_serial() 
{
    Serial.begin(115200);
}
 
/* Função: inicializa e conecta-se na rede WI-FI desejada
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void init_wifi(void) 
{
    delay(10);
    reconnect_wifi();
}
  
/* Função: inicializa parâmetros de conexão MQTT(endereço do  
 *         broker, porta e seta função de callback)
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void init_mqtt(void) 
{
    /* informa a qual broker e porta deve ser conectado */
    MQTT.setServer(BROKER_MQTT, BROKER_PORT); 
    /* atribui função de callback (função chamada quando qualquer informação do 
    tópico subescrito chega) */
    MQTT.setCallback(mqtt_callback);            
}
  
/* Função: função de callback 
 *          esta função é chamada toda vez que uma informação de 
 *          um dos tópicos subescritos chega)
 * Parâmetros: nenhum
 * Retorno: nenhum
 * */
void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{
    String msg;
 
    //obtem a string do payload recebido
    for(int i = 0; i < length; i++) 
    {
       char c = (char)payload[i];
       msg += c;
    }
    Serial.print("[MQTT] Mensagem recebida: ");
    Serial.println(msg);     
}
  
/* Função: reconecta-se ao broker MQTT (caso ainda não esteja conectado ou em caso de a conexão cair)
 *          em caso de sucesso na conexão ou reconexão, o subscribe dos tópicos é refeito.
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void reconnect_mqtt(void) 
{
    while (!MQTT.connected()) 
    {
        Serial.print("* Tentando se conectar ao Broker MQTT: ");
        Serial.println(BROKER_MQTT);
        if (MQTT.connect(ID_MQTT)) 
        {
            Serial.println("Conectado com sucesso ao broker MQTT!");
            MQTT.subscribe(TOPICO_SUBSCRIBE); 
        } 
        else
        {
            Serial.println("Falha ao reconectar no broker.");
            Serial.println("Havera nova tentatica de conexao em 2s");
            delay(2000);
        }
    }
}
  
/* Função: reconecta-se ao WiFi
 * Parâmetros: nenhum
 * Retorno: nenhum
*/
void reconnect_wifi() 
{
    /* se já está conectado a rede WI-FI, nada é feito. 
       Caso contrário, são efetuadas tentativas de conexão */
    if (WiFi.status() == WL_CONNECTED)
        return;
         
    WiFi.begin(SSID, PASSWORD);
     
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(100);
        Serial.print(".");
    }
   
    Serial.println();
    Serial.print("Conectado com sucesso na rede ");
    Serial.print(SSID);
    Serial.println("IP obtido: ");
    Serial.println(WiFi.localIP());
}
 
/* Função: verifica o estado das conexões WiFI e ao broker MQTT. 
 *         Em caso de desconexão (qualquer uma das duas), a conexão
 *         é refeita.
 * Parâmetros: nenhum
 * Retorno: nenhum
 */
void verifica_conexoes_wifi_mqtt(void)
{
    /* se não há conexão com o WiFI, a conexão é refeita */
    reconnect_wifi(); 
    /* se não há conexão com o Broker, a conexão é refeita */
    if (!MQTT.connected()) 
        reconnect_mqtt(); 
} 
 
/* programa principal */
void loop() 
{   
    /* garante funcionamento das conexões WiFi e ao broker MQTT */
    verifica_conexoes_wifi_mqtt(); 
  
    int temperature = dht11.readTemperature();
    digitalWrite(12, HIGH);
    int humidity = dht11.readHumidity();

    MQ135.update();
    CO2 = MQ135.readSensor();
    Serial.print("Raw Sensor Value: ");
    Serial.println(MQ135.readSensor());
    
    // Print the CO2 level to the serial monitor
    MQTT.publish(TOPICO_CO2_PUBLISH, String(CO2).c_str());
    MQTT.publish(TOPICO_TEMP_PUBLISH, String(temperature).c_str());
    MQTT.publish(TOPICO_HUM_PUBLISH, String(humidity).c_str());
      
    /* keep-alive da comunicação com broker MQTT */    
    MQTT.loop();
    /* Agurda 1 segundo para próximo envio */
    delay(1000);   
}
