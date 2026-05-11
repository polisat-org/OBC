/**

  * FreeRTOS Queue Teste 1

  * Envianndo buffer para o serial usando QUEUE

  * Data: 10/05
  * Autor: Misael
  * POLISAT


*/
// Usando apenas 1 por demo purposes

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else 
  static const BaseType_t app_cpu = 1;
#endif

// Configurações

static const uint8_t msg_queue_len = 5;

// Globals 

static QueueHandle_t msg_queue; 

//************************************ Tarefas **********************************

//TASK: Esperar pelo item do queue e printá-lo
void printMessages(void *parameters) {
  int item;

  // LOOP Contínuo

  while(1) {
    // Verifica se aquela mensagem no queue (não bloqueia )
      if (xQueueReceive(msg_queue, (void *)&item, 0) == pdTRUE) {
        Serial.println(item);
      }

      // Esperando depois de tentar novamente
      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  
  Serial.begin(115200);

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("----FreeRTOS Queue Demo---");

  // Criacao do QUEUE
  msg_queue = xQueueCreate(msg_queue_len, sizeof(int));

  // definições da taks, atributos e prioridade
  xTaskCreatePinnedToCore(printMessages, "Print Messages", 1024, NULL, 1, NULL, app_cpu);
}

void loop() {

  static int num = 0;

  //Tentativa de adição do item para o Queue for 10 ms, falha somente se o queue estiver cheio

  if (xQueueSend(msg_queue, (void *)&num, 10) != pdTRUE) {
    Serial.println("Queue esta cheio");
  }

  num++;

  //Esperando antes de tentar novamente

  vTaskDelay(1000 / portTICK_PERIOD_MS);

}
