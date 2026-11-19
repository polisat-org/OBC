// Alguns testes para o I2C do MUX
// Eh so colocar esse codigo na main e conectar a PCB do EPS pelo I2C
// Pode tentar testar com modulo do INA no mesmo canal para conferir
// Abre o PuTTy (baud 115200) tambem para ver os resultados; lembra de colocar __io_putchar para acessar
// Pode colocar na main.c se nao quiser criar outro arquivo, em USER CODE 0

// Includes
#include <stdio.h>

// Defines
#define MUX_ADDR (0x70)
#define INA_ADDR (0x41)
#define MP_ADDR  (0x4B)

// Funcoes

// Scan do I2C, testa todos os enderecos nesse canal I2C
// Retorna quantos enderecos responderam
uint8_t scanI2C(I2C_HandleTypeDef *hi2c) {
    uint8_t qtde = 0;
    printf("Scan I2C...\r\n");
    for (uint8_t a = 1; a < 128; a++) {
        if (HAL_I2C_IsDeviceReady(hi2c, a << 1, 2, 20) == HAL_OK) {
            printf("Found 0x%02X\r\n", a);
            qtde ++;
            HAL_Delay(100);
        }
        else {
            printf("Nothing in 0x%02X\r\n", a);
            HAL_Delay(10);
        }
    }
    return qtde;
}

// Manda byte para ativar canal 
HAL_StatusTypeDef muxWrite(I2C_HandleTypeDef *hi2c, uint8_t data) {
    HAL_StatusTypeDef r = HAL_I2C_Master_Transmit(hi2c, MUX_ADDR << 1, &data, 1, 1000);
    if (r != HAL_OK)
        printf("Failed to write 0x%02X on MUX (err=0x%lX)\r\n",
               data, (unsigned long)HAL_I2C_GetError(hi2c));
    return r;
}

HAL_StatusTypeDef muxRead(I2C_HandleTypeDef *hi2c, uint8_t *out) {
    HAL_StatusTypeDef r = HAL_I2C_Master_Receive(hi2c, MUX_ADDR << 1, out, 1, 1000);
    if (r != HAL_OK)
        printf("Failed to read control byte on MUX (err=0x%lX)\r\n",
               (unsigned long)HAL_I2C_GetError(hi2c));
    return r;
}

HAL_StatusTypeDef selectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel) {
    if (channel > 7) {
        printf("Invalid channel %d\r\n", channel);
        return HAL_ERROR;
    }
    HAL_StatusTypeDef r = muxWrite(hi2c, (uint8_t)(1u << channel));
    if (r == HAL_OK) printf("Channel %d selected on MUX.\r\n", channel);
    return r;
}

// Scan de enderecos para cada canal do MUX
// OBS: byte de escolha do canal n eh (1 << n)
void scanMUX(I2C_HandleTypeDef *hi2c) {
    uint8_t data = 0;
    for (uint8_t channel = 0; channel < 8; channel++) {
        uint8_t qtde = 0;
        data = (1 << channel);
        if(HAL_I2C_Master_Transmit(hi2c, MUX_ADDR << 1, &data, 1, 1000) != HAL_OK) {
            printf("Failed to select channel %d (err=0x%lX)\r\n", channel, (unsigned long)HAL_I2C_GetError(hi2c));
        } else {
            printf("Channel %d selected on MUX.\r\n", channel);
            qtde = scanI2C(hi2c);
        }
        printf("End of scan channel %d, %d addresses\r\n", channel, qtde);
        HAL_Delay(100);  
    } 
    data = 0;   // desliga todos os canais
    HAL_I2C_Master_Transmit(hi2c, MUX_ADDR << 1, &data, 1, 1000);
    return;
}

// Em teoria estamos usando endereco 70, mas se quiser testar outros
uint8_t muxAddrTest(I2C_HandleTypeDef *hi2c) {
    for(uint8_t i = 0; i<8; i++) {
        if(HAL_I2C_IsDeviceReady(hi2c, (MUX_ADDR + i) << 1, 5, 100) == HAL_OK)
            return MUX_ADDR + i;
    }
    return 0;
}

// Essa parte vai na main; se trocar canal I2C por algum que nao seja 1, so corrigir aqui
printf("Scan inicial\r\n");
scanI2C(&hi2c1);
printf("Teste de endereco \r\n");
uint8_t addr = muxAddrTest(&hi2c1);
if(addr == 0) printf("Nao achou MUX\r\n");
else printf("MUX no endereco %d", addr);
printf("Scan dos canais MUX\r\n");
scanMUX(&hi2c1);
// Se algum der certo, tenta selecionar o canal com selectChannel(&hi2c1) e comunicar com dispositivo ali


// putchar para poder usar printf; coloca em USER CODE 4 se ja nao tiver colocado
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}