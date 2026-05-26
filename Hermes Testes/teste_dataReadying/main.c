#include <stdio.h>
#include "dataReadying.h"
#include "telemetryType.h"

int main() {
    telemetryFull t1;
        t1.hora = 9;
        t1.minuto = 57;
        t1.segundo = 14;
        t1.dia = 18;
        t1.mes = 5;
        t1.ano = 26;

        t1.temperatura_bateria = 21.5763f;
        t1.temperatura_externa = -19.3462f;
        t1.deploy_antena = 15;

        t1.tensao_13v8 = 12.5342f;
        t1.tensao_5v     =  4.987f;
        t1.corrente_13v8 =  1.320f;
        t1.corrente_5v   =  0.874f;

        // Paineis Solares
        t1.tensao_painel_solar_1 = 5.120f;
        t1.tensao_painel_solar_2 = 4.980f;
        t1.tensao_painel_solar_3 = 5.054f;
        t1.tensao_painel_solar_4 = 4.763f;

        // Status e Falhas dos Carregadores
        t1.status_carregador_1    = 1;
        t1.status_carregador_2    = 0;
        t1.falha_carregador_1     = 0;
        t1.falha_carregador_2     = 1;

        // ADC
        t1.adc_tensao_sistema_1   = 210;
        t1.adc_tensao_sistema_2   = 208;
        t1.adc_tensao_ntc_1       =  97;
        t1.adc_tensao_ntc_2       = 102;
        t1.adc_corrente_carga_1   = 134;
        t1.adc_corrente_carga_2   = 129;
        t1.adc_corrente_sistema_1 =  88;
        t1.adc_corrente_sistema_2 =  91;

        // Baterias
        t1.tensao_bateria_1 = 1260;
        t1.tensao_bateria_2 = 1245;
        t1.carga_bateria_1  =   87;
        t1.carga_bateria_2  =   74;
        t1.taxa_carga_descarga_bateria_1 =  150;
        t1.taxa_carga_descarga_bateria_2 =  -320;

    // Teste 1: colocar dados no buffer
    uint8_t buffer[51];
    int tamanho = readyData(&t1, buffer);
    printf("O tamanho dos dados eh %d \n\r", tamanho);
    for (int i = 0; i < tamanho; i++) {
        printf("%d \n\r", buffer[i]);
    }
    
    // Teste 2: ler dados do buffer
    telemetryFull t2;
    int tamanhoFinal = parseData(&t2, buffer);
    printf("O tamanho dos dados parseados eh %d \n\r", tamanhoFinal);
    printTelemetry(&t2); 

    return 0;
}