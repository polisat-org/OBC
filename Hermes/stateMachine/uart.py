import serial
import struct
import time

def read_telemetry(ser):
    TELEMETRY_FORMAT = (
    '<'         # little-endianess 
    'BBBBBB'    # hora, minuto, segundo, dia, mes, ano
    'ffff'        # temperatura_bateria, temperatura_estrutura, umidade, pressao
    'B'         # deploy_antena
    'ffff'      # tensao_13v8, tensao_5v, corrente_13v8, corrente_5v
    'ffff'      # tensao_painel_solar_1..4
    'BBBB'      # status_carregador_1..2, falha_carregador_1..2
    'BBBBBBBB'  # adc_tensao_sistema_1..2, adc_tensao_ntc_1..2,
                # adc_corrente_carga_1..2, adc_corrente_sistema_1..2
    'HHHHhh'    # tensao_bateria_1..2, carga_bateria_1..2,
                # taxa_carga_descarga_bateria_1..2
    )

    TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FORMAT)

    print(f'Expecting {TELEMETRY_SIZE} bytes per packet')

    data = ser.read(TELEMETRY_SIZE)
    if len(data) == TELEMETRY_SIZE:
        fields = struct.unpack(TELEMETRY_FORMAT, data)
        (hora, minuto, segundo, dia, mes, ano,
        temp_bateria, temp_estrutura, umidade, pressao,
        deploy_antena,
        tensao_13v8, tensao_5v, corrente_13v8, corrente_5v,
        tensao_painel_1, tensao_painel_2, tensao_painel_3, tensao_painel_4,
        status_carregador_1, status_carregador_2,
        falha_carregador_1, falha_carregador_2,
        adc_tensao_sis_1, adc_tensao_sis_2,
        adc_tensao_ntc_1, adc_tensao_ntc_2,
        adc_corrente_carga_1, adc_corrente_carga_2,
        adc_corrente_sis_1, adc_corrente_sis_2,
        tensao_bat_1, tensao_bat_2,
        carga_bat_1, carga_bat_2,
        taxa_bat_1, taxa_bat_2) = fields

        print(f"=== Telemetria ===")
        print(f"Hora:        {hora:02d}:{minuto:02d}:{segundo:02d}  {dia:02d}/{mes:02d}/{ano:02d}")
        print(f"Temp bateria:    {temp_bateria:.2f} °C")
        print(f"Temp estrutura:  {temp_estrutura:.2f} °C")
        print(f"Umidade: {umidade:.2f}%")
        print(f"Pressão: {pressao:.2f} Pa")
        print(f"Deploy:      {deploy_antena}")
        print(f"Tensão 13V8: {tensao_13v8:.3f} V  |  Corrente: {corrente_13v8:.3f} A")
        print(f"Tensão 5V:   {tensao_5v:.3f} V    |  Corrente: {corrente_5v:.3f} A")
        print(f"Painéis:     {tensao_painel_1:.3f} V  {tensao_painel_2:.3f} V  "
            f"{tensao_painel_3:.3f} V  {tensao_painel_4:.3f} V")
        print(f"Carregador status: {status_carregador_1} {status_carregador_2}  "
            f"falha: {falha_carregador_1} {falha_carregador_2}")
        print(f"Bateria 1:   {tensao_bat_1} mV  carga: {carga_bat_1}  taxa: {taxa_bat_1}")
        print(f"Bateria 2:   {tensao_bat_2} mV  carga: {carga_bat_2}  taxa: {taxa_bat_2}")

def main():
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=5)

    while True:
        marker = ser.read(1)

        if not marker:
            continue
            
        if marker == b'H':
            print('[HK] Housekeeping ran')
            marker = 0
        
        elif marker == b'R':
            print('[RC] Receive window open')
            tc = int(input('Type the telecommand: ')).to_bytes(1)
            ser.write(tc)
            time.sleep(0.5)
            response = ser.readline()
            print(response.decode(errors='replace'))
            time.sleep(0.5)
            response = ser.readline()
            print(response.decode(errors='replace'))
            marker = 0
        
        elif marker == b'T':
            read_telemetry(ser)
            marker = 0

if __name__ == '__main__':
    main()
