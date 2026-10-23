import argparse
from email import parser
import struct
import threading

import serial

TELEMETRY_FORMAT = (
    '<'         # little-endian
    'BBBBBB'    # hora, minuto, segundo, dia, mes, ano
    'ffff'      # temperatura_bateria, temperatura_estrutura, umidade, pressao
    'B'         # deploy_antena
    'ffff'      # tensao_13v8, tensao_5v, corrente_13v8, corrente_5v
    'ffff'      # tensao_painel_solar_1..4
    'BBBB'      # status_carregador_1..2, falha_carregador_1..2
    'BBBBBBBB'  # ADCs
    '6s'        # angulo_apontamento
    'HHHHhh'    # dados das baterias
)

TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FORMAT)


def read_exact(ser, size):
    # cria vetor data vazio para armazenar os bytes lidos
    data = bytearray()

    # le bytes do serial e coloca em data ate ter tamanho size
    while len(data) < size:
        chunk = ser.read(size - len(data))
        if not chunk:
            return None
        data.extend(chunk)

    return bytes(data)

def read_telemetry(ser):
    # le o pacote de telemetria do serial
    data = read_exact(ser, TELEMETRY_SIZE)
    if data is None:
        print('[USART2] incomplete telemetry packet')
        return

    # coloca no formato TELEMETRY_FORMAT
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
        angulo_apontamento,
        tensao_bat_1, tensao_bat_2,
        carga_bat_1, carga_bat_2,
        taxa_bat_1, taxa_bat_2) = fields

    angulo = angulo_apontamento.rstrip(b'\0').decode(errors='replace')

    # imprime a telemetria
    print('=== Telemetria ===')
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
    print(f"Ângulo:      {angulo}")
    print(f"Bateria 1:   {tensao_bat_1} mV  carga: {carga_bat_1}  taxa: {taxa_bat_1}")
    print(f"Bateria 2:   {tensao_bat_2} mV  carga: {carga_bat_2}  taxa: {taxa_bat_2}")


def read_usart2(ser, stop_event):
    while not stop_event.is_set():
        # le primeiro byte do serial, que indica tipo de mensagem
        marker = ser.read(1)

        if not marker:
            continue

        if marker == b'H':
            print('[HK] housekeeping ran')
        elif marker == b'T':
            read_telemetry(ser)
        # se for mensagem de debug, le a linha completa e imprime
        elif marker == b'[': 
            line = marker + ser.readline()
            print(line.decode(errors='replace').rstrip())
        else:
            print(f'[USART2] unexpected byte: 0x{marker[0]:02X}')


def parse_args():
    # cria o parser de argumentos da linha de comando
    parser = argparse.ArgumentParser(
        description='Test USART1 telecommands and USART2 debug/telemetry.'
    )
    # parser.add_argument('--debug-port', default='/dev/ttyACM0',
    #                     help='USART2 port (default: /dev/ttyACM0)')
    # parser.add_argument('--command-port', default='/dev/ttyUSB0',
    #                     help='USART1 USB-UART port (default: /dev/ttyUSB0)')

    # para windows, usa com4 e com3 como portas padrão se nao mudar na linha de comando
    parser.add_argument('--debug-port', default='COM4',
                        help='USART2 port (default: COM4)')
    parser.add_argument('--command-port', default='COM3',
                        help='USART1 USB-UART port (default: COM3)')

    # deixa baudrate padrao 115200, mas pode mudar na linha de comando
    parser.add_argument('--baudrate', type=int, default=115200)
    return parser.parse_args()

def main():
    args = parse_args()
    # cria um evento para parar a thread de leitura do serial
    stop_event = threading.Event()

    # abre as portas seriais para debug e comandos
    debug_serial = serial.Serial(
        args.debug_port, args.baudrate, timeout=0.2
    )
    command_serial = serial.Serial(
        args.command_port, args.baudrate, timeout=0.2
    )

    # cria uma thread para ler o serial de debug 
    reader = threading.Thread(
        target=read_usart2,
        args=(debug_serial, stop_event),
        daemon=True
    )
    reader.start()

    print(f'USART2 debug: {args.debug_port}')
    print(f'USART1 commands: {args.command_port}')
    print(f'Telemetry packet size: {TELEMETRY_SIZE} bytes')
    print('Commands: 0x01=mission, 0x02=deploy, 0x04=detumbling, q=quit')

    try:
        while True:
            # le telecomando do usuario e envia para o serial de comando 

            value = input('tc> ').strip()

            if value.lower() in {'q', 'quit', 'exit'}:
                break

            try:
                telecommand = int(value, 0)
            except ValueError:
                print('Use a decimal value or hexadecimal such as 0x01.')
                continue

            if not 0 <= telecommand <= 0xFF:
                print('The telecommand must fit in one byte (0..255).')
                continue

            command_serial.write(bytes([telecommand]))
            command_serial.flush()
            print(f'[USART1] sent 0x{telecommand:02X}')
            ack = command_serial.read(1)
            if ack:
                print(f'[USART1] received 0x{ack[0]:02X}')
            else:
                print('No response (timeout)')

    finally:
        stop_event.set()
        reader.join(timeout=1)
        command_serial.close()
        debug_serial.close()

if __name__ == '__main__':
    main()
