import customtkinter as ctk
import serial
import struct

ctk.set_appearance_mode('dark')


def terminal():
    porta = campo_port.get()
    baud = campo_bau.get()
    tipo = option_menu.get()
    tamanho = option_tamanho.get()

    if tipo == 'vetor':
        TAMANHO_PACOTE = struct.calcsize('<' + tamanho + 'B') 

        try:
            ser = serial.Serial(porta, baud)
            
            while True:
                dado_bruto = ser.read(TAMANHO_PACOTE)
                
                if len(dado_bruto) == TAMANHO_PACOTE:
                    valor_traduzido = struct.unpack('<' + tamanho + 'B', dado_bruto)[0]

                    print(f"Recebido: {valor_traduzido}")

                    resultado.configure(text = valor_traduzido)
                
        
        except KeyboardInterrupt:
            print("\nPrograma encerrado.")
        finally:
            if 'ser' in locals():
                ser.close()

    if tipo == 'float':
        TAMANHO_PACOTE = struct.calcsize('<f') 

        try:
            ser = serial.Serial(porta, baud)
            
            while True:
                dado_bruto = ser.read(TAMANHO_PACOTE)
                
                if len(dado_bruto) == TAMANHO_PACOTE:
                    valor_traduzido = struct.unpack('<f', dado_bruto)[0]
            
                    print(f"Recebido: {valor_traduzido:.2f}")

                    resultado.configure(text = valor_traduzido)
                
        
        except KeyboardInterrupt:
            print("\nPrograma encerrado.")
        finally:
            if 'ser' in locals():
                ser.close()

app = ctk.CTk()
app.title("Leitor Serial")
app.geometry('400x400')

campo_porta = ctk.CTkLabel(app, text = "Porta")
campo_porta.pack()
campo_port = ctk.CTkEntry(app, placeholder_text = 'Exemplo: "COM3"')
campo_port.pack(pady = '8')
campo_baud = ctk.CTkLabel(app, text = "Baud")
campo_baud.pack()
campo_bau = ctk.CTkEntry(app, placeholder_text = 'Exemplo: "9600"')
campo_bau.pack(pady = '8')

campo_tipos = ctk.CTkLabel(app, text = "Tipos")
campo_tipos.pack()
tipos = ['float', 'inteiro', 'vetor', 'string']

option_menu = ctk.CTkOptionMenu(app, values=tipos)
option_menu.pack()


option_vetor = ctk.CTkLabel(app, text = "Tamanho do vetor")
option_vetor.pack(pady = '10')
option_tamanho = ctk.CTkEntry(app, placeholder_text = 'Exemplo: "2"')
option_tamanho.pack(pady = '10')

resultado = ctk.CTkLabel(app, text=" ")
resultado.pack(pady = '10')

campo_button = ctk.CTkButton(app, text = "Captura de dados", command = terminal)
campo_button.pack(pady = '20')

campo_break = ctk.CTkButton(app, text = "break", command = terminal)
campo_break.pack(pady = '10')

app.mainloop()