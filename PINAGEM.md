# H-DROP — Conexões e Pinagem (Arduino Uno)

Este documento descreve a pinagem do firmware atual em [src/main.cpp](src/main.cpp): 3 sensores ultrassônicos com filtro de 5 camadas, MPU-6050, célula de carga (HX711) e fita LED indicadora.

> O [platformio.ini](platformio.ini) compila apenas um `.cpp` por vez (`build_src_filter`). [src/mpu.cpp](src/mpu.cpp) e [src/ultrassonicos.cpp](src/ultrassonicos.cpp) são variantes isoladas para diagnóstico — usam os mesmos pinos dos seus respectivos sensores, descritos abaixo.

---

## Tabela de pinagem

| Componente | Sinal | Pino Arduino Uno |
|---|---|---|
| Ultrassônico Frontal (0°) | TRIG | 2 |
| Ultrassônico Frontal (0°) | ECHO | 3 |
| Ultrassônico Esquerdo (+90°) | TRIG | 4 |
| Ultrassônico Esquerdo (+90°) | ECHO | 5 |
| Ultrassônico Direito (-90°) | TRIG | 9 |
| Ultrassônico Direito (-90°) | ECHO | 10 |
| Célula de carga (HX711) | DOUT | 6 |
| Célula de carga (HX711) | SCK | 7 |
| Fita LED (indicador intermitente) | sinal | 8 |
| MPU-6050 (I2C) | SDA | A4 |
| MPU-6050 (I2C) | SCL | A5 |

## Alimentação

- Todos os sensores: **VCC → 5V**, **GND → GND comum** com o Arduino.
- Sensores ultrassônicos alimentados pela fonte MB102 (5V), com GND comum ao Arduino.
- MPU-6050: não conectar os pinos `XDA`, `XCL`, `AD0`, `INT`.
- HX711: alimentar o módulo em 5V; a célula de carga conecta nos terminais E+/E-/A+/A- do próprio módulo HX711 (não direto no Arduino).

## Notas

- `LOADCELL_CALIBRATION_FACTOR` em [src/main.cpp](src/main.cpp) está como placeholder (`1.0f`) — calibrar com um peso conhecido antes de confiar na leitura de `peso_kg`.
- A fita LED apenas pisca a cada `LED_BLINK_MS` (500 ms), sem relação com os demais sensores.
- Saída de diagnóstico via Serial (115200 baud) no formato Teleplot: `>f:,l:,r:,af:,al:,ar:,ax:,ay:,az:,gx:,gy:,gz:,peso:`.
