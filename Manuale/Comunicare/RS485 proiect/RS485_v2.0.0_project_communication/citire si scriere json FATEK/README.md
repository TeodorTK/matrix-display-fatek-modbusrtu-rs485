# ESP32 Modbus RTU Client - Citire și Scriere JSON FATEK

Acest program permite citirea și scrierea registrilor Modbus RTU ai unui PLC FATEK prin intermediul JSON.

## Funcționalități

- **Citire automată**: Citește registrii D și M de 5 ori pe secundă (200ms interval)
- **Scriere prin JSON**: Permite scrierea registrilor prin trimiterea unui JSON prin Serial
- **Frecvență**: 5 citiri/secundă și 5 scrieri/secundă

## Registrii suportați

### Data Registers (D):
- D0 - D10 (11 registri)
- D46 - D48 (3 registri)
- D57 - D58 (2 registri)

### Discrete M Relays:
- M0
- M70 - M79 (10 coils)
- M93 - M95 (3 coils)
- M97 - M98 (2 coils)
- M120

## Format JSON pentru scriere

Trimiteți un JSON prin Serial Monitor cu următorul format:

```json
{
  "D": {
    "D0": 100,
    "D1": 200,
    "D2": 300,
    "D46": 4600,
    "D57": 5700
  },
  "M": {
    "M0": true,
    "M70": false,
    "M71": true,
    "M93": false,
    "M97": true,
    "M120": false
  }
}
```

### Exemple de valori:
- **D registrii**: Numere întregi (0-65535)
- **M coils**: `true` sau `false` (ON/OFF)

## Configurare

- **Baudrate**: 9600
- **Parity**: Even (8E1)
- **RX Pin**: 16
- **TX Pin**: 17
- **RE/DE Pin**: 22
- **Slave ID**: 5

## Utilizare

1. Conectați ESP32 la PLC-ul FATEK prin RS485
2. Deschideți Serial Monitor la 115200 baud
3. Programul va începe să citească automat registrii
4. Pentru a scrie registrii, trimiteți un JSON prin Serial Monitor
5. Programul va procesa scrierile la 5 Hz (un registru la fiecare 200ms)

## Fișier exemplu

Vezi `config_exemplu.json` pentru un exemplu complet de configurare JSON.

## Note

- Programul citește și scrie asincron, fără a bloca sistemul
- Scrierile sunt procesate un registru la un moment dat pentru stabilitate
- Toate valorile citite sunt afișate în Serial Monitor după fiecare ciclu complet de citire

