#ifndef SX1276_H
#define SX1276_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ==================== REGISTROS SX1276 ====================
#define REG_FIFO                   0x00
#define REG_OP_MODE                0x01
#define REG_FRF_MSB                0x06
#define REG_FRF_MID                0x07
#define REG_FRF_LSB                0x08
#define REG_PA_CONFIG              0x09
#define REG_LNA                    0x0C
#define REG_FIFO_ADDR_PTR          0x0D
#define REG_FIFO_TX_BASE_ADDR      0x0E
#define REG_FIFO_RX_BASE_ADDR      0x0F
#define REG_FIFO_RX_CURRENT_ADDR   0x10
#define REG_IRQ_FLAGS_MASK         0x11
#define REG_IRQ_FLAGS              0x12
#define REG_RX_NB_BYTES            0x13
#define REG_PKT_SNR_VALUE          0x19
#define REG_PKT_RSSI_VALUE         0x1A
#define REG_MODEM_CONFIG_1         0x1D
#define REG_MODEM_CONFIG_2         0x1E
#define REG_SYMB_TIMEOUT_LSB       0x1F
#define REG_PREAMBLE_MSB           0x20
#define REG_PREAMBLE_LSB           0x21
#define REG_PAYLOAD_LENGTH         0x22
#define REG_MAX_PAYLOAD_LENGTH     0x23
#define REG_MODEM_CONFIG_3         0x26
#define REG_SYNC_WORD              0x39
#define REG_DIO_MAPPING_1          0x40
#define REG_VERSION                0x42
#define REG_PA_DAC                 0x4D

// Modos (REG_OP_MODE)
#define MODULATION_LORA            1 // 1:LORA, 0:FSK
// #define MODE_SLEEP                 0x00
// #define MODE_STDBY                 0x01
// #define MODE_TX                    0x03
// #define MODE_RX_CONTINUOUS         0x05

/* Bits IRQ Flags */
#define IRQ_RX_TIMEOUT_MASK        0x80
#define IRQ_RX_DONE_MASK           0x40
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_TX_DONE_MASK           0x08



// ==================== PINES ====================
/* LoRa SPI1 */
#define LORA_PIN_MISO   GPIO_NUM_19
#define LORA_PIN_MOSI   GPIO_NUM_23
#define LORA_PIN_SCK    GPIO_NUM_18
#define LORA_PIN_CS     GPIO_NUM_5  //Equivalente SS este tmb es uout

/* LoRa Control*/
#define LORA_PIN_RST     GPIO_NUM_4  //GPIO out // cambiar a 33
#define LORA_PIN_DIO0    GPIO_NUM_15  // usado para IRQ de TxDone / RxDone // cambiar a 32

// ==================== PARÁMETROS PARA SPI CONGIF ====================

#define LORA_SPI_HOST               SPI2_HOST           // ESP32 DevKit 38 pines - bus SPI2_HOST para los pines 19,23,18
#define LORA_SPI_CLOCK_HZ           (5 * 1000 * 1000)   // 8 MHz, el SX1276 soporta hasta 10 MHz
#define LORA_MF_ID                  0x12                //Semtech ID para hacer check de comunicación SPI

// ==================== PARÁMETROS DE LORA ====================
#define LORA_RST_WAIT_TIME_LOW      1                   //Minimo 100us
#define LORA_RST_WAIT_TIME_HIGH     10                  //Minimo 5ms

#define LORA_TX_TIMEOUT_MS          4000


#define LORA_FREQUENCY_HZ       915000000UL // Banda ISM permitida en Argentina/ENACOM: 902-928 MHz
#define LORA_TX_POWER_DBM       17      // Entre 2 y 17 en modo PA_BOOST, o 20 con PA_DAC en +20dBm
#define LORA_PREAMBLE_LEN       8       // Cantidad de simbolos de preambulo
#define LORA_SYNC_WORD          0x12    // distinto del público 0x34 para no mezclarse con otras redes

#define LORA_MAX_PAYLOAD        124     // FIFO de hasta 256 bytes. Defino la mitad para envio de datos. Redimensionar en funcion de la trama.
#define HEADER_MODE             0       // 1: Implicito, 0: Explicito
#define CRC_MODE                1       // 1: Habilitado, 0: Deshabilitadi

typedef enum {
    LORA_BW_7_kHz, //0
    LORA_BW_10_kHz,
    LORA_BW_15_kHz,
    LORA_BW_20_kHz,
    LORA_BW_31_kHz,
    LORA_BW_41_kHz,
    LORA_BW_62_kHz,
    LORA_BW_125_kHz,
    LORA_BW_250_kHz,
    LORA_BW_500_kHz, //9
}sx1276_bw_options;

typedef enum {
    LORA_MODE_SLEEP, //0
    LORA_MODE_STANDBY,
    LORA_MODE_FSTX,
    LORA_MODE_TX,
    LORA_MODE_FSRX,
    LORA_MODE_RX //5
} sx1276_modes;

typedef enum {
     /*Coding rate
        001 para 4/5
        010 para 4/6
        011 para 4/7
        100 para 4/8
    */
    CR_4_5 = 1,
    CR_4_6,       
    CR_4_7,
    CR_4_8
} sx1276_coding_rate;

typedef enum {
    SF_6 = 6,
    SF_7,
    SF_8,
    SF_9,
    SF_10,
    SF_11,
    SF_12,
} sx1276_sf;

/**
    Inicializa SPI, resetea el módulo, verifica el chip (REG_VERSION) y
    configura frecuencia, potencia y parámetros del modem.
    Debe llamarse una sola vez al arrancar, antes de transmitir o recibir.
 */
esp_err_t sx1276_init(void);


/*
    Envía un paquete de forma bloqueante. Espera hasta LORA_TX_TIMEOUT_MS
    a que llegue la interrupción de TxDone.
    Devuelve ESP_OK si se transmitió, ESP_ERR_TIMEOUT si no hubo respuesta a tiempo.
 */
esp_err_t sx1276_send_packet(const uint8_t *data, uint8_t len);


/*
    Pone el módulo en modo recepción continua. Se llama una sola vez
    antes del loop de recepción.
 */
void sx1276_start_receive(void);

/*
    Bloquea la tarea actual hasta que llegue un paquete (o hasta timeout_ms,
    0 = esperar indefinidamente). Al volver, buf contiene el payload y
    *len su longitud. rssi y snr son opcionales (pasar NULL si no interesan).
    Devuelve ESP_OK si llegó paquete válido (CRC ok), ESP_ERR_TIMEOUT si venció
    el timeout, ESP_ERR_INVALID_CRC si el paquete llegó corrupto.
 */
esp_err_t sx1276_receive_packet(uint8_t *buf, uint8_t *len, int16_t *rssi, float *snr, uint32_t timeout_ms);

#endif // SX1276_H
