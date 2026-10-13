
#include "sx1276.h"
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "sx1276_nodo_X";  //Para debug

static spi_device_handle_t s_spi;
static TaskHandle_t s_waiting_task = NULL; // tarea a notificar desde la ISR de DIO0


static uint8_t sx1276_read_reg(uint8_t reg);
static void sx1276_write_reg(uint8_t reg, uint8_t value);
static void sx1276_set_lna_boost(bool boost_mode);
static void sx1276_reset_lora(void);
static void sx1276_set_cs(bool mode);
static void sx1276_set_irq(uint8_t pin, gpio_mode_t mode, gpio_int_type_t type, gpio_isr_t handlerisr);
static void sx1276_set_frequency(uint32_t freq_hz);
static void sx1276_set_mode(sx1276_modes lora_mode);
static void sx1276_set_packet_params(uint8_t header_mode, sx1276_sf sf, uint8_t crc_mode, sx1276_coding_rate cr);
static void sx1276_set_bw(sx1276_bw_options bw);
static void sx1276_read_fifo(uint8_t *buf, uint8_t len);
static void sx1276_write_fifo(const uint8_t *buf, uint8_t len);

// ==================== ISR de DIO0 ====================
static void IRAM_ATTR dio0_isr_handler(void *arg)
{
    /*
    El módulo LoRa pondrá el pin DIO0 en alto cuando finalice la transmisión
    o recepción. Del lado del ESP32 validamos que tal evento se corresponda
    con alguna tarea que haya disparado la transmisión o recepción y que no
    sea ruido en el PIN.

    */


    BaseType_t xHigherPriorityTaskWoken = pdFALSE;                   
    
    /*
        Verifico si alguna tarea se encuentra en espera
        Si la hay, notifico a la tarea que ya puede desbloquearse.
    */
    if (s_waiting_task != NULL) {                                           
        vTaskNotifyGiveFromISR(s_waiting_task, &xHigherPriorityTaskWoken);
    }

    /*
        Si la tarea en espera tiene mayor prioridad que la que
        siguió ejecutándose luego de que ésta se bloqueara, y antes de producirse la IRQ,
        xHigherPriorityTaskWoken se pondrá en true.
        Si eso ocurre, forzamos que el flujo del programa retome en el punto en el que se bloqueó
        la tarea en espera, en lugar de esperar el próximo quantum del scheduler.
    */

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


esp_err_t sx1276_init(void)
{
    // CS: false -> pone en nivel alto para desconectar al slave del bus.
    gpio_reset_pin(LORA_PIN_CS);
    gpio_set_direction(LORA_PIN_CS, GPIO_MODE_OUTPUT);
    sx1276_set_cs(false);

    // Reset: Nivel bajo durante 100us segun datasheet. Luego son 5ms antes de usar el pin.
    sx1276_reset_lora();
   
    /* Configuracion de SPI */
    // Defino bus SPI para el master.
    spi_bus_config_t buscfg = {
        .miso_io_num = LORA_PIN_MISO,
        .mosi_io_num = LORA_PIN_MOSI,
        .sclk_io_num = LORA_PIN_SCK,
        .quadwp_io_num = -1, 
        .quadhd_io_num = -1,
        .max_transfer_sz = LORA_MAX_PAYLOAD,
    };

    // Inicializo el bus SPI
    esp_err_t err = spi_bus_initialize(LORA_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    
    // Valido inicializacion sin errores
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize fallo: %s", esp_err_to_name(err));
        return err;
    }

    // Defino configuración para la comunicación SPI con el slave.
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = LORA_SPI_CLOCK_HZ,    // Máximo 10MHz para SX1276
        .mode = 0,                              // Módulo samplea en flanco de subida.
        .spics_io_num = -1,                     // Control manual del nivel de CS.
        .queue_size = 1,                        // Cantidad de transacciones en la queue
    };
    
    // Agrego la conexión de la interfaz SPI2_HOST al bus que inicialicé
    err = spi_bus_add_device(LORA_SPI_HOST, &devcfg, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device fallo: %s", esp_err_to_name(err));
        return err;
    }

    // Prueba de conectividad de LoRa (version esperada es LORA_MF_ID=0x12 para SX1276)
    uint8_t version = sx1276_read_reg(REG_VERSION);
    if (version != LORA_MF_ID) {
        ESP_LOGE(TAG, "No se detecto SX1276 (REG_VERSION=0x%02X). Revisar cableado.", version);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "SX1276 detectado (version 0x%02X)", version);

    // Apago todo lo que no necesito durante la inicialización. Solo disponibilizo registros.
    sx1276_set_mode(LORA_MODE_SLEEP);
   
      
    // Redimensionamiento de memoria para TX y TX del módulo. Posible mejora.
    // sx1276_write_reg(REG_FIFO_TX_BASE_ADDR, tx_fifo_len);
    // sx1276_write_reg(REG_FIFO_RX_BASE_ADDR, rx_fifo_len);
    
    
    /*   Configuraciones propias de la comunicación LoRa    */
    sx1276_set_frequency(LORA_FREQUENCY_HZ); 
    
    sx1276_set_bw(LORA_BW_125_kHz);

    sx1276_set_packet_params(HEADER_MODE,SF_7,CRC_MODE,CR_4_5);
    

    /* Config del preambulo */
    sx1276_write_reg(REG_PREAMBLE_MSB, 0x00);
    sx1276_write_reg(REG_PREAMBLE_LSB, LORA_PREAMBLE_LEN);
    sx1276_write_reg(REG_SYNC_WORD, LORA_SYNC_WORD);
    
    // --- Configuraciones electricas para transmisión y recepcion de RF
    // LNA con Boost de corriente apagado para minimizar consumo.
    sx1276_set_lna_boost(false);  

    // Automatic Gain Control activo.
    // El bloque LNA ajusta automáticamente el nivel de ganancia necesario.
    sx1276_write_reg(REG_MODEM_CONFIG_3, 0x04);
    
    // Potencia de salida (PA_BOOST)
    uint8_t power = LORA_TX_POWER_DBM;
    if (power > 17) power = 17;
    if (power < 2) power = 2;
    sx1276_write_reg(REG_PA_CONFIG, 0x80 | (power - 2)); // 0x80 = PA_BOOST prendido

    /* Prueba config para llegar a los 500m en ciudad */
    /* PA_BOOST 20dBm on */
    /* uint8_t power = 20;
    sx1276_write_reg(REG_PA_CONFIG, 0x80 | 0x04 | 0x0F);
    sx1276_write_reg(REG_PA_DAC, 0x10 | 0x07);
    //OcpTrim 23 == 0x17 --> imax = 200mA
    sx1276_write_reg(REG_OCP, 0x10 | 0x17); */
    
    // DIO0 como entrada de interrupción (flanco ascendente)
    sx1276_set_irq(LORA_PIN_DIO0, GPIO_MODE_INPUT,GPIO_INTR_POSEDGE,dio0_isr_handler);

    sx1276_set_mode(LORA_MODE_STANDBY);    

    ESP_LOGI(TAG, "SX1276 inicializado a %lu Hz, SF%d, potencia %d dBm",
             (unsigned long)LORA_FREQUENCY_HZ, SF_7, power);

    return ESP_OK;
}

esp_err_t sx1276_send_packet(const uint8_t *data, uint8_t len)
{
    // if (LORA_MAX_PAYLOAD < len ) {
    //     return ESP_ERR_INVALID_SIZE;
    // }

    // Fuerzo sacar el dispositivo de SLEEP si lo estuviera, para que la FIFO sea accesible.
    sx1276_set_mode(LORA_MODE_STANDBY);

    // Seteo el comporatmiento de DIO0 para que se ponga en alto ante TxDone (01 en bits 6 y 7)
    sx1276_write_reg(REG_DIO_MAPPING_1, 0x40);
    
    // FIFO apuntando al comienzo
    sx1276_write_reg(REG_FIFO_ADDR_PTR, 0x00);

    //  Escribo en la FIFO
    sx1276_write_fifo(data, len);
    
    // Indico cantidad de bytes a transmitir. Obligatorio si el header es implícito
    sx1276_write_reg(REG_PAYLOAD_LENGTH, len);

    // Limpio flags de IRQ previos
    sx1276_write_reg(REG_IRQ_FLAGS, 0xFF);

    // Obtengo el identificador/handle de esta task para usarlo en dio0_isr_handler
    s_waiting_task = xTaskGetCurrentTaskHandle();
    // Borro la notificación de esta tarea para que sea el hanlder de isr quien la incremente.
    ulTaskNotifyValueClear(NULL, 0xFFFFFFFF); 

    sx1276_set_mode(LORA_MODE_TX);

    /*
        Bloqueo la tarea el tiempo suficiente para transmitir los datos 
        o hasta que reciba la notificación de TxDone mediante la IRQ de DIO0
        Si se agota el tiempo, notified valdrá 0.
    */ 
    uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(LORA_TX_TIMEOUT_MS));
   
    
    /* 
        Finalizado el bloqueo marco que la task ya no está en espera.
        Valido leyendo los registros correspondientes si efectivamente
        se produjo el TxDone y limpio los flags
    */
    s_waiting_task = NULL;
    
    uint8_t flag_tx_done = sx1276_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK;

    if (notified == 0) {
        ESP_LOGW(TAG, "Timeout esperando IRQ DIO0 TxDone");
        return ESP_ERR_TIMEOUT;
    } else if( flag_tx_done == 0 ){
        ESP_LOGW(TAG, "Flag DIO0 Tx invalida. Retransmitir.");
        return ESP_ERR_INVALID_STATE;
    }

    sx1276_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    sx1276_set_mode(LORA_MODE_STANDBY);

    return ESP_OK;
}

esp_err_t sx1276_receive_packet(uint8_t *buf, uint8_t *len, int16_t *rssi, float *snr, uint32_t timeout_ms)
{
    s_waiting_task = xTaskGetCurrentTaskHandle();
    ulTaskNotifyValueClear(NULL, 0xFFFFFFFF);

    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    uint32_t notified = ulTaskNotifyTake(pdTRUE, ticks);
    s_waiting_task = NULL;

    if (notified == 0) {
        ESP_LOGW(TAG, "Timeout esperando IRQ DIO0 RxDone.");
        return ESP_ERR_TIMEOUT;
    }


    uint8_t irq_flags = sx1276_read_reg(REG_IRQ_FLAGS);

    if ( (irq_flags & IRQ_RX_DONE_MASK) == 0 ) {
        ESP_LOGW(TAG, "Flag DIO0 Rx invalida.");
        return ESP_ERR_INVALID_STATE;
    }

    if (irq_flags & IRQ_PAYLOAD_CRC_ERROR_MASK) {
        ESP_LOGW(TAG, "RxDone con CRC invalido. Paquete descartado.");
        return ESP_ERR_INVALID_CRC;
    }
    

    uint8_t nb_bytes = sx1276_read_reg(REG_RX_NB_BYTES);
    uint8_t current_addr = sx1276_read_reg(REG_FIFO_RX_CURRENT_ADDR);
    sx1276_write_reg(REG_FIFO_ADDR_PTR, current_addr);
    sx1276_read_fifo(buf, nb_bytes);
    *len = nb_bytes;

    if (rssi != NULL) {
        // Formula típica para banda de 868/915MHz (ver datasheet, offset -157 para HF)
        *rssi = (int16_t)(sx1276_read_reg(REG_PKT_RSSI_VALUE) - 157);
    }
    if (snr != NULL) {
        int8_t raw_snr = (int8_t)sx1276_read_reg(REG_PKT_SNR_VALUE);
        *snr = raw_snr / 4.0f;
    }
    
    // Limpio todos los flags
    sx1276_write_reg(REG_IRQ_FLAGS, 0xFF); 
    
    return ESP_OK;
}

void sx1276_start_receive(void)
{
    // DIO0 -> RxDone (mapeo 00 en bits 7:6, ya es 0x00 por defecto en modo RX)
    // Seteo el comporatmiento de DIO0 para que se ponga en alto cuando RxDone (00 en bits 6 y 7)
    sx1276_write_reg(REG_DIO_MAPPING_1, 0x00);
    sx1276_write_reg(REG_FIFO_ADDR_PTR, 0x00);
    sx1276_write_reg(REG_IRQ_FLAGS, 0xFF);
    sx1276_set_mode(LORA_MODE_RX);
}


// ======= Funciones estáticas =======
static void sx1276_write_fifo(const uint8_t *buf, uint8_t len)
{
    
    uint8_t tx[LORA_MAX_PAYLOAD + 1];
    tx[0] = REG_FIFO | 0x80;
    memcpy(&tx[1], buf, len);
    spi_transaction_t t = {
        .length = (size_t)(len + 1) * 8,
        .tx_buffer = tx,
    };
    sx1276_set_cs(true);
    spi_device_transmit(s_spi, &t);
    sx1276_set_cs(false);
}

static void sx1276_read_fifo(uint8_t *buf, uint8_t len)
{
    uint8_t tx[LORA_MAX_PAYLOAD + 1] = { 0 };
    uint8_t rx[LORA_MAX_PAYLOAD + 1] = { 0 };
    tx[0] = REG_FIFO & 0x7F;
    spi_transaction_t t = {
        .length = (size_t)(len + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    sx1276_set_cs(true);
    spi_device_transmit(s_spi, &t);
    sx1276_set_cs(false);
    memcpy(buf, &rx[1], len);
}

static uint8_t sx1276_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), 0x00 };
    // uint8_t tx[2] = { reg , 0x00 };
    uint8_t rx[2] = { 0 };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    sx1276_set_cs(true);
    spi_device_transmit(s_spi, &t);
    sx1276_set_cs(false);
    return rx[1];
}

static void sx1276_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), value };
    // uint8_t tx[2] = { reg, value };
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    sx1276_set_cs(true);
    
    spi_device_transmit(s_spi, &t);

    sx1276_set_cs(false);
}


static void sx1276_set_lna_boost(bool boost_mode){
    /* 
        LNA : Low Noise Amplifier / Amplificador de bajo ruido. 
        Bits 0 y 1  --> 11 para hacer Boost on, 150% LNA current
                    --> 00 para deshabilitar 
    */
    uint8_t lna_reg_value = sx1276_read_reg(REG_LNA);
    uint8_t value = boost_mode ?  (lna_reg_value | 0x03) : (lna_reg_value & ~(0x03));
    sx1276_write_reg(REG_LNA, value);
}


static void sx1276_reset_lora(void)
{
    gpio_reset_pin(LORA_PIN_RST);
    gpio_set_direction(LORA_PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(LORA_RST_WAIT_TIME_LOW)); 
    gpio_set_level(LORA_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(LORA_RST_WAIT_TIME_HIGH));
}

static void sx1276_set_cs(bool mode)
{
    gpio_set_level(LORA_PIN_CS, mode ? 0 : 1);
}


static void sx1276_set_irq(uint8_t pin, gpio_mode_t mode, gpio_int_type_t type, gpio_isr_t handlerisr){ 
    gpio_reset_pin(LORA_PIN_DIO0);
    gpio_set_direction(LORA_PIN_DIO0, GPIO_MODE_INPUT);
    gpio_set_intr_type(LORA_PIN_DIO0, GPIO_INTR_POSEDGE);
    
    // Instalar ISR de DIO0 
    // (compartida entre TX y RX, se reconfigura el mapeo según el caso)
    gpio_install_isr_service(0);
    gpio_isr_handler_add(LORA_PIN_DIO0, handlerisr, NULL);
}

static void sx1276_set_frequency(uint32_t freq_hz)
{
    // Frf = freq_hz * 2^19 / 32e6
    uint64_t frf = ((uint64_t)freq_hz << 19) / 32000000ULL;
    sx1276_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    sx1276_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    sx1276_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

static void sx1276_set_mode(sx1276_modes lora_mode)
{
    // uint8_t value = sx1276_read_reg(REG_OP_MODE) & 0x78;
    // value = value | (uint8_t)(MODULATION_LORA<<7) | (uint8_t)lora_mode;
    // sx1276_write_reg(REG_OP_MODE, value);
    sx1276_write_reg(REG_OP_MODE, MODULATION_LORA | (uint8_t)(lora_mode));

}

static void sx1276_set_bw(sx1276_bw_options bw)
{
    uint8_t value = sx1276_read_reg(REG_MODEM_CONFIG_1);
    value = (value & 0x0F) | (uint8_t)(bw << 4);
    sx1276_write_reg(REG_MODEM_CONFIG_1, value);
}

static void sx1276_set_packet_params(uint8_t header_mode, sx1276_sf sf, uint8_t crc_mode, sx1276_coding_rate cr)
{
    //Limpio los bits que quiero sobreescribir
    uint8_t value =  sx1276_read_reg(REG_MODEM_CONFIG_1) & 0xF0;
    value = value | (uint8_t)(cr << 1)| header_mode;
    sx1276_write_reg(REG_MODEM_CONFIG_1, value );
    
    value = sx1276_read_reg(REG_MODEM_CONFIG_2) & 0x0B;
    value = value | (uint8_t)(sf << 4 ) | (uint8_t)(crc_mode << 2 );
    sx1276_write_reg(REG_MODEM_CONFIG_2, value);

}