/* 
    Ejemplo de actualización de un nodo remoto cuando la transmisión del binario
    se realiza por LoRa desde el gateway.
*/
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "protocol_examples_common.h"
#include "errno.h"
#include "sx1276.h"

#define BUFFSIZE 1024
#define HASH_LEN 32 /* SHA-256 digest length */

static const char *TAG = "Nodo-Remoto";
/*an ota data write buffer ready to write to the flash*/
static char ota_write_data[BUFFSIZE + 1] = { 0 };

static void __attribute__((noreturn)) task_fatal_error(void)
{
    ESP_LOGE(TAG, "Exiting task due to fatal error...");
    (void)vTaskDelete(NULL);

    while (1) {
        ;
    }
}

static void print_sha256 (const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i) {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s: %s", label, hash_print);
}

static void infinite_loop(void)
{
    int i = 0;
    ESP_LOGI(TAG, "When a new firmware is available on the server, press the reset button to download it");
    while(1) {
        ESP_LOGI(TAG, "Waiting for a new firmware ... %d", ++i);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

static void ota_by_lora_example_task(void *pvParameter)
{
    esp_err_t err;
    
    esp_ota_handle_t update_handle = 0 ;
    /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
    
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Starting OTA example task");

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running) {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08"PRIx32", but running from offset 0x%08"PRIx32,
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08"PRIx32")",
             running->type, running->subtype, running->address);
    
    
    update_partition = esp_ota_get_next_update_partition(NULL);
    assert(update_partition != NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%"PRIx32,
             update_partition->subtype, update_partition->address);

    int binary_file_length = 0;
    bool header_is_ok = false;
    
    while (1) {
        int data_read = lora_recepcion(/* params */); //Recibo por lora
        
        if (data_read < 0) {
            ESP_LOGI(TAG, "No se recibieron datos.");

        }
        else if (data_read > 0)
        {
            ESP_LOGI(TAG, "Comenzando update OTA por LoRa...");

            if (header_is_ok == false)
            {
                //Asumo que será válido, las verificaciones finales cambian el estado en caso de ser necesario.
                header_is_ok = true;
                
                esp_app_desc_t new_app_info;

                // Validar que el paquete cuente minimamente con un tamaño de header compatible con imagen ESP-IDF
                if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
                {
                    memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

                    esp_app_desc_t running_app_info;
                    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                    }

                    const esp_partition_t* last_invalid_app = esp_ota_get_last_invalid_partition();
                    esp_app_desc_t invalid_app_info;
                    if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                    }
                    
                    
                    
                    // Verificar que la versión de app recibida no coincida con la última inválida
                    if (last_invalid_app != NULL) {
                        if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0) {
                            ESP_LOGW(TAG, "Actualización descartada: La versión recibida coincide con la última inválida.");
                            header_is_ok = false;
                        }
                    }
                    
                    // Verificar que la versión de app no sea la misma que está corriendo
                    if(memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0){
                        ESP_LOGW(TAG, "Actualización descartada: La versión recibida se encuentra en ejecución.");
                        header_is_ok = false;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "La cantidad de bytes recibidos es menor al esperado para el header de una nueva app.");
                }
            }
            else
            {
                err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                        //http_cleanup(client); //Pertenece a update por http
                        esp_ota_abort(update_handle);
                        task_fatal_error();
                    }
                ESP_LOGI(TAG, "esp_ota_begin succeeded");

                err = esp_ota_write( update_handle, (const void *)ota_write_data, data_read);
            
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Falló esp_ota_write. (%s)", esp_err_to_name(err));
                    ESP_LOGE(TAG, "Descartando actualización");
                    esp_ota_abort(update_handle);
                    header_is_ok = false;
                    binary_file_length = 0;
                    //Avisar a LoRa que falló, para que no siga enviando paquetes.
                }
                binary_file_length += data_read;
                ESP_LOGD(TAG, "Tamaño de imagen escrita %d", binary_file_length);

                if(binary_file_length == app_size)
                {
                    ESP_LOGI(TAG, "Se completó la recepción de la nueva app.");
                    break;
                }
            }
        }
    }

    ESP_LOGI(TAG, "Total Write binary data length: %d", binary_file_length);
    if (esp_http_client_is_complete_data_received(client) != true) {
        ESP_LOGE(TAG, "Error in receiving complete file");
        //http_cleanup(client); //Pertenece a update por http
        esp_ota_abort(update_handle);
        task_fatal_error();
    }

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        } else {
            ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        }
        //http_cleanup(client); //Pertenece a update por http
        task_fatal_error();
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        //http_cleanup(client); //Pertenece a update por http
        task_fatal_error();
    }
    ESP_LOGI(TAG, "Prepare to restart system!");
    esp_restart();
    return ;
}

static bool diagnostic(void)
{
    /*
    Diagnostico de la nueva app para asegurar que sea válida.
    Por ahora harcodeado en true.
    */
    bool diagnostic_is_ok = true;

    return diagnostic_is_ok;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Ejemplo OTA por transmisión LoRa");

    /*------------------- Print de los hash de particiones para debug -------------------*/ 
    uint8_t sha_256[HASH_LEN] = { 0 };
    esp_partition_t partition;

    // Obtener hash de la tabla de particiones
    partition.address   = ESP_PARTITION_TABLE_OFFSET;
    partition.size      = ESP_PARTITION_TABLE_MAX_LEN;
    partition.type      = ESP_PARTITION_TYPE_DATA;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for the partition table: ");

    // Obtener hash del bootloader
    partition.address   = ESP_BOOTLOADER_OFFSET;
    partition.size      = ESP_PARTITION_TABLE_OFFSET;
    partition.type      = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 for bootloader: ");

    // Obtener hash de la partición de arranque (corriendo actualmente)
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 for current firmware: ");

    /*------------------- Logica de decisión del rollback -------------------*/ 

    const esp_partition_t *running = esp_ota_get_running_partition();

    esp_ota_img_states_t ota_state;
    esp_err_t err = esp_ota_get_state_partition(running, &ota_state);
    if (err == ESP_OK) {
        
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            // run diagnostic function ...
            bool diagnostic_is_ok = diagnostic();
            
            if (diagnostic_is_ok) {
                ESP_LOGI(TAG, "Diagnostics completed successfully! Continuing execution ...");
                esp_ota_mark_app_valid_cancel_rollback();
            } else {
                ESP_LOGE(TAG, "Diagnostics failed! Start rollback to the previous version ...");
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }
    ESP_ERROR_CHECK(err);

    // Initialize NVS.
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    

    xTaskCreate(&ota_by_lora_example_task, "ota_by_lora_example_task", 8192, NULL, 5, NULL);
}