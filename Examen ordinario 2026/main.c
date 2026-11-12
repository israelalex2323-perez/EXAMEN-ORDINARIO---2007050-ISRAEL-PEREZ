
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_attr.h"

#include "driver/i2c_master.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"


/* 2. DEFINICIONES
   Este bloque establece los valores constantes que utiliza el programa durante la configuracion de los perifericos
   y la ejecucion del experimento. Primero, los tres defines de pines identifican las conexiones del potenciometro y
   las lineas SDA y SCL utilizadas por la comunicacion I2C.Despues, las definiciones del ADC establecen la unidad,
   el canal, la resolucion de 12 bits y la atenuacion que se aplicara durante las conversiones analogicas.
   Posteriormente, FRECUENCIA_ADC determina la frecuencia de adquisicion continua, mientras que TAMANO_BUFFER_ADC y
   TAMANO_POOL_ADC establecen los espacios de almacenamiento utilizados por el controlador DMA.
   Los siguientes defines establecen los tiempos de estabilizacion, duracion de las pruebas e intervalos para
   actualizar las mediciones y presentar los reportes. Finalmente, las constantes del INA219 identifican sus
   registros, direccion I2C, configuracion y calibracion.CURRENT_LSB y POWER_LSB permiten convertir los registros
   de corriente y potencia a sus respectivas unidades. VOLTAJE_REFERENCIA y VALOR_ADC_MAXIMO se utilizan para
   obtener una estimacion del voltaje del potenciometro. */

// Pines del circuito
#define PIN_POTENCIOMETRO       34
#define PIN_SDA                 21
#define PIN_SCL                 22

// Configuracion ADC
#define CANAL_ADC               ADC_CHANNEL_6
#define UNIDAD_ADC              ADC_UNIT_1
#define RESOLUCION_ADC          ADC_BITWIDTH_12
#define ATENUACION_ADC          ADC_ATTEN_DB_12

// Configuracion ADC Continuous
#define FRECUENCIA_ADC          20000
#define TAMANO_BUFFER_ADC       256
#define TAMANO_POOL_ADC         1024

// Tiempos del experimento
#define TIEMPO_ESTABILIZACION   10
#define TIEMPO_PRUEBA           120
#define INTERVALO_INA           1000000
#define INTERVALO_REPORTE       5000000

// Configuracion INA219
#define DIRECCION_INA219        0x40

#define REG_CONFIG              0x00
#define REG_BUS_VOLTAGE         0x02
#define REG_POWER               0x03
#define REG_CURRENT             0x04
#define REG_CALIBRATION         0x05

#define VALOR_CONFIG_INA219     0x399F
#define VALOR_CALIBRACION       4096

#define CURRENT_LSB             0.0001
#define POWER_LSB               0.002

// Conversion aproximada del ADC
#define VOLTAJE_REFERENCIA      3.3
#define VALOR_ADC_MAXIMO        4095


/* 3. VARIABLES GLOBALES
   Este bloque declara las variables que necesitan utilizar diferentes funciones del programa para compartir datos.
   Primero, Bus_I2C y Dispositivo_INA219 almacenan los manejadores del bus de comunicacion y del sensor INA219.
   Despues, ADC_OneShot y ADC_DMA almacenan los manejadores correspondientes a los dos metodos de adquisicion ADC.
   Las variables Voltaje_Alimentacion, Corriente_Alimentacion y Potencia_Alimentacion guardan las mediciones electricas
   obtenidas mediante las lecturas del INA219. Posteriormente, Energia_J acumula la energia calculada,
   mientras que Potencia_Anterior y Tiempo_Energia_Anterior conservan los datos de la medicion previa para integrar
   la potencia respecto al tiempo. Suma_ADC_Reporte y Muestras_ADC_Reporte permiten acumular
   las conversiones del potenciometro y calcular su promedio. Finalmente, Desbordamientos_DMA registra los eventos en
   los que se produce un desbordamiento del buffer DMA. */

// Manejadores de los perifericos
static i2c_master_bus_handle_t Bus_I2C;
static i2c_master_dev_handle_t Dispositivo_INA219;

static adc_oneshot_unit_handle_t ADC_OneShot;
static adc_continuous_handle_t ADC_DMA;

// Variables de medicion del INA219
static float Voltaje_Alimentacion = 0;
static float Corriente_Alimentacion = 0;
static float Potencia_Alimentacion = 0;

// Variables para la energia
static double Energia_J = 0;
static float Potencia_Anterior = 0;
static int64_t Tiempo_Energia_Anterior = 0;

// Variables para el promedio del potenciometro
static uint64_t Suma_ADC_Reporte = 0;
static uint32_t Muestras_ADC_Reporte = 0;

// Contador de desbordamientos DMA
static volatile uint32_t Desbordamientos_DMA = 0;


/* 4. ESTRUCTURAS
   Este bloque define la estructura Resultados_ADC, que permite reunir en un solo conjunto los datos obtenidos
   durante cada una de las pruebas de adquisicion. Primero, Muestras_Totales almacena el numero de muestras
   registradas, mientras que Desbordamientos, Lecturas_INA y Bloques_Procesados contabilizan eventos del experimento.
   Despues, Voltaje_Promedio, Corriente_Promedio y Potencia_Promedio guardan los promedios electricos.
   Las variables Suma_Voltaje, Suma_Corriente y Suma_Potencia acumulan las lecturas utilizadas para obtener esos valores.
   Posteriormente, Energia_Total y Duracion_Efectiva almacenan la energia calculada y el tiempo real de cada prueba.
   Tiempo_Procesamiento conserva el tiempo empleado en las operaciones de adquisicion y procesamiento de muestras.
   Finalmente, se declaran tres estructuras independientes: Resultado_OneShot guarda los resultados del primer metodo,
   Resultado_DMA conserva los del segundo metodo y Resultado_Actual almacena temporalmente los resultados
   correspondientes a la prueba que se esta ejecutando. */

// Estructura para guardar los resultados de cada prueba
typedef struct
{
    uint64_t Muestras_Totales;

    uint32_t Desbordamientos;
    uint32_t Lecturas_INA;
    uint32_t Bloques_Procesados;

    double Voltaje_Promedio;
    double Corriente_Promedio;
    double Potencia_Promedio;

    double Suma_Voltaje;
    double Suma_Corriente;
    double Suma_Potencia;

    double Energia_Total;
    double Duracion_Efectiva;

    int64_t Tiempo_Procesamiento;

} Resultados_ADC;


// Resultados independientes de cada metodo
static Resultados_ADC Resultado_OneShot;
static Resultados_ADC Resultado_DMA;
static Resultados_ADC Resultado_Actual;


/* 5. PROTOTIPOS DE FUNCIONES
   Este bloque declara las funciones que se implementan posteriormente en el archivo y que son utilizadas por
   la funcion principal y por otros procedimientos. Primero, se declaran las funciones de inicializacion,
   encargadas de configurar I2C, INA219 y los controladores ADC. Despues, se indican las funciones de comunicacion con el
   INA219, que permiten escribir registros, leer registros individuales y obtener las variables electricas.
   Posteriormente, se declaran las funciones de control del experimento, encargadas de realizar las estabilizaciones
   y preparar el ADC Continuous antes de iniciar la prueba. Tambien se declaran las funciones de adquisicion One-shot
   y DMA, junto con los procedimientos de procesamiento, registro de muestras, calculo de resultados y reportes.
   Finalmente, se declara ADC_Overflow_Callback(), que recibe los datos del evento de desbordamiento del controlador
   ADC Continuous y permite contabilizar dicho evento. */

// Inicializacion
void I2C_Init(void);
void INA219_Init(void);
void ADC_OneShot_Init(void);
void ADC_DMA_Init(void);

// Funciones del INA219
void INA219_Escribir(uint8_t Registro, uint16_t Dato);
uint16_t INA219_Leer_Registro(uint8_t Registro);
void INA219_Leer(void);

// Control del experimento
void Estabilizacion(void);
void Preparar_DMA(void);
void Mostrar_Estado_Estabilizacion(int Tiempo);

// Adquisicion
void Prueba_Sin_DMA(void);
void Prueba_Con_DMA(void);

// Procesamiento
void Registrar_Muestra(int Valor_ADC);
void Actualizar_Alimentacion(void);
void Calcular_Resultados(void);

// Reportes
void Mostrar_Reporte(int Tiempo);
void Mostrar_Comparacion(void);

// Callback de desbordamiento DMA
static bool ADC_Overflow_Callback(
    adc_continuous_handle_t Handle,
    const adc_continuous_evt_data_t *Datos,
    void *Usuario
);


/* 6. FUNCION PRINCIPAL
   Este bloque contiene app_main(), que organiza la ejecucion general del experimento mediante una maquina de estados.
   Primero, se llaman I2C_Init(), INA219_Init() y ADC_OneShot_Init() para configurar los perifericos que
   se necesitan al comienzo del programa. Despues, se declara la variable Estado y se inicializa
   en cero para indicar que el experimento comienza en su primera etapa de ejecucion. El ciclo while(1) mantiene el programa funcionando
   permanentemente, mientras que switch(Estado) selecciona la funcion que corresponde ejecutar en cada etapa.
   En el estado 0 se realiza la estabilizacion inicial y se avanza al estado 1 para ejecutar la prueba One-shot.
   El estado 2 prepara el ADC Continuous y realiza su estabilizacion antes de avanzar al estado 3.
   En el estado 3 se ejecuta la prueba DMA y posteriormente se detiene y libera el controlador ADC Continuous.
   El estado 4 llama a Mostrar_Comparacion() para presentar los resultados de ambos metodos de adquisicion.
   Finalmente, el estado 5 mantiene el programa en espera mediante vTaskDelay(), evitando repetir el experimento. */

void app_main(void)
{
    // Inicializacion de los perifericos
    I2C_Init();
    INA219_Init();
    ADC_OneShot_Init();

    // Variable para controlar las etapas del experimento
    int Estado = 0;

    while (1)
    {
        switch (Estado)
        {
            case 0:

                // Estabilizacion inicial
                Estabilizacion();
                Estado = 1;
                break;

            case 1:

                // Prueba ADC One-shot
                Prueba_Sin_DMA();
                Estado = 2;
                break;

            case 2:

                // Preparacion y estabilizacion para DMA
                Preparar_DMA();
                Estado = 3;
                break;

            case 3:

                // Prueba ADC Continuous con DMA
                Prueba_Con_DMA();

                // Detencion y liberacion del ADC Continuous
                adc_continuous_stop(ADC_DMA);
                adc_continuous_deinit(ADC_DMA);
                Estado = 4;
              break;

            case 4:

                // Comparacion final de los resultados
                Mostrar_Comparacion();
                Estado = 5;
                break;

            case 5:

                // El experimento ha terminado
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
        }
    }
}


/* 7. FUNCIONES COMPLETAS
   Este apartado contiene la implementacion de las funciones declaradas anteriormente y constituye el funcionamiento
   especifico de cada periferico y etapa del experimento. Primero se encuentran los procedimientos relacionados
   con la comunicacion I2C y la configuracion del INA219. Despues se implementan las funciones de configuracion
   y adquisicion de los dos metodos de conversion del ADC. Posteriormente se desarrollan las funciones encargadas
   de estabilizar el sistema, registrar muestras, actualizar las mediciones electricas y generar reportes periodicos.
   Finalmente, se incluyen las funciones que calculan los resultados obtenidos y presentan la comparacion final
   entre ADC One-shot y ADC Continuous con DMA. */


/* 
   7.1 INICIALIZACION I2C
   Este bloque se encarga de configurar la comunicacion I2C que utiliza el ESP32 para intercambiar informacion con
   el sensor INA219. Primero se declara la estructura Configuracion_I2C, donde se establece el puerto I2C_NUM_0,
   los pines SDA y SCL, la fuente de reloj, el contador para ignorar interferencias y la habilitacion de pull-up.
   Despues, mediante i2c_new_master_bus(), se crea el bus y se guarda su manejador en la variable Bus_I2C.
   Posteriormente se declara Configuracion_INA219, donde se establece la direccion del sensor y la velocidad de
   comunicacion de 100 kHz. Finalmente, i2c_master_bus_add_device() incorpora el INA219 al bus y almacena su manejador en Dispositivo_INA219
   para utilizarlo en las siguientes funciones.
    */

void I2C_Init(void)
{
    // Configuracion del bus I2C
    i2c_master_bus_config_t Configuracion_I2C = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    i2c_new_master_bus(
        &Configuracion_I2C,
        &Bus_I2C
    );

    // Configuracion del dispositivo INA219
    i2c_device_config_t Configuracion_INA219 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DIRECCION_INA219,
        .scl_speed_hz = 100000
    };

    i2c_master_bus_add_device(
        Bus_I2C,
        &Configuracion_INA219,
        &Dispositivo_INA219
    );
}


/* 
   7.2 INICIALIZACION INA219
   Este bloque configura el sensor INA219 para comenzar amedir las variables electricas de la alimentacion.
   Primero, printf() muestra un mensaje que identifica el inicio de la configuracion del sensor en el monitor serial.
   Despues, INA219_Escribir() recibe REG_CONFIG y VALOR_CONFIG_INA219 para establecer el modo de medicion
   y los parametros de funcionamiento del dispositivo. Posteriormente, otra llamada a INA219_Escribir() utiliza
   REG_CALIBRATION y VALOR_CALIBRACION para establecer la calibracion correspondiente a la resistencia shunt de
   0.1 ohm utilizada en el sistema. Finalmente, printf() muestra el mensaje de confirmacion de configuracion del INA219.
    */

void INA219_Init(void)
{
    printf("\nInicializando INA219...\n");

    // Configuracion de medicion del sensor
    INA219_Escribir(
        REG_CONFIG,
        VALOR_CONFIG_INA219
    );

    INA219_Escribir(
        REG_CALIBRATION,
        VALOR_CALIBRACION
    );

    printf("INA219 configurado.\n");
}


/* 
   7.3 ESCRITURA DE REGISTROS INA219
   Este bloque permite enviar informacion a un registro especifico del sensor INA219 mediante comunicacion I2C.
   Primero, la funcion recibe Registro y Dato como parametros, donde Registro identifica la posicion que se modificara
   y Dato contiene el valor de 16 bits que sera transmitido. Despues, se declara el arreglo Datos con tres posiciones
   para almacenar la direccion del registro y sus dos bytes. La instruccion Datos[0] guarda el numero del registro,
   mientras que Datos[1] obtiene el byte mas significativo mediante un desplazamiento de ocho bits y una mascara.
   Posteriormente, Datos[2] conserva el byte menos significativo aplicando una mascara para seleccionar los ocho bits.
   Finalmente, i2c_master_transmit() envia los tres elementos al dispositivo identificado por Dispositivo_INA219.
    */

void INA219_Escribir(uint8_t Registro, uint16_t Dato)
{
    // Se envia el registro y posteriormente los dos bytes
    uint8_t Datos[3];

    Datos[0] = Registro;
    Datos[1] = (Dato >> 8) & 0xFF;
    Datos[2] = Dato & 0xFF;

    i2c_master_transmit(
        Dispositivo_INA219,
        Datos,
        sizeof(Datos),
        100
    );
}


/* 
   7.4 LECTURA DE REGISTROS INA219
   Este bloque permite obtener el contenido de un registro de 16 bits del sensor INA219 mediante comunicacion I2C.
   Primero, la funcion recibe Registro, que indica cual es el registro que se desea consultar en el dispositivo.
   Despues, se declara el arreglo Lectura con dos posiciones inicializadas en cero para recibir los bytes transmitidos.
   Posteriormente, i2c_master_transmit_receive() envia la direccion del registro y solicita la recepcion de dos
   bytes, que quedan almacenados en Lectura. Finalmente, la instruccion return desplaza Lectura[0]
   ocho posiciones hacia la izquierda y combina ese resultado con Lectura[1] mediante el operador OR, reconstruyendo
   el valor completo de 16 bits que devuelve la funcion.
    */

uint16_t INA219_Leer_Registro(uint8_t Registro)
{
    // Lectura de los dos bytes del registro seleccionado
    uint8_t Lectura[2] = {0, 0};

    i2c_master_transmit_receive(
        Dispositivo_INA219,
        &Registro,
        1,
        Lectura,
        2,
        100
    );

    return ((uint16_t)Lectura[0] << 8) | Lectura[1];
}


/* 
   7.5 LECTURA DE VOLTAJE, CORRIENTE Y POTENCIA
   Este bloque obtiene las tres variables electricas medidas por el INA219 y las convierte a unidades fisicas.
   Primero se declaran Dato_Voltaje, Dato_Corriente y Dato_Potencia, que almacenaran los valores digitales
   recibidos desde los registros internos del sensor. Despues, se llama tres veces a INA219_Leer_Registro(),
   utilizando REG_BUS_VOLTAGE, REG_CURRENT y REG_POWER para obtener los datos de voltaje, corriente y potencia.
   Posteriormente, Dato_Voltaje se desplaza tres bits a la derecha y se multiplica por 0.004 para obtener el voltaje.
   Dato_Corriente se interpreta como un entero con signo y se multiplica por CURRENT_LSB para convertirlo a amperios.
   Finalmente, Dato_Potencia se multiplica por POWER_LSB y los tres resultados se almacenan en las variables globales
   Voltaje_Alimentacion, Corriente_Alimentacion y Potencia_Alimentacion para utilizarlos en los reportes.
    */

void INA219_Leer(void)
{
    uint16_t Dato_Voltaje;
    uint16_t Dato_Corriente;
    uint16_t Dato_Potencia;

    // Lectura de los registros del INA219
    Dato_Voltaje = INA219_Leer_Registro(REG_BUS_VOLTAGE);
    Dato_Corriente = INA219_Leer_Registro(REG_CURRENT);
    Dato_Potencia = INA219_Leer_Registro(REG_POWER);

    // Conversion de los valores a unidades fisicas
    Voltaje_Alimentacion =
        ((Dato_Voltaje >> 3) * 0.004);

    Corriente_Alimentacion =
        ((int16_t)Dato_Corriente * CURRENT_LSB);

    Potencia_Alimentacion =
        (Dato_Potencia * POWER_LSB);
}


/* 
   7.6 INICIALIZACION ADC ONE-SHOT
   Este bloque configura el ADC One-shot para realizar conversiones analogicas solicitadas por el programa.
   Primero, printf() muestra el inicio de la configuracion y se declara Configuracion_ADC para seleccionar la unidad
   ADC definida en UNIDAD_ADC y deshabilitar el modo ULP.Despues, adc_oneshot_new_unit() crea el controlador ADC
   y almacena su manejador en la variable ADC_OneShot. Posteriormente, se declara Configuracion_Canal para
   establecer la atenuacion y resolucion de conversion. La funcion adc_oneshot_config_channel() utiliza esa
   estructura junto con ADC_OneShot y CANAL_ADC para configurar el canal conectado al potenciometro.
   Finalmente, printf() indica que la configuracion del ADC One-shot ha terminado.
   */

void ADC_OneShot_Init(void)
{
    printf("Inicializando ADC One-shot...\n");

    // Configuracion de la unidad ADC
    adc_oneshot_unit_init_cfg_t Configuracion_ADC = {
        .unit_id = UNIDAD_ADC,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };

    adc_oneshot_new_unit(
        &Configuracion_ADC,
        &ADC_OneShot
    );

    // Configuracion del canal del potenciometro
    adc_oneshot_chan_cfg_t Configuracion_Canal = {
        .atten = ATENUACION_ADC,
        .bitwidth = RESOLUCION_ADC
    };

    adc_oneshot_config_channel(
        ADC_OneShot,
        CANAL_ADC,
        &Configuracion_Canal
    );

    printf("ADC One-shot configurado.\n");
}


/* 
   7.7 INICIALIZACION ADC CONTINUOUS CON DMA
   Este bloque configura el ADC Continuous para realizar conversiones continuas y transferir los datos mediante DMA.
   Primero, printf() anuncia el inicio de la configuracion y se declara Configuracion_Manejador para establecer el
   espacio maximo de almacenamiento y el tamano de cada bloque. Despues, adc_continuous_new_handle() crea el controlador
   y guarda su manejador en ADC_DMA. Posteriormente, Patron_ADC define la atenuacion, el canal,
   la unidad y la resolucion que utilizara cada conversion. Con esos parametros se declara Configuracion_Continua,
   donde sample_freq_hz establece la frecuencia de muestreo, conv_mode define el modo de conversion y format determina
el formato de los datos digitales recibidos. La instruccion adc_continuous_config() aplica esta
   configuracion al controlador almacenado en ADC_DMA. Finalmente, Callbacks_ADC registra ADC_Overflow_Callback()
   mediante adc_continuous_register_event_callbacks(), para contabilizar los desbordamientos que ocurran en el buffer.
   */

void ADC_DMA_Init(void)
{
    printf("\nConfigurando ADC Continuous con DMA...\n");

    // Configuracion del manejador DMA
    adc_continuous_handle_cfg_t Configuracion_Manejador = {
        .max_store_buf_size = TAMANO_POOL_ADC,
        .conv_frame_size = TAMANO_BUFFER_ADC
    };

    adc_continuous_new_handle(
        &Configuracion_Manejador,
        &ADC_DMA
    );

    // Configuracion del patron de conversion
    adc_digi_pattern_config_t Patron_ADC = {
        .atten = ATENUACION_ADC,
        .channel = CANAL_ADC,
        .unit = UNIDAD_ADC,
        .bit_width = RESOLUCION_ADC
    };

    // Configuracion general de adquisicion continua
    adc_continuous_config_t Configuracion_Continua = {
        .sample_freq_hz = FRECUENCIA_ADC,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        .pattern_num = 1,
        .adc_pattern = &Patron_ADC
    };

    adc_continuous_config(
        ADC_DMA,
        &Configuracion_Continua
    );

    // Registro del callback de desbordamiento
    adc_continuous_evt_cbs_t Callbacks_ADC = {
        .on_pool_ovf = ADC_Overflow_Callback
    };

    adc_continuous_register_event_callbacks(
        ADC_DMA,
        &Callbacks_ADC,
        NULL
    );

    printf("ADC Continuous configurado.\n");
}


/* 
   7.8 CALLBACK DE DESBORDAMIENTO DMA
   Este bloque define la funcion que se ejecuta cuando el controlador ADC Continuous notifica un desbordamiento.
   Primero, se reciben Handle, Datos y Usuario como parametros proporcionados por el sistema de eventos del controlador.
   Handle identifica el controlador ADC, Datos contiene la informacion del evento y Usuario permite recibir un dato
   adicional asociado al registro del callback. El atributo IRAM_ATTR indica que la funcion puede ejecutarse
   desde memoria interna, como se requiere para este callback. Despues, la instruccion Desbordamientos_DMA++ incrementa
   el contador global cada vez que ocurre el evento. Finalmente, return false indica que el callback no solicita
   una tarea de mayor prioridad para ser ejecutada al salir.
   */

static bool IRAM_ATTR ADC_Overflow_Callback(
    adc_continuous_handle_t Handle,
    const adc_continuous_evt_data_t *Datos,
    void *Usuario
)
{
    // Se incrementa el contador cuando el buffer se desborda
    Desbordamientos_DMA++;

    return false;
}


/* 
   7.9 REPORTE DURANTE LAS ESTABILIZACIONES
   Este bloque obtiene y presenta las mediciones actuales mientras el sistema permanece en una etapa de estabilizacion.
   Primero, se declaran Valor_ADC, Voltaje_Potenciometro y Tiempo_Restante para almacenar la lectura analogica, su
   conversion aproximada a voltaje y el tiempo pendiente. Despues, adc_oneshot_read() solicita una conversion del
   canal CANAL_ADC y almacena el resultado en Valor_ADC. Posteriormente, se divide el valor ADC entre
   VALOR_ADC_MAXIMO y se multiplica por VOLTAJE_REFERENCIA para calcular Voltaje_Potenciometro.
   La funcion INA219_Leer() obtiene las variables electricas y actualiza las variables globales correspondientes.
   Finalmente, las instrucciones printf() muestran el tiempo transcurrido, el tiempo restante, la lectura ADC, el
   voltaje estimado del potenciometro y las mediciones del INA219 en el monitor serial.
    */

void Mostrar_Estado_Estabilizacion(int Tiempo)
{
    int Valor_ADC = 0;
    double Voltaje_Potenciometro;
    int Tiempo_Restante = TIEMPO_ESTABILIZACION - Tiempo;

    // Lectura del potenciometro mediante ADC One-shot
    adc_oneshot_read(
        ADC_OneShot,
        CANAL_ADC,
        &Valor_ADC
    );

    Voltaje_Potenciometro =
        ((double)Valor_ADC / VALOR_ADC_MAXIMO)
        * VOLTAJE_REFERENCIA;

    // Lectura de las variables electricas del INA219
    INA219_Leer();

    // Reporte del tiempo y de las mediciones actuales
    printf("\nTiempo de estabilizacion: %d/%d s\n",
           Tiempo,
           TIEMPO_ESTABILIZACION);

    printf("Tiempo restante: %d s\n",
           Tiempo_Restante);

    printf("ADC potenciometro: %d\n",
           Valor_ADC);

    printf("Voltaje potenciometro estimado: %.3f V\n",
           Voltaje_Potenciometro);

    printf("Voltaje alimentacion: %.3f V\n",
           Voltaje_Alimentacion);

    printf("Corriente: %.3f mA\n",
           Corriente_Alimentacion * 1000.0);

    printf("Potencia: %.3f mW\n",
           Potencia_Alimentacion * 1000.0);
}


/* 
   7.10 ESTABILIZACION INICIAL
   Este bloque mantiene el sistema durante el tiempo de estabilizacion establecido en TIEMPO_ESTABILIZACION.
   Primero, se declara la variable Tiempo, que funciona como contador de los segundos transcurridos en esta etapa.
   Despues, printf() presenta el encabezado que identifica el comienzo de la estabilizacion inicial en el monitor.
   Posteriormente, el ciclo for comienza en el segundo uno y se repite mientras Tiempo sea menor o igual al tiempo
   definido en TIEMPO_ESTABILIZACION. En cada repeticion, vTaskDelay() introduce una espera de
   1000 milisegundos y posteriormente se llama a Mostrar_Estado_Estabilizacion() para realizar las lecturas
   del potenciometro y del INA219. Finalmente, cuando el contador alcanza el tiempo definido,
   termina el ciclo y la funcion permite continuar con la siguiente etapa de la maquina de estados.
    */

void Estabilizacion(void)
{
    int Tiempo;

    printf("\n========================================\n");
    printf("       ESTABILIZACION INICIAL\n");
    printf("========================================\n");

    // Mostrar las mediciones una vez por segundo
    for (Tiempo = 1;
         Tiempo <= TIEMPO_ESTABILIZACION;
         Tiempo++)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        Mostrar_Estado_Estabilizacion(Tiempo);
    }
}


/* 
   7.11 PREPARACION Y ESTABILIZACION PARA DMA
   Este bloque prepara el sistema para ejecutar la segunda prueba utilizando ADC Continuous con transferencia DMA.
   Primero, se declara Tiempo para contar los segundos de estabilizacion que se realizan antes de iniciar la prueba.
   Despues, printf() muestra el encabezado de esta etapa y memset() reinicia todos los campos de Resultado_Actual.
   Tambien se ponen en cero los acumuladores del potenciometro, las variables de energia y el contador de desbordamientos.
   Posteriormente, el ciclo for mantiene una estabilizacion de TIEMPO_ESTABILIZACION segundos, utilizando vTaskDelay()
   y Mostrar_Estado_Estabilizacion() para presentar las mediciones actuales durante ese periodo.
   Una vez finalizado el ciclo, adc_oneshot_del_unit() libera el controlador One-shot y ADC_OneShot se establece en NULL.
   Finalmente, ADC_DMA_Init() configura el nuevo controlador que se utilizara durante la adquisicion continua con DMA.
    */

void Preparar_DMA(void)
{
    int Tiempo;

    printf("\n========================================\n");
    printf("       ESTABILIZACION PARA DMA\n");
    printf("========================================\n");

    // Reinicio de las variables de adquisicion
    memset(
        &Resultado_Actual,
        0,
        sizeof(Resultado_Actual)
    );

    Suma_ADC_Reporte = 0;
    Muestras_ADC_Reporte = 0;

    Energia_J = 0;
    Potencia_Anterior = 0;
    Tiempo_Energia_Anterior = 0;

    Desbordamientos_DMA = 0;

    // Mostrar las mediciones una vez por segundo
    for (Tiempo = 1;
         Tiempo <= TIEMPO_ESTABILIZACION;
         Tiempo++)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        Mostrar_Estado_Estabilizacion(Tiempo);
    }

    // Liberar One-shot despues de terminar la estabilizacion
    adc_oneshot_del_unit(ADC_OneShot);
    ADC_OneShot = NULL;

    // Configurar el ADC Continuous con DMA
    ADC_DMA_Init();
}


/* 
   7.12 REGISTRO DE MUESTRAS ADC
   Este bloque centraliza el registro de las muestras que se obtienen mediante cualquiera de los dos metodos ADC.
   Primero, la funcion recibe Valor_ADC como parametro, que contiene el resultado digital de una conversion.
   Despues, Resultado_Actual.Muestras_Totales se incrementa para contabilizar una nueva muestra procesada.
   Posteriormente, el valor recibido se suma a Suma_ADC_Reporte y se incrementa Muestras_ADC_Reporte para mantener el
   acumulado y el numero de muestras del intervalo actual. Finalmente, estos dos acumuladores permiten que
   Mostrar_Reporte() calcule el promedio del potenciometro correspondiente al intervalo de reporte.
    */

void Registrar_Muestra(int Valor_ADC)
{
    // Acumulacion de muestras para calcular el promedio
    Resultado_Actual.Muestras_Totales++;

    Suma_ADC_Reporte += Valor_ADC;
    Muestras_ADC_Reporte++;
}


/* 
   7.13 ACTUALIZACION DE LAS MEDICIONES INA219
   Este bloque realiza una nueva lectura del sensor INA219, acumula las variables electricas y calcula la energia.
   Primero, se declara Tiempo_Actual para guardar el instante en que se realiza la medicion mediante el temporizador.
   Despues, INA219_Leer() actualiza el voltaje, corriente y potencia almacenados en las variables globales.
   Posteriormente, estos tres valores se agregan a sus respectivos acumuladores dentro de Resultado_Actual y
   Lecturas_INA se incrementa para contabilizar la lectura. La funcion esp_timer_get_time() obtiene el tiempo actual
   en microsegundos para calcular el intervalo transcurrido. Si Tiempo_Energia_Anterior es diferente de cero, se calcula
   Diferencia_Tiempo en segundos y se integra la potencia mediante el promedio entre la potencia anterior y actual.
   Finalmente, Potencia_Anterior y Tiempo_Energia_Anterior se actualizan para utilizarlos en la siguiente medicion.
    */

void Actualizar_Alimentacion(void)
{
    int64_t Tiempo_Actual;

    // Lectura de las variables electricas
    INA219_Leer();

    Resultado_Actual.Suma_Voltaje +=
        Voltaje_Alimentacion;

    Resultado_Actual.Suma_Corriente +=
        Corriente_Alimentacion;

    Resultado_Actual.Suma_Potencia +=
        Potencia_Alimentacion;

    Resultado_Actual.Lecturas_INA++;

    // Integracion de potencia para calcular energia
    Tiempo_Actual = esp_timer_get_time();

    if (Tiempo_Energia_Anterior != 0)
    {
        double Diferencia_Tiempo =
            (Tiempo_Actual - Tiempo_Energia_Anterior)
            / 1000000.0;

        Energia_J +=
            ((Potencia_Anterior + Potencia_Alimentacion) / 2.0)
            * Diferencia_Tiempo;
    }

    Potencia_Anterior = Potencia_Alimentacion;
    Tiempo_Energia_Anterior = Tiempo_Actual;
}


/* 
   7.14 REPORTE PERIODICO DEL EXPERIMENTO
   Este bloque calcula el promedio de las muestras ADC registradas durante el intervalo y presenta los resultados.
   Primero, se declara Promedio_ADC y se inicializa en cero para almacenar el promedio del potenciometro.
   Despues, la condicion if comprueba que Muestras_ADC_Reporte sea mayor que cero, evitando realizar una division entre
   cero cuando no existan muestras registradas.Posteriormente, se divide Suma_ADC_Reporte entre
   Muestras_ADC_Reporte y se almacena el resultado en Promedio_ADC.
   La instruccion printf() presenta el tiempo del reporte, el total de muestras, el promedio ADC y las mediciones
   actuales de voltaje, corriente, potencia y energia. Finalmente, Suma_ADC_Reporte y Muestras_ADC_Reporte se
   reinician para comenzar a acumular las muestras del siguiente intervalo de reporte.
    */

void Mostrar_Reporte(int Tiempo)
{
    double Promedio_ADC = 0;

    // Calculo del promedio de las muestras del intervalo
    if (Muestras_ADC_Reporte > 0)
    {
        Promedio_ADC =
            (double)Suma_ADC_Reporte / Muestras_ADC_Reporte;
    }

    printf("%5d | %10" PRIu64 " | %8.0f | %6.3f | %8.3f | %8.3f | %8.4f\n",
           Tiempo,
           Resultado_Actual.Muestras_Totales,
           Promedio_ADC,
           Voltaje_Alimentacion,
           Corriente_Alimentacion * 1000.0,
           Potencia_Alimentacion * 1000.0,
           Energia_J);

    // Reinicio del promedio del potenciometro para el siguiente intervalo
    Suma_ADC_Reporte = 0;
    Muestras_ADC_Reporte = 0;
}


/* 
   7.15 PRUEBA ADC ONE-SHOT
   Este bloque realiza la primera adquisicion del experimento utilizando el controlador ADC One-shot, que solicita cada
   conversion mediante una llamada directa del programa. Primero, se declaran las variables para almacenar la
   lectura ADC, los tiempos de inicio, ejecucion y finalizacion, los intervalos de reporte e INA219 y el tiempo de proceso.
   Despues, Tiempo_Mostrar establece el primer segundo que aparecera en la tabla y memset() reinicia Resultado_Actual.
   Tambien se reinician los acumuladores de muestras y energia para comenzar la prueba sin conservar datos anteriores.
   Posteriormente, printf() presenta el encabezado de la tabla y esp_timer_get_time() establece el instante inicial.
   Con ese valor se calculan Tiempo_Reporte y Tiempo_INA, que determinan cuando deben realizarse las siguientes
   actualizaciones del sensor y los reportes por UART.
   Dentro del ciclo while se compara el tiempo transcurrido con TIEMPO_PRUEBA para mantener la adquisicion durante
   los 120 segundos establecidos. En cada iteracion se obtiene el tiempo inicial de proceso, adc_oneshot_read() solicita una conversion del canal ADC
   y Registrar_Muestra() contabiliza el valor obtenido. Despues se acumula el tiempo de procesamiento y se
   incrementa Bloques_Procesados para registrar cada ciclo. La condicion relacionada con Tiempo_INA ejecuta
   Actualizar_Alimentacion() cada segundo, mientras que Tiempo_Reporte permite llamar a Mostrar_Reporte() cada
   cinco segundos y avanzar los intervalos correspondientes. Finalmente, al terminar el ciclo se registra la duracion
   efectiva, se llama a Calcular_Resultados() y se copia Resultado_Actual en Resultado_OneShot para conservar
   los resultados de esta primera prueba.
    */

void Prueba_Sin_DMA(void)
{
    int Valor_ADC = 0;

    int64_t Tiempo_Inicio;
    int64_t Tiempo_Actual;
    int64_t Tiempo_Fin;

    int64_t Tiempo_Reporte;
    int64_t Tiempo_INA;

    int64_t Tiempo_Procesamiento_Inicio;

    int Tiempo_Mostrar = 5;

    // Reinicio de resultados y acumuladores
    memset(
        &Resultado_Actual,
        0,
        sizeof(Resultado_Actual)
    );

    Suma_ADC_Reporte = 0;
    Muestras_ADC_Reporte = 0;

    Energia_J = 0;
    Potencia_Anterior = 0;
    Tiempo_Energia_Anterior = 0;

    // Inicio de la prueba
    printf("\n========== TABLA ADC ONE-SHOT SIN DMA  ==========\n");
    printf("Duracion programada: %d s\n", TIEMPO_PRUEBA);
    printf(" Tiempo |  Muestras | ADC prom. | V (V)   | I(mA)   | P(mW)   | E(J)\n");
    printf("--------+-----------+-----------+---------+---------+---------+-----\n");

    Tiempo_Inicio = esp_timer_get_time();

    Tiempo_Reporte =
        Tiempo_Inicio + INTERVALO_REPORTE;

    Tiempo_INA =
        Tiempo_Inicio + INTERVALO_INA;

    // Adquisicion continua mediante conversiones solicitadas por software
    while ((esp_timer_get_time() - Tiempo_Inicio)
           < (TIEMPO_PRUEBA * 1000000LL))
    {
        Tiempo_Procesamiento_Inicio = esp_timer_get_time();

        adc_oneshot_read(
            ADC_OneShot,
            CANAL_ADC,
            &Valor_ADC
        );

        Registrar_Muestra(Valor_ADC);

        Resultado_Actual.Tiempo_Procesamiento +=
            esp_timer_get_time() - Tiempo_Procesamiento_Inicio;

        Resultado_Actual.Bloques_Procesados++;

        Tiempo_Actual = esp_timer_get_time();

        // Lectura del INA219 cada segundo
        if (Tiempo_Actual >= Tiempo_INA)
        {
            Actualizar_Alimentacion();

            Tiempo_INA += INTERVALO_INA;
        }

        // Reporte por UART cada cinco segundos
        if (Tiempo_Actual >= Tiempo_Reporte)
        {
            Mostrar_Reporte(Tiempo_Mostrar);

            Tiempo_Reporte += INTERVALO_REPORTE;
            Tiempo_Mostrar += 5;
        }
    }

    // Registrar la duracion efectiva de la prueba
    Tiempo_Fin = esp_timer_get_time();

    Resultado_Actual.Duracion_Efectiva =
        (Tiempo_Fin - Tiempo_Inicio) / 1000000.0;

    // Calculo de los resultados de One-shot
    Calcular_Resultados();

    Resultado_OneShot = Resultado_Actual;

    printf("\nPRUEBA SIN DMA FINALIZADA.\n");
}


/* 
   7.16 PRUEBA ADC CONTINUOUS CON DMA
   Este bloque realiza la segunda adquisicion utilizando
   ADC Continuous, donde las conversiones se transfieren
   mediante DMA hacia un buffer de almacenamiento.
   Primero, se declara Buffer_ADC para recibir los datos,
   Longitud_Leida para conocer cuantos bytes se recibieron
   e Indice para recorrer las posiciones del buffer.
   Tambien se declaran las variables de tiempo utilizadas
   para controlar la duracion, los reportes y las lecturas INA.
   Despues, memset() reinicia Resultado_Actual y se ponen
   en cero los acumuladores de muestras, energia y
   desbordamientos para iniciar esta prueba independientemente.
   Posteriormente, printf() presenta el encabezado de la tabla
   y adc_continuous_start() inicia las conversiones continuas.
   esp_timer_get_time() establece el tiempo inicial y permite
   calcular los instantes de los siguientes reportes y
   actualizaciones del INA219.
   Dentro del ciclo while se comprueba que no se haya
   alcanzado la duracion definida en TIEMPO_PRUEBA.
   En cada iteracion, Longitud_Leida se reinicia y
   adc_continuous_read() solicita los datos disponibles
   para almacenarlos en Buffer_ADC.
   Si Longitud_Leida es mayor que cero, comienza el
   procesamiento del bloque recibido y el ciclo for recorre
   el buffer avanzando segun SOC_ADC_DIGI_RESULT_BYTES.
   En cada posicion se interpreta la informacion mediante
   adc_digi_output_data_t y se obtiene el valor digital de
   12 bits desde Dato_ADC->type1.data.
   Registrar_Muestra() contabiliza cada conversion recibida,
   mientras que Tiempo_Procesamiento y Bloques_Procesados
   registran el tiempo y la cantidad de bloques procesados.
   Posteriormente, las condiciones if actualizan el INA219
   cada segundo y muestran los reportes cada cinco segundos.
   Finalmente, se calcula la duracion efectiva, se ejecuta
   Calcular_Resultados(), se guarda el contador de
   desbordamientos y se copia la estructura en Resultado_DMA.
    */

void Prueba_Con_DMA(void)
{
    uint8_t Buffer_ADC[TAMANO_BUFFER_ADC];

    uint32_t Longitud_Leida = 0;

    uint32_t Indice;

    int64_t Tiempo_Inicio;
    int64_t Tiempo_Actual;
    int64_t Tiempo_Fin;

    int64_t Tiempo_Reporte;
    int64_t Tiempo_INA;

    int64_t Tiempo_Procesamiento_Inicio;

    int Tiempo_Mostrar = 5;

    // Reinicio de resultados y acumuladores
    memset(
        &Resultado_Actual,
        0,
        sizeof(Resultado_Actual)
    );

    Suma_ADC_Reporte = 0;
    Muestras_ADC_Reporte = 0;

    Energia_J = 0;
    Potencia_Anterior = 0;
    Tiempo_Energia_Anterior = 0;

    Desbordamientos_DMA = 0;

    // Inicio de la prueba
    printf("\n========== TABLA ADC  CON DMA ==========\n");
    printf("Frecuencia configurada: %d Hz | Duracion: %d s\n",
           FRECUENCIA_ADC, TIEMPO_PRUEBA);
    printf(" Tiempo |  Muestras | ADC prom. | V(V) | I(mA)   | P(mW)   | E(J)\n");
    printf("--------+-----------+-----------+---------+---------+---------+---------\n");

    // Inicio de la adquisicion continua
    adc_continuous_start(ADC_DMA);

    Tiempo_Inicio = esp_timer_get_time();

    Tiempo_Reporte =
        Tiempo_Inicio + INTERVALO_REPORTE;

    Tiempo_INA =
        Tiempo_Inicio + INTERVALO_INA;

    // Lectura y procesamiento de los bloques transferidos por DMA
    while ((esp_timer_get_time() - Tiempo_Inicio)
           < (TIEMPO_PRUEBA * 1000000LL))
    {
        Longitud_Leida = 0;

        adc_continuous_read(
            ADC_DMA,
            Buffer_ADC,
            TAMANO_BUFFER_ADC,
            &Longitud_Leida,
            100
        );

        // Procesamiento de las conversiones recibidas
        if (Longitud_Leida > 0)
        {
            Tiempo_Procesamiento_Inicio = esp_timer_get_time();

            for (Indice = 0;
                 Indice < Longitud_Leida;
                 Indice += SOC_ADC_DIGI_RESULT_BYTES)
            {
                adc_digi_output_data_t *Dato_ADC =
                    (adc_digi_output_data_t *)&Buffer_ADC[Indice];

                // Lectura del valor digital de 12 bits
                int Valor_ADC = Dato_ADC->type1.data;

                Registrar_Muestra(Valor_ADC);
            }

            Resultado_Actual.Tiempo_Procesamiento +=
                esp_timer_get_time() - Tiempo_Procesamiento_Inicio;

            Resultado_Actual.Bloques_Procesados++;
        }

        Tiempo_Actual = esp_timer_get_time();

        // Lectura del INA219 cada segundo
        if (Tiempo_Actual >= Tiempo_INA)
        {
            Actualizar_Alimentacion();

            Tiempo_INA += INTERVALO_INA;
        }

        // Reporte por UART cada cinco segundos
        if (Tiempo_Actual >= Tiempo_Reporte)
        {
            Mostrar_Reporte(Tiempo_Mostrar);

            Tiempo_Reporte += INTERVALO_REPORTE;
            Tiempo_Mostrar += 5;
        }
    }

    // Registrar la duracion efectiva de la prueba
    Tiempo_Fin = esp_timer_get_time();

    Resultado_Actual.Duracion_Efectiva =
        (Tiempo_Fin - Tiempo_Inicio) / 1000000.0;

    // Calculo de los resultados de DMA
    Calcular_Resultados();

    Resultado_Actual.Desbordamientos =
        Desbordamientos_DMA;

    Resultado_DMA = Resultado_Actual;

    printf("\nDesbordamientos DMA: %" PRIu32 "\n",
           Resultado_DMA.Desbordamientos);

    printf("PRUEBA CON DMA FINALIZADA.\n");
}


/* 
   7.17 CALCULO DE RESULTADOS
   Este bloque obtiene los promedios de las variables
   electricas y almacena la energia acumulada durante la prueba.
   Primero, la condicion if comprueba que Lecturas_INA sea
   mayor que cero para determinar si existen mediciones
   electricas que puedan utilizarse en los calculos.
   Despues, se divide Suma_Voltaje entre Lecturas_INA para
   obtener Voltaje_Promedio dentro de Resultado_Actual.
   De la misma manera, Suma_Corriente y Suma_Potencia se
   dividen entre el numero de lecturas para obtener sus
   respectivos promedios electricos.
   Finalmente, Energia_J se asigna a Energia_Total para
   conservar la energia calculada mediante la integracion
   de potencia durante el desarrollo del experimento.
    */

void Calcular_Resultados(void)
{
    // Calculo de los promedios electricos
    if (Resultado_Actual.Lecturas_INA > 0)
    {
        Resultado_Actual.Voltaje_Promedio =
            Resultado_Actual.Suma_Voltaje /
            Resultado_Actual.Lecturas_INA;

        Resultado_Actual.Corriente_Promedio =
            Resultado_Actual.Suma_Corriente /
            Resultado_Actual.Lecturas_INA;

        Resultado_Actual.Potencia_Promedio =
            Resultado_Actual.Suma_Potencia /
            Resultado_Actual.Lecturas_INA;
    }

    // Calculo de energia acumulada
    Resultado_Actual.Energia_Total = Energia_J;
}


/* 
   7.18 COMPARACION FINAL
   Este bloque presenta una comparacion entre los resultados
   obtenidos mediante ADC One-shot y ADC Continuous con DMA.
   Primero, se declaran Tiempo_Bloque_OneShot y Tiempo_Bloque_DMA
   para almacenar el tiempo promedio de procesamiento por bloque
   correspondiente a cada metodo de adquisicion.
   Despues, las condiciones if comprueban que cada prueba
   haya procesado al menos un bloque antes de calcular el
   tiempo promedio correspondiente.
   Para cada metodo, Tiempo_Procesamiento se divide entre
   Bloques_Procesados y posteriormente entre 1000 para
   convertir el resultado de microsegundos a milisegundos.
   Finalmente, las instrucciones printf() presentan una
   tabla que compara la duracion efectiva, los promedios
   electricos, la energia, las muestras registradas,
   el tiempo de procesamiento y los desbordamientos DMA.
   Los valores de corriente y potencia se multiplican por
   1000 para mostrarlos en miliamperios y milivatios.
   En las metricas que no aplican al metodo One-shot se
   muestra el texto correspondiente en lugar de un valor.
    */

void Mostrar_Comparacion(void)
{
    double Tiempo_Bloque_OneShot = 0;
    double Tiempo_Bloque_DMA = 0;

    // Calculo del tiempo promedio de procesamiento por bloque
    if (Resultado_OneShot.Bloques_Procesados > 0)
    {
        Tiempo_Bloque_OneShot =
            (double)Resultado_OneShot.Tiempo_Procesamiento /
            Resultado_OneShot.Bloques_Procesados / 1000.0;
    }

    if (Resultado_DMA.Bloques_Procesados > 0)
    {
        Tiempo_Bloque_DMA =
            (double)Resultado_DMA.Tiempo_Procesamiento /
            Resultado_DMA.Bloques_Procesados / 1000.0;
    }

    // Tabla comparativa de los resultados finales
    printf("\n================ COMPARACION FINAL ================\n");
    printf("Metrica                              |       One-Shot |            DMA\n");
    printf("-------------------------------------+----------------+----------------\n");

    printf("Duracion efectiva (s)                | %14.3f | %14.3f\n",
           Resultado_OneShot.Duracion_Efectiva,
           Resultado_DMA.Duracion_Efectiva);

    printf("Voltaje promedio (V)                 | %14.3f | %14.3f\n",
           Resultado_OneShot.Voltaje_Promedio,
           Resultado_DMA.Voltaje_Promedio);

    printf("Corriente promedio (mA)              | %14.3f | %14.3f\n",
           Resultado_OneShot.Corriente_Promedio * 1000.0,
           Resultado_DMA.Corriente_Promedio * 1000.0);

    printf("Potencia promedio (mW)               | %14.3f | %14.3f\n",
           Resultado_OneShot.Potencia_Promedio * 1000.0,
           Resultado_DMA.Potencia_Promedio * 1000.0);

    printf("Energia acumulada (J)                | %14.4f | %14.4f\n",
           Resultado_OneShot.Energia_Total,
           Resultado_DMA.Energia_Total);

    printf("Muestras validas                     | %14" PRIu64 " | %14" PRIu64 "\n",
           Resultado_OneShot.Muestras_Totales,
           Resultado_DMA.Muestras_Totales);

    printf("Tiempo procesamiento por bloque (ms)| %14.6f | %14.6f\n",
           Tiempo_Bloque_OneShot,
           Tiempo_Bloque_DMA);

    printf("Desbordamientos DMA                  | %14s | %14" PRIu32 "\n",
           "No aplica",
           Resultado_DMA.Desbordamientos);
}
