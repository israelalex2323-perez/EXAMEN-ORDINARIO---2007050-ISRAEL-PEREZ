
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"

#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "soc/soc_caps.h"

/* =========================================================
   CONFIGURACION I2C E INA219
   ========================================================= */

/*
En esta parte se establecen los valores que se van a utilizar para
la comunicación entre el ESP32 y el sensor INA219. Se seleccionan
los pines GPIO 21 y 22 para las conexiones SDA y SCL, respectivamente,
que permiten el intercambio de información entre los dos dispositivos.
También se define una velocidad de comunicación de 100 kHz y la
dirección 0x40, que es la que utiliza el sensor para ser identificado.

Por último, se guardan las direcciones de los registros internos del
INA219. Estos registros permiten enviarle configuraciones al sensor
y consultar los valores que obtiene durante las mediciones de voltaje
y corriente.
*/

#define I2C_SDA_GPIO        21
#define I2C_SCL_GPIO        22
#define I2C_FREQUENCY       100000

#define INA219_ADDRESS      0x40
#define INA219_REG_CONFIG   0x00
#define INA219_REG_BUSV     0x02
#define INA219_REG_CURRENT  0x04
#define INA219_REG_CALIB    0x05

/* =========================================================
   CONFIGURACION ADC
   ========================================================= */

/*
Aquí se definen los parámetros que se utilizarán para obtener las
lecturas del potenciómetro conectado al GPIO34 del ESP32. Este pin
corresponde al canal ADC1_CH6, por lo que se selecciona ese canal
para convertir el voltaje de entrada en valores digitales.

También se establecen los valores que se utilizarán durante la prueba
con DMA. Se configura una frecuencia de 20,000 muestras por segundo,
pero se utiliza una decimación de 2, es decir, se conserva una muestra
por cada dos que se reciben. Con esto se obtiene una frecuencia efectiva
esperada de 10,000 muestras por segundo.

Finalmente, se define el espacio destinado a almacenar los datos
recibidos y el tamaño de los bloques que se van a procesar durante
la adquisición.
*/

#define ADC_CHANNEL         ADC_CHANNEL_6
#define ADC_ATTEN           ADC_ATTEN_DB_12

#define FRECUENCIA_DMA      20000
#define DECIMACION          2
#define FRECUENCIA_EFECTIVA (FRECUENCIA_DMA / DECIMACION)

#define BUFFER_DMA          4096
#define TAMANO_BLOQUE       256

// En el método One-shot se realizan 10 lecturas individuales por cada bloque.
#define MUESTRAS_POR_BLOQUE 10

/* =========================================================
   CONFIGURACION DE LA PRUEBA
   ========================================================= */

/*
Se establecen los tiempos que se utilizarán para realizar las pruebas.
Cada método tendrá una duración de 120 segundos, lo que permite
obtener una cantidad considerable de muestras y mediciones eléctricas.

Antes de comenzar cada prueba se deja un periodo de estabilización
de 10 segundos, durante el cual se realizan lecturas del INA219 para
observar el comportamiento inicial del sistema antes de comenzar
a registrar los resultados correspondientes a la evaluación.
*/

#define DURACION_PRUEBA     120000000LL
#define ESTABILIZACION_S    10

/* =========================================================
   VARIABLES GLOBALES
   ========================================================= */

/*
En esta sección se declaran las variables que se necesitan en
diferentes partes del programa. Los identificadores de I2C permiten
mantener la comunicación con el INA219, mientras que la variable
ADC_OneShot guarda la referencia de la unidad ADC utilizada para
realizar las conversiones individuales.

También se tienen variables para conservar las últimas lecturas
de voltaje, corriente y potencia obtenidas del sensor. Por otra parte,
se utiliza Desbordamientos_DMA como contador de las ocasiones en
que el almacenamiento de datos del DMA llega a su límite.

Estas variables se mantienen disponibles para que las distintas
funciones puedan consultar o actualizar sus valores durante las pruebas.
*/

static i2c_master_bus_handle_t Bus_I2C;
static i2c_master_dev_handle_t INA219;

static adc_oneshot_unit_handle_t ADC_OneShot;

static float Voltaje_Lectura = 0;
static float Corriente_Lectura = 0;
static float Potencia_Lectura = 0;

static volatile uint32_t Desbordamientos_DMA = 0;

/* =========================================================
   ESTRUCTURA DE RESULTADOS
   ========================================================= */

/*
Se crea la estructura Resultados_t para mantener organizados los
datos que se obtienen durante cada prueba. En lugar de declarar
muchas variables independientes, se agrupan en una sola estructura
las mediciones eléctricas, los valores obtenidos del ADC y la
información relacionada con el tiempo de ejecución.

Dentro de ella se guardan las sumas y los promedios de voltaje,
corriente y potencia, además de la energía acumulada durante la
prueba. También se registran la cantidad de muestras obtenidas,
su promedio, los valores mínimo y máximo del ADC y el tiempo
utilizado para procesar las lecturas.

Otros campos permiten conocer la duración de la prueba, la frecuencia
real de adquisición, la cantidad de lecturas realizadas al INA219,
los bloques procesados y los posibles desbordamientos del DMA.
De esta manera, al terminar cada método se cuenta con la información
necesaria para mostrar sus resultados y realizar una comparación.

Se declaran dos variables de esta estructura porque se necesita
guardar por separado la información de cada método: una corresponde
a la prueba sin DMA y la otra a la prueba con DMA.
*/

typedef struct {
    double Energia_J;

    double Voltaje_Suma;
    double Corriente_Suma;
    double Potencia_Suma;

    double Voltaje_Promedio;
    double Corriente_Promedio;
    double Potencia_Promedio;

    uint64_t Suma_ADC;
    double Promedio_ADC;

    double Tiempo_Procesamiento_ADC_us;
    double Tiempo_Procesamiento_Promedio_us;

    double Duracion_s;
    double Frecuencia_Real_Hz;

    uint32_t Lecturas_INA;
    uint32_t Muestras_ADC;
    uint32_t Muestras_ADC_Raw;
    uint32_t Muestras_Perdidas;
    uint32_t Bloques;
    uint32_t Desbordamientos;

    uint32_t ADC_Minimo;
    uint32_t ADC_Maximo;

} Resultados_t;

static Resultados_t Resultados_Sin_DMA;
static Resultados_t Resultados_Con_DMA;

/* =========================================================
   PROTOTIPOS
   ========================================================= */

/*
Antes de desarrollar las funciones se declaran sus prototipos.
Esto permite indicar desde el inicio cuáles serán las funciones
que se utilizarán en el programa, qué información recibirán y
cuáles devolverán un resultado.

Se incluyen las funciones relacionadas con la comunicación I2C,
las lecturas del INA219, la preparación del ADC y el desarrollo
de las dos pruebas. También se declaran las funciones que permiten
actualizar los datos obtenidos, calcular los promedios, mostrar
reportes durante la ejecución y presentar la comparación final.

De esta forma, el código queda organizado en diferentes partes
y cada función se encarga de una actividad específica.
*/

static void I2C_Init(void);
static void INA219_Init(void);

static void ADC_Init(void);
static void ADC_Deinit(void);

static void Estabilizar_Sistema(void);

static uint16_t INA219_Leer_Registro(uint8_t Registro);
static void INA219_Leer(void);

static void Actualizar_Energia(
    Resultados_t *Resultados,
    int64_t *Marca_Energia_us,
    double *Potencia_Anterior
);

static void Finalizar_Resultados(
    Resultados_t *Resultados,
    int64_t Inicio_Prueba,
    int64_t Fin_Prueba
);

static void Mostrar_Reporte(
    const char *Modo,
    Resultados_t *Resultados,
    int64_t Transcurrido_us
);

static bool ADC_Desborde_Callback(
    adc_continuous_handle_t Handle,
    const adc_continuous_evt_data_t *Datos,
    void *Usuario
);

static Resultados_t Prueba_Sin_DMA(void);
static Resultados_t Prueba_Con_DMA(void);

static void Mostrar_Comparacion(void);
static void Ejecutar_Evaluacion_Completa(void);

/* =========================================================
   FUNCION PRINCIPAL
   ========================================================= */

/*
La función app_main es el punto donde comienza la ejecución del
programa. Primero se prepara la comunicación I2C y se configura
el INA219, ya que este sensor se utilizará durante las dos pruebas
para medir el consumo eléctrico del sistema.

Después se ejecuta la evaluación completa mediante la función
Ejecutar_Evaluacion_Completa. Cuando termina, el programa permanece
en un ciclo de espera con pausas de dos segundos. Esto evita que
la evaluación vuelva a comenzar automáticamente y permite que
los resultados permanezcan visibles en la terminal.
*/

void app_main(void)
{
    I2C_Init();
    INA219_Init();

    while (1) {
        Ejecutar_Evaluacion_Completa();

        // Se mantiene el programa en espera después de mostrar los resultados.
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

/* =========================================================
   LOGICA PRINCIPAL DE LA EVALUACION
   ========================================================= */

/*
Esta función organiza el orden en que se realizan las pruebas para
poder comparar los dos métodos de adquisición. Primero se ejecuta
la prueba sin DMA, en la que el ESP32 solicita las conversiones
del ADC de manera individual mediante One-shot. Los resultados
obtenidos se guardan en Resultados_Sin_DMA.

Después se agrega una pausa de tres segundos para separar ambos
procesos y posteriormente se ejecuta la prueba con DMA, donde las
conversiones se reciben de forma continua y se procesan por bloques.
Sus resultados se guardan en Resultados_Con_DMA.

Una vez terminadas las dos pruebas, se llama a Mostrar_Comparacion
para presentar los valores obtenidos y observar las diferencias
entre ambos métodos.
*/

static void Ejecutar_Evaluacion_Completa(void)
{
    /* Primera etapa: realizar la prueba sin DMA. */
    Resultados_Sin_DMA = Prueba_Sin_DMA();

    /* Pausa para separar los dos métodos de adquisición. */
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* Segunda etapa: realizar la prueba con DMA. */
    Resultados_Con_DMA = Prueba_Con_DMA();

    /* Mostrar los resultados de ambas pruebas. */
    Mostrar_Comparacion();
}

/* =========================================================
   INICIALIZAR I2C
   ========================================================= */

/*
En esta función se prepara la comunicación que permite al ESP32
intercambiar información con el INA219. Primero se crea una
configuración donde se indican los pines que se utilizarán para
SDA y SCL, el puerto I2C y la velocidad de comunicación.

Después se agrega el INA219 al bus utilizando su dirección 0x40.
Con esto se obtiene un identificador que posteriormente se utiliza
para enviar datos al sensor y solicitar las mediciones que se
necesitan durante las pruebas.
*/

static void I2C_Init(void)
{
    i2c_master_bus_config_t Configuracion_I2C = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_new_master_bus(&Configuracion_I2C, &Bus_I2C);

    i2c_device_config_t Configuracion_INA219 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INA219_ADDRESS,
        .scl_speed_hz = I2C_FREQUENCY,
    };

    i2c_master_bus_add_device(
        Bus_I2C,
        &Configuracion_INA219,
        &INA219
    );
}

/* =========================================================
   INICIALIZAR INA219
   ========================================================= */

/*
En esta función se preparan los valores que se enviarán a los
registros internos del INA219 para dejarlo listo para realizar
las mediciones eléctricas. Se utilizan dos arreglos: el primero
contiene la información de calibración y el segundo los valores
de configuración general del sensor.

La calibración permite que el INA219 pueda obtener correctamente
la medición de corriente de acuerdo con la resistencia de medición
utilizada. Posteriormente, ambos arreglos se envían mediante I2C
al sensor. Al terminar se agrega una pausa de dos segundos para
dar tiempo a que quede preparado antes de continuar con el programa.
*/

static void INA219_Init(void)
{
    uint8_t Calibracion[3] = {
        INA219_REG_CALIB, 0x10, 0x00
    };

    uint8_t Configuracion[3] = {
        INA219_REG_CONFIG, 0x39, 0x9F
    };

    i2c_master_transmit(
        INA219,
        Calibracion,
        sizeof(Calibracion),
        1000
    );

    i2c_master_transmit(
        INA219,
        Configuracion,
        sizeof(Configuracion),
        1000
    );

    vTaskDelay(pdMS_TO_TICKS(2000));
}

/* =========================================================
   INICIALIZAR ADC ONE-SHOT
   ========================================================= */

/*
Esta función prepara el convertidor analógico-digital del ESP32
para trabajar en modo One-shot. Primero se establece que se utilizará
la unidad ADC1 y se guarda su identificador en ADC_OneShot para
poder realizar las lecturas posteriormente.

Después se configura el canal que está conectado al potenciómetro,
indicando la resolución predeterminada y la atenuación seleccionada.
En este método, cada conversión se solicita directamente desde
el programa, por lo que el ESP32 realiza una lectura cuando se
ejecuta la función correspondiente.
*/

static void ADC_Init(void)
{
    adc_oneshot_unit_init_cfg_t Configuracion_Unidad = {
        .unit_id = ADC_UNIT_1,
    };

    adc_oneshot_new_unit(
        &Configuracion_Unidad,
        &ADC_OneShot
    );

    adc_oneshot_chan_cfg_t Configuracion_Canal = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
    };

    adc_oneshot_config_channel(
        ADC_OneShot,
        ADC_CHANNEL,
        &Configuracion_Canal
    );
}

/* =========================================================
   LIBERAR ADC ONE-SHOT
   ========================================================= */

/*
Esta función se utiliza cuando termina la prueba sin DMA. Como
ya no se necesitan realizar más conversiones individuales, se
elimina la unidad ADC que se había creado anteriormente y se
asigna NULL a su identificador.

Esto permite finalizar el uso de esta configuración antes de
continuar con la prueba que utiliza el modo continuo del ADC.
*/

static void ADC_Deinit(void)
{
    adc_oneshot_del_unit(ADC_OneShot);
    ADC_OneShot = NULL;
}

/* =========================================================
   ESTABILIZAR SISTEMA
   ========================================================= */

/*
Antes de comenzar a guardar los resultados de cada prueba se
realiza una etapa de estabilización de 10 segundos. Durante este
tiempo se consulta el INA219 una vez por segundo y se muestran
en la terminal los valores de voltaje, corriente y potencia.

Esto permite observar cómo se comportan las mediciones eléctricas
al inicio y dejar un tiempo de espera antes de comenzar a contar
las muestras y calcular los resultados de cada método. Las lecturas
realizadas durante esta etapa no se incluyen en los resultados
principales de la prueba.
*/

static void Estabilizar_Sistema(void)
{
    printf("\nEstabilizando el sistema durante %d segundos...\n",
           ESTABILIZACION_S);

    for (int i = 1; i <= ESTABILIZACION_S; i++) {

        INA219_Leer();

        printf("[ESTAB] t=%d s | V=%.3f V | I=%.2f mA | P=%.2f mW\n",
               i, Voltaje_Lectura, Corriente_Lectura, Potencia_Lectura);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("Estabilizacion finalizada.\n");
}

/* =========================================================
   LEER REGISTRO DEL INA219
   ========================================================= */

/*
Esta función permite consultar un registro específico del INA219.
Para ello recibe la dirección del registro que se desea leer y
envía esa dirección al sensor mediante la comunicación I2C.

Después se reciben dos bytes de información, que se combinan
para formar un solo valor de 16 bits. El resultado se devuelve
para que otras funciones puedan utilizarlo en la conversión de
los datos de voltaje y corriente.
*/

static uint16_t INA219_Leer_Registro(uint8_t Registro)
{
    uint8_t Datos[2] = {0};

    i2c_master_transmit_receive(
        INA219,
        &Registro,
        1,
        Datos,
        2,
        1000
    );

    // Se unen los dos bytes recibidos para formar el valor completo del registro.
    return ((uint16_t)Datos[0] << 8) | Datos[1];
}

/* =========================================================
   LEER VOLTAJE Y CORRIENTE DEL INA219
   ========================================================= */

/*
Aquí se obtienen las mediciones eléctricas que proporciona el
INA219. Primero se consulta el registro del voltaje del bus y
después el registro de corriente. Los valores recibidos se
convierten a unidades que facilitan su interpretación durante
la evaluación.

El voltaje se expresa en voltios y la corriente en miliamperios.
Con estos dos valores también se calcula la potencia mediante
su multiplicación, obteniendo el resultado en miliwatts. Los
tres valores se guardan en variables globales para que puedan
utilizarse en los reportes y en el cálculo del consumo.
*/

static void INA219_Leer(void)
{
    uint16_t Valor_Voltaje =
        INA219_Leer_Registro(INA219_REG_BUSV);

    int16_t Valor_Corriente =
        (int16_t)INA219_Leer_Registro(INA219_REG_CURRENT);

    Voltaje_Lectura =
        ((Valor_Voltaje >> 3) * 4) / 1000.0f;

    Corriente_Lectura =
        Valor_Corriente * 0.1f;

    Potencia_Lectura =
        Voltaje_Lectura * Corriente_Lectura;
}

/* =========================================================
   ACTUALIZAR ENERGIA Y MEDICIONES DEL INA219
   ========================================================= */

/*
Esta función se encarga de guardar las mediciones eléctricas
obtenidas durante cada prueba y calcular la energía acumulada.
Primero se solicita una nueva lectura al INA219 y se obtiene
el tiempo actual para conocer cuánto tiempo pasó desde la
medición anterior.

Para calcular la energía se utiliza el promedio de la potencia
anterior y la actual, considerando también el tiempo transcurrido
entre ambas lecturas. De esta manera se va acumulando la energía
consumida durante toda la prueba, expresada en joules.

Además, cada lectura de voltaje, corriente y potencia se suma
a su respectivo acumulador. Estos valores se utilizarán al final
para obtener los promedios de las mediciones realizadas.
*/

static void Actualizar_Energia(
    Resultados_t *Resultados,
    int64_t *Marca_Energia_us,
    double *Potencia_Anterior
)
{
    INA219_Leer();

    int64_t Reloj_us = esp_timer_get_time();

    double Intervalo_s =
        (Reloj_us - *Marca_Energia_us) / 1000000.0;

    if (Resultados->Lecturas_INA > 0) {

        double Potencia_Promedio =
            (*Potencia_Anterior + Potencia_Lectura) / 2.0;

        Resultados->Energia_J +=
            (Potencia_Promedio / 1000.0) * Intervalo_s;
    }

    *Potencia_Anterior = Potencia_Lectura;
    *Marca_Energia_us = Reloj_us;

    Resultados->Voltaje_Suma += Voltaje_Lectura;
    Resultados->Corriente_Suma += Corriente_Lectura;
    Resultados->Potencia_Suma += Potencia_Lectura;

    Resultados->Lecturas_INA++;
}

/* =========================================================
   FINALIZAR Y CALCULAR RESULTADOS
   ========================================================= */

/*
Esta función reúne los datos obtenidos durante una prueba y
realiza los cálculos necesarios para presentar sus resultados.
Primero se obtiene la duración real utilizando el tiempo de inicio
y el tiempo de finalización, y con esa información se calcula
la frecuencia de adquisición dividiendo la cantidad de muestras
entre los segundos que duró la prueba.

También se calculan los promedios de voltaje, corriente y potencia
a partir de las lecturas acumuladas del INA219. De manera similar,
se obtiene el promedio de los valores del ADC y el tiempo promedio
de procesamiento por muestra.

Finalmente, se guarda el número de desbordamientos registrados
durante la prueba. Con todos estos datos se completa la estructura
de resultados para utilizarla posteriormente en la comparación.
*/

static void Finalizar_Resultados(
    Resultados_t *Resultados,
    int64_t Inicio_Prueba,
    int64_t Fin_Prueba
)
{
    int64_t Duracion_us = Fin_Prueba - Inicio_Prueba;

    Resultados->Duracion_s =
        Duracion_us / 1000000.0;

    if (Resultados->Duracion_s > 0) {

        Resultados->Frecuencia_Real_Hz =
            Resultados->Muestras_ADC / Resultados->Duracion_s;
    }

    double Esperadas =
        FRECUENCIA_EFECTIVA * Resultados->Duracion_s;

    if (Esperadas > Resultados->Muestras_ADC) {

        Resultados->Muestras_Perdidas =
            (uint32_t)(Esperadas - Resultados->Muestras_ADC);
    }

    if (Resultados->Lecturas_INA > 0) {

        Resultados->Voltaje_Promedio =
            Resultados->Voltaje_Suma /
            Resultados->Lecturas_INA;

        Resultados->Corriente_Promedio =
            Resultados->Corriente_Suma /
            Resultados->Lecturas_INA;

        Resultados->Potencia_Promedio =
            Resultados->Potencia_Suma /
            Resultados->Lecturas_INA;
    }

    if (Resultados->Muestras_ADC > 0) {

        Resultados->Promedio_ADC =
            (double)Resultados->Suma_ADC /
            Resultados->Muestras_ADC;

        Resultados->Tiempo_Procesamiento_Promedio_us =
            Resultados->Tiempo_Procesamiento_ADC_us /
            Resultados->Muestras_ADC;
    }

    Resultados->Desbordamientos = Desbordamientos_DMA;
}

/* =========================================================
   MOSTRAR REPORTE PERIODICO
   ========================================================= */

/*
Esta función permite observar cómo van cambiando los resultados
mientras se está ejecutando cada prueba. Se utiliza tanto en el
método sin DMA como en el método con DMA, por lo que recibe el
nombre del modo que se está evaluando y la estructura donde se
encuentran guardados sus datos.

Antes de mostrar la información se calculan los promedios
parciales de voltaje, corriente, potencia y ADC. También se
obtiene el tiempo transcurrido y la frecuencia alcanzada hasta
ese momento, dividiendo las muestras acumuladas entre el tiempo.

Con estos datos se imprime un reporte en la terminal cada cinco
segundos. Así se puede observar la cantidad de muestras obtenidas,
sus valores mínimo y máximo, las mediciones eléctricas y la energía
acumulada sin tener que esperar hasta que termine toda la prueba.
*/

static void Mostrar_Reporte(
    const char *Modo,
    Resultados_t *Resultados,
    int64_t Transcurrido_us
)
{
    double Voltaje_Promedio = 0;
    double Corriente_Promedio = 0;
    double Potencia_Promedio = 0;
    double Promedio_ADC = 0;
    double Perdidas = 0;

    if (Resultados->Lecturas_INA > 0) {

        Voltaje_Promedio =
            Resultados->Voltaje_Suma /
            Resultados->Lecturas_INA;

        Corriente_Promedio =
            Resultados->Corriente_Suma /
            Resultados->Lecturas_INA;

        Potencia_Promedio =
            Resultados->Potencia_Suma /
            Resultados->Lecturas_INA;
    }

    if (Resultados->Muestras_ADC > 0) {

        Promedio_ADC =
            (double)Resultados->Suma_ADC /
            Resultados->Muestras_ADC;
    }

    double Segundos = Transcurrido_us / 1000000.0;

    double Frecuencia = 0;

    if (Segundos > 0) {
        Frecuencia = Resultados->Muestras_ADC / Segundos;
    }

    double Esperadas = FRECUENCIA_EFECTIVA * Segundos;

    if (Esperadas > Resultados->Muestras_ADC) {
        Perdidas = Esperadas - Resultados->Muestras_ADC;
    }

    printf("\n---------- %s ----------\n", Modo);

    printf("Tiempo: %.2f s\n", Segundos);

    printf("Muestras ADC: %lu\n",
           (unsigned long)Resultados->Muestras_ADC);

    printf("Muestras perdidas: %.0f\n", Perdidas);

    printf("Desbordamientos DMA: %lu\n",
           (unsigned long)Desbordamientos_DMA);

    printf("Frecuencia efectiva: %.2f Hz\n", Frecuencia);

    printf("Promedio ADC: %.2f\n", Promedio_ADC);

    printf("ADC minimo: %lu\n",
           (unsigned long)Resultados->ADC_Minimo);

    printf("ADC maximo: %lu\n",
           (unsigned long)Resultados->ADC_Maximo);

    printf("Voltaje promedio: %.3f V\n", Voltaje_Promedio);

    printf("Corriente promedio: %.3f mA\n", Corriente_Promedio);

    printf("Potencia promedio: %.3f mW\n", Potencia_Promedio);

    printf("Energia acumulada: %.6f J\n",
           Resultados->Energia_J);

    printf("-------------------------------\n");
}

/* =========================================================
   CALLBACK DE DESBORDAMIENTO DMA
   ========================================================= */

/*
Esta función se utiliza como aviso cuando ocurre un desbordamiento
en el almacenamiento de datos del ADC continuo. Esto puede suceder
cuando se generan datos y el programa no alcanza a procesarlos antes
de que el espacio disponible llegue a su límite.

Cada vez que se presenta este evento, se aumenta el contador
Desbordamientos_DMA. De esta manera, al terminar la prueba se
puede consultar cuántas veces se presentó esta situación y tener
una referencia sobre el comportamiento de la recepción de datos.
*/

static bool ADC_Desborde_Callback(
    adc_continuous_handle_t Handle,
    const adc_continuous_evt_data_t *Datos,
    void *Usuario
)
{
    Desbordamientos_DMA++;

    return false;
}

/* =========================================================
   PRUEBA SIN DMA: ADC ONE-SHOT
   ========================================================= */

/*
En esta función se desarrolla la primera parte de la evaluación,
utilizando el ADC en modo One-shot, es decir, solicitando cada
conversión directamente desde el programa. Para comenzar se
declara una estructura local donde se guardarán los resultados
de esta prueba y se preparan las variables que se necesitan para
calcular la energía y llevar el control de las mediciones.

Después se configura el ADC y se realiza el periodo de estabilización
de 10 segundos. Una vez terminado este tiempo, comienza el conteo
de los 120 segundos de evaluación. Durante este periodo se realizan
grupos de 10 lecturas individuales del potenciómetro. Cada conversión
se mide con el temporizador para acumular el tiempo utilizado en
la función de lectura.

Con cada muestra se actualiza la cantidad total de lecturas, la
suma de los valores y los valores mínimo y máximo del ADC. Además,
se consulta el INA219 aproximadamente cada segundo para actualizar
las mediciones eléctricas y el cálculo de energía.

Cada cinco segundos se muestra un reporte con los resultados
parciales. También se agrega una pausa de un milisegundo después
de procesar cada grupo de lecturas. Cuando se cumplen los 120
segundos, se realiza una última lectura del INA219, se libera
el ADC y se calculan los resultados finales de esta prueba.

Esta parte permite observar el comportamiento del método en el
que el procesador solicita y obtiene las conversiones una por una.
*/

static Resultados_t Prueba_Sin_DMA(void)
{
    Resultados_t Resultados = {0};

    double Potencia_Anterior = 0;

    Resultados.ADC_Minimo = UINT32_MAX;
    Desbordamientos_DMA = 0;

    printf("\n========================================\n");
    printf("     INICIANDO PRUEBA ADC SIN DMA\n");
    printf("========================================\n");

    ADC_Init();

    Estabilizar_Sistema();

    int64_t Inicio_Prueba = esp_timer_get_time();

    int64_t Ultima_Lectura_INA_us = Inicio_Prueba;
    int64_t Ultimo_Resumen_us = Inicio_Prueba;
    int64_t Marca_Energia_us = Inicio_Prueba;

    int Valor_ADC = 0;

    Actualizar_Energia(
        &Resultados,
        &Marca_Energia_us,
        &Potencia_Anterior
    );

    while ((esp_timer_get_time() - Inicio_Prueba) < DURACION_PRUEBA) {

        for (int i = 0; i < MUESTRAS_POR_BLOQUE; i++) {

            int64_t Inicio_Conversion = esp_timer_get_time();

            adc_oneshot_read(
                ADC_OneShot,
                ADC_CHANNEL,
                &Valor_ADC
            );

            int64_t Fin_Conversion = esp_timer_get_time();

            Resultados.Tiempo_Procesamiento_ADC_us +=
                Fin_Conversion - Inicio_Conversion;

            Resultados.Muestras_ADC++;

            Resultados.Suma_ADC += Valor_ADC;

            if ((uint32_t)Valor_ADC < Resultados.ADC_Minimo) {
                Resultados.ADC_Minimo = Valor_ADC;
            }

            if ((uint32_t)Valor_ADC > Resultados.ADC_Maximo) {
                Resultados.ADC_Maximo = Valor_ADC;
            }
        }

        Resultados.Bloques++;

        int64_t Reloj_us = esp_timer_get_time();

        if ((Reloj_us - Ultima_Lectura_INA_us) >= 1000000LL) {

            Actualizar_Energia(
                &Resultados,
                &Marca_Energia_us,
                &Potencia_Anterior
            );

            Ultima_Lectura_INA_us = Reloj_us;
        }

        if ((Reloj_us - Ultimo_Resumen_us) >= 5000000LL) {

            Mostrar_Reporte(
                "ADC SIN DMA",
                &Resultados,
                Reloj_us - Inicio_Prueba
            );

            Ultimo_Resumen_us = Reloj_us;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    Actualizar_Energia(
        &Resultados,
        &Marca_Energia_us,
        &Potencia_Anterior
    );

    int64_t Fin_Prueba = esp_timer_get_time();

    ADC_Deinit();

    Finalizar_Resultados(
        &Resultados,
        Inicio_Prueba,
        Fin_Prueba
    );

    printf("\nFINALIZO LA PRUEBA SIN DMA\n");

    return Resultados;
}

/* =========================================================
   PRUEBA CON DMA: ADC CONTINUOUS
   ========================================================= */

/*
En esta función se desarrolla la segunda parte de la evaluación,
pero ahora utilizando el ADC en modo continuo con DMA. A diferencia
del método anterior, aquí no se solicita cada conversión de manera
individual, sino que el ADC realiza las adquisiciones de forma
continua y los datos se van almacenando para que el programa pueda
recuperarlos y procesarlos por bloques.

Primero se prepara el almacenamiento que recibirá las muestras y
se configura el ADC con una frecuencia de 20,000 muestras por segundo.
También se establece el canal conectado al potenciómetro y el formato
en el que se recibirán los datos. Se registra una función que permite
contar los desbordamientos que puedan ocurrir durante la adquisición.

Después de realizar la estabilización de 10 segundos, se inicia la
adquisición continua y comienza el conteo de los 120 segundos.
Durante la prueba, el programa solicita los datos almacenados y
los coloca temporalmente en un buffer para poder recorrerlos.

Cada bloque recibido se procesa para obtener los valores del ADC.
Se utiliza la decimación de dos, por lo que solamente se conserva
una de cada dos muestras recibidas. Las muestras seleccionadas
se suman y se utilizan para actualizar el promedio, así como los
valores mínimo y máximo.

Al igual que en la prueba anterior, el INA219 se consulta aproximadamente
cada segundo y se muestran reportes parciales cada cinco segundos.
También se mide el tiempo utilizado para procesar los bloques
recibidos. Cuando se cumple el tiempo establecido, se realiza una
última lectura eléctrica, se detiene la adquisición continua y se
liberan los recursos utilizados por el DMA.

Con esta prueba se busca observar el comportamiento del ADC cuando
las conversiones se realizan de manera continua y el movimiento
de los datos se apoya en DMA.
*/

static Resultados_t Prueba_Con_DMA(void)
{
    Resultados_t Resultados = {0};

    double Potencia_Anterior = 0;

    Resultados.ADC_Minimo = UINT32_MAX;
    Desbordamientos_DMA = 0;

    printf("\n========================================\n");
    printf("     INICIANDO PRUEBA ADC CON DMA\n");
    printf("========================================\n");

    adc_continuous_handle_t Handle_DMA = NULL;

    adc_continuous_handle_cfg_t Configuracion_DMA = {
        .max_store_buf_size = BUFFER_DMA,
        .conv_frame_size = TAMANO_BLOQUE,
    };

    adc_continuous_new_handle(
        &Configuracion_DMA,
        &Handle_DMA
    );

    adc_digi_pattern_config_t Patron_ADC = {
        .atten = ADC_ATTEN,
        .channel = ADC_CHANNEL,
        .unit = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };

    adc_continuous_config_t Configuracion_ADC = {
        .sample_freq_hz = FRECUENCIA_DMA,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        .pattern_num = 1,
        .adc_pattern = &Patron_ADC,
    };

    adc_continuous_config(
        Handle_DMA,
        &Configuracion_ADC
    );

    adc_continuous_evt_cbs_t Callbacks_DMA = {
        .on_pool_ovf = ADC_Desborde_Callback,
    };

    adc_continuous_register_event_callbacks(
        Handle_DMA,
        &Callbacks_DMA,
        NULL
    );

    Estabilizar_Sistema();

    adc_continuous_start(Handle_DMA);

    int64_t Inicio_Prueba = esp_timer_get_time();

    int64_t Ultima_Lectura_INA_us = Inicio_Prueba;
    int64_t Ultimo_Resumen_us = Inicio_Prueba;
    int64_t Marca_Energia_us = Inicio_Prueba;

    // Espacio temporal donde se colocan los datos recibidos en cada lectura del DMA.
    uint8_t Buffer_ADC[TAMANO_BLOQUE];

    uint32_t Bytes_Recibidos = 0;

    // Lleva el control para conservar una muestra por cada dos recibidas.
    uint32_t Contador_Decimacion = 0;

    Actualizar_Energia(
        &Resultados,
        &Marca_Energia_us,
        &Potencia_Anterior
    );

    while ((esp_timer_get_time() - Inicio_Prueba) < DURACION_PRUEBA) {

        Bytes_Recibidos = 0;

        adc_continuous_read(
            Handle_DMA,
            Buffer_ADC,
            sizeof(Buffer_ADC),
            &Bytes_Recibidos,
            100
        );

        if (Bytes_Recibidos > 0) {

            int64_t Inicio_Procesamiento = esp_timer_get_time();

            uint32_t Numero_Muestras =
                Bytes_Recibidos / SOC_ADC_DIGI_RESULT_BYTES;

            for (uint32_t i = 0; i < Numero_Muestras; i++) {

                adc_digi_output_data_t *Dato_ADC =
                    (adc_digi_output_data_t *)
                    &Buffer_ADC[i * SOC_ADC_DIGI_RESULT_BYTES];

                Resultados.Muestras_ADC_Raw++;

                Contador_Decimacion++;

                if ((Contador_Decimacion % DECIMACION) == 0) {

                    uint32_t Valor_ADC = Dato_ADC->type1.data;

                    Resultados.Muestras_ADC++;

                    Resultados.Suma_ADC += Valor_ADC;

                    if (Valor_ADC < Resultados.ADC_Minimo) {
                        Resultados.ADC_Minimo = Valor_ADC;
                    }

                    if (Valor_ADC > Resultados.ADC_Maximo) {
                        Resultados.ADC_Maximo = Valor_ADC;
                    }
                }
            }

            int64_t Fin_Procesamiento = esp_timer_get_time();

            Resultados.Tiempo_Procesamiento_ADC_us +=
                Fin_Procesamiento - Inicio_Procesamiento;

            Resultados.Bloques++;
        }

        int64_t Reloj_us = esp_timer_get_time();

        if ((Reloj_us - Ultima_Lectura_INA_us) >= 1000000LL) {

            Actualizar_Energia(
                &Resultados,
                &Marca_Energia_us,
                &Potencia_Anterior
            );

            Ultima_Lectura_INA_us = Reloj_us;
        }

        if ((Reloj_us - Ultimo_Resumen_us) >= 5000000LL) {

            Mostrar_Reporte(
                "ADC CON DMA",
                &Resultados,
                Reloj_us - Inicio_Prueba
            );

            Ultimo_Resumen_us = Reloj_us;
        }
    }

    Actualizar_Energia(
        &Resultados,
        &Marca_Energia_us,
        &Potencia_Anterior
    );

    int64_t Fin_Prueba = esp_timer_get_time();

    adc_continuous_stop(Handle_DMA);

    adc_continuous_deinit(Handle_DMA);

    Finalizar_Resultados(
        &Resultados,
        Inicio_Prueba,
        Fin_Prueba
    );

    printf("\nFINALIZO LA PRUEBA CON DMA\n");

    return Resultados;
}

/* =========================================================
   COMPARACION FINAL DE LAS DOS PRUEBAS
   ========================================================= */

/*
Esta es la última función del programa y se encarga de mostrar
en una misma tabla los resultados obtenidos durante las dos
pruebas. Se comparan los valores de voltaje, corriente y potencia
promedio, así como la energía acumulada durante los 120 segundos
de cada método.

También se muestran la cantidad de muestras válidas, las muestras
perdidas calculadas, la frecuencia efectiva alcanzada y el tiempo
promedio de procesamiento por muestra. En el caso de los
desbordamientos DMA, se muestra el resultado correspondiente
a la prueba continua, ya que ese contador se utiliza para observar
los problemas de almacenamiento que pueden presentarse en ese modo.

Por último, se calcula el porcentaje de reducción de energía al
utilizar DMA, tomando como referencia el consumo registrado en
la prueba sin DMA. Esto permite observar si existe una diferencia
en el consumo eléctrico entre ambos métodos y tener una referencia
para analizar sus resultados.
*/

static void Mostrar_Comparacion(void)
{
    double Reduccion_Energia = 0;

    if (Resultados_Sin_DMA.Energia_J > 0) {
        Reduccion_Energia =
            ((Resultados_Sin_DMA.Energia_J - Resultados_Con_DMA.Energia_J) /
            Resultados_Sin_DMA.Energia_J) * 100.0;
    }

    printf("\n");
    printf("==============================================================\n");
    printf("                     COMPARACION FINAL                        \n");
    printf("==============================================================\n");

    printf("\n%-36s %14s %14s\n", "Metrica", "Sin DMA", "Con DMA");

    printf("%-36s %14.2f %14.2f\n", "Duracion efectiva (s)",
           Resultados_Sin_DMA.Duracion_s, Resultados_Con_DMA.Duracion_s);

    printf("%-36s %14.3f %14.3f\n", "Voltaje promedio (V)",
           Resultados_Sin_DMA.Voltaje_Promedio, Resultados_Con_DMA.Voltaje_Promedio);

    printf("%-36s %14.3f %14.3f\n", "Corriente promedio (mA)",
           Resultados_Sin_DMA.Corriente_Promedio, Resultados_Con_DMA.Corriente_Promedio);

    printf("%-36s %14.3f %14.3f\n", "Potencia promedio (mW)",
           Resultados_Sin_DMA.Potencia_Promedio, Resultados_Con_DMA.Potencia_Promedio);

    printf("%-36s %14.6f %14.6f\n", "Energia acumulada (J)",
           Resultados_Sin_DMA.Energia_J, Resultados_Con_DMA.Energia_J);

    printf("%-36s %14.0f %14.0f\n", "Muestras validas",
           (double)Resultados_Sin_DMA.Muestras_ADC, (double)Resultados_Con_DMA.Muestras_ADC);

    printf("%-36s %14.0f %14.0f\n", "Muestras perdidas",
           (double)Resultados_Sin_DMA.Muestras_Perdidas, (double)Resultados_Con_DMA.Muestras_Perdidas);

    printf("%-36s %14.2f %14.2f\n", "Frecuencia efectiva (Hz)",
           Resultados_Sin_DMA.Frecuencia_Real_Hz, Resultados_Con_DMA.Frecuencia_Real_Hz);

    printf("%-36s %14.3f %14.3f\n", "Tiempo CPU por muestra (us)",
           Resultados_Sin_DMA.Tiempo_Procesamiento_Promedio_us, Resultados_Con_DMA.Tiempo_Procesamiento_Promedio_us);

    printf("%-36s %14s %14lu\n", "Desbordamientos DMA",
           "N/A", (unsigned long)Resultados_Con_DMA.Desbordamientos);

    printf("\nReduccion de energia con DMA: %.2f %%\n", Reduccion_Energia);

    printf("\n==============================================================\n");
    printf("                    EVALUACION TERMINADA                      \n");
    printf("==============================================================\n");
}
