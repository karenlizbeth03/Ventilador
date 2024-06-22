#include "DHT.h"
#include <LiquidCrystal.h>

#define DHTPIN 2     // Pin donde está conectado el primer sensor
#define DHTPIN2 10    // Pin donde está conectado el segundo sensor
#define MOTOR_PIN 3  // Pin donde está conectado el motor
#define PWM_FREQ 100 // Frecuencia PWM en Hz
#define LED_1 11 // Pin donde está conectado el LED para indicar potencia calefactor es baja
#define LED_2 12 // Pin donde está conectado el LED para indicar potencia calefactor es media
#define LED_3 13 // Pin donde está conectado el LED que indica potencia calefactor es alta

#define FIS_TYPE float
#define FIS_RESOLUSION 101
#define FIS_MIN -3.4028235E+38
#define FIS_MAX 3.4028235E+38
typedef FIS_TYPE(*_FIS_MF)(FIS_TYPE, FIS_TYPE*);
typedef FIS_TYPE(*_FIS_ARR_OP)(FIS_TYPE, FIS_TYPE);
typedef FIS_TYPE(*_FIS_ARR)(FIS_TYPE*, int, _FIS_ARR_OP);

DHT dht(DHTPIN, DHT22);
DHT dht2(DHTPIN2, DHT22);
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // Inicializar la pantalla LCD

// Número de entradas al sistema de inferencia difusa
const int fis_gcI = 3;
// Número de salidas al sistema de inferencia difusa
const int fis_gcO = 2;
// Número de reglas al sistema de inferencia difusa
const int fis_gcR = 20;

FIS_TYPE g_fisInput[fis_gcI];
FIS_TYPE g_fisOutput[fis_gcO];

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2); // Inicializar la pantalla LCD de 16x2
  pinMode(MOTOR_PIN, OUTPUT); // Configurar el pin del motor como salida
  pinMode(LED_1, OUTPUT); // Configurar el pin del LED del ventilador como salida
  pinMode(LED_2, OUTPUT); // Configurar el pin del LED del calefactor como salida
  pinMode(LED_3, OUTPUT);
  Serial.println("Iniciando...");
  dht.begin();
  dht2.begin();
}

void loop() {
  delay(2000);
  float h = dht.readHumidity(); //Leemos la Humedad del primer sensor
  float t = dht.readTemperature(); //Leemos la temperatura en grados Celsius del primer sensor
  float f = dht.readTemperature(true); //Leemos la temperatura en grados Fahrenheit del primer sensor

  float h2 = dht2.readHumidity(); //Leemos la Humedad del segundo sensor
  float t2 = dht2.readTemperature(); //Leemos la temperatura en grados Celsius del segundo sensor
  float f2 = dht2.readTemperature(true); //Leemos la temperatura en grados Fahrenheit del segundo sensor

  //--------Enviamos las lecturas por el puerto serial-------------
  Serial.print("Sensor 1 - Humedad ");
  Serial.print(h);
  Serial.print(" %, Temperatura: ");
  Serial.print(t);
  Serial.print(" *C, ");
 
  Serial.print("Sensor 2 - Humedad ");
  Serial.print(h2);
  Serial.print(" %, Temperatura: ");
  Serial.print(t2);
  Serial.print(" *C, ");
  Serial.print(f2);
  Serial.println(" *F");
// Imprimir los valores de salida en el monitor serial
Serial.print("Velocidad: ");
Serial.println(g_fisOutput[0]);
Serial.print("Calefactor: ");
Serial.println(g_fisOutput[1]);
  // Mostrar en la pantalla LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T1:");
  lcd.print(int(t));
  
  lcd.print(" | T2:");
  lcd.print(int(t2));
 

  lcd.setCursor(0, 1);
  lcd.print("H:");
  lcd.print(int(h));
  lcd.print("% ");
  
  // Aquí va la evaluación del sistema de inferencia difusa
  g_fisOutput[0] = 0;
  g_fisOutput[1] = 0;

  fis_evaluate();

 // Set output value: Velocidad Ventilador
int fanSpeed = map(g_fisOutput[0], -1, 1, 100, 250); // Mapear entre 100 y 250
analogWrite(MOTOR_PIN, fanSpeed);
lcd.print(" V:");
lcd.print(int(g_fisOutput[0])); 

// Set output value: Potencia Calefactor
int heaterPower = map(g_fisOutput[1], -1, 1, 0, 255); // Mapear entre 0 y 255
if (heaterPower < 85) {
  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_2, LOW);
  digitalWrite(LED_3, LOW);
} else if (heaterPower < 170) {
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_3, HIGH); // Usar LED_2 para potencia media
  digitalWrite(LED_2, LOW);
} else {
  digitalWrite(LED_3, LOW);
  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, HIGH); // Usar LED_3 para potencia alta
}
lcd.print(" C:");
lcd.print(int(g_fisOutput[1])); 
  delay(2000);
}
//***********************************************************************
// Funciones de soporte para el sistema de inferencia difusa                         
//***********************************************************************
// Función de miembro trapezoidal
FIS_TYPE fis_trapmf(FIS_TYPE x, FIS_TYPE* p)
{
    // Parámetros del miembro trapezoidal
    FIS_TYPE a = p[0], b = p[1], c = p[2], d = p[3];
    // Cálculo de las pertenencias del punto x a los dos trapezoides
    FIS_TYPE t1 = ((x <= c) ? 1 : ((d < x) ? 0 : ((c != d) ? ((d - x) / (d - c)) : 0)));
    FIS_TYPE t2 = ((b <= x) ? 1 : ((x < a) ? 0 : ((a != b) ? ((x - a) / (b - a)) : 0)));
    // Devuelve el mínimo entre los dos valores calculados
    return (FIS_TYPE) min(t1, t2);
}

// Función de miembro triangular
FIS_TYPE fis_trimf(FIS_TYPE x, FIS_TYPE* p)
{
    // Parámetros del miembro triangular
    FIS_TYPE a = p[0], b = p[1], c = p[2];
    // Cálculo de las pertenencias del punto x a los tres triángulos
    FIS_TYPE t1 = (x - a) / (b - a);
    FIS_TYPE t2 = (c - x) / (c - b);
    // Devuelve el máximo entre los dos valores calculados
    if ((a == b) && (b == c)) return (FIS_TYPE) (x == a);
    if (a == b) return (FIS_TYPE) (t2*(b <= x)*(x <= c));
    if (b == c) return (FIS_TYPE) (t1*(a <= x)*(x <= b));
    t1 = min(t1, t2);
    return (FIS_TYPE) max(t1, 0);
}

// Función de mínimo
FIS_TYPE fis_min(FIS_TYPE a, FIS_TYPE b)
{
    // Devuelve el mínimo entre dos valores
    return min(a, b);
}

// Función de máximo
FIS_TYPE fis_max(FIS_TYPE a, FIS_TYPE b)
{
    // Devuelve el máximo entre dos valores
    return max(a, b);
}

// Función de operación en un arreglo
FIS_TYPE fis_array_operation(FIS_TYPE *array, int size, _FIS_ARR_OP pfnOp)
{
    int i;
    FIS_TYPE ret = 0;

    // Si el tamaño del arreglo es 0, devuelve 0
    if (size == 0) return ret;
    // Si el tamaño del arreglo es 1, devuelve el único elemento del arreglo
    if (size == 1) return array[0];

    // Inicializa el valor de retorno con el primer elemento del arreglo
    ret = array[0];
    // Itera sobre los elementos restantes del arreglo
    for (int i = 1; i < size; i++)
    {
        // Aplica la operación especificada en cada elemento del arreglo
        ret = (*pfnOp)(ret, array[i]);
    }

    // Devuelve el resultado de la operación
    return ret;
}


//***********************************************************************
// Datos para el sistema de inferencia difusa                              
//***********************************************************************
// Punteros a las implementaciones de las funciones miembro
_FIS_MF fis_gMF[] =
{
    fis_trapmf, fis_trimf
};

// Conteo de funciones miembro para cada Entrada
int fis_gIMFCount[] = { 5, 5, 3 };

// Conteo de funciones miembro para cada Salida
int fis_gOMFCount[] = { 4, 4 };

// Coeficientes para las Funciones de Miembro de Entrada
FIS_TYPE fis_gMFI0Coeff1[] = { -15, -15, 5, 10 };
FIS_TYPE fis_gMFI0Coeff2[] = { 5, 15, 25 };
FIS_TYPE fis_gMFI0Coeff3[] = { 20, 25, 28 };
FIS_TYPE fis_gMFI0Coeff4[] = { 28, 30, 32 };
FIS_TYPE fis_gMFI0Coeff5[] = { 30, 35, 40, 40 };
FIS_TYPE* fis_gMFI0Coeff[] = { fis_gMFI0Coeff1, fis_gMFI0Coeff2, fis_gMFI0Coeff3, fis_gMFI0Coeff4, fis_gMFI0Coeff5 };
FIS_TYPE fis_gMFI1Coeff1[] = { -15, -15, 5, 10 };
FIS_TYPE fis_gMFI1Coeff2[] = { 5, 15, 25 };
FIS_TYPE fis_gMFI1Coeff3[] = { 20, 25, 28 };
FIS_TYPE fis_gMFI1Coeff4[] = { 28, 30, 32 };
FIS_TYPE fis_gMFI1Coeff5[] = { 30, 35, 40, 40 };
FIS_TYPE* fis_gMFI1Coeff[] = { fis_gMFI1Coeff1, fis_gMFI1Coeff2, fis_gMFI1Coeff3, fis_gMFI1Coeff4, fis_gMFI1Coeff5 };
FIS_TYPE fis_gMFI2Coeff1[] = { 0, 0, 15, 30 };
FIS_TYPE fis_gMFI2Coeff2[] = { 15, 45, 75 };
FIS_TYPE fis_gMFI2Coeff3[] = { 60, 75, 100, 100 };
FIS_TYPE* fis_gMFI2Coeff[] = { fis_gMFI2Coeff1, fis_gMFI2Coeff2, fis_gMFI2Coeff3 };
FIS_TYPE** fis_gMFICoeff[] = { fis_gMFI0Coeff, fis_gMFI1Coeff, fis_gMFI2Coeff };

// Coeficientes para las Funciones de Miembro de Salida
FIS_TYPE fis_gMFO0Coeff1[] = { 0, 0, 1 };
FIS_TYPE fis_gMFO0Coeff2[] = { 0, 1, 2 };
FIS_TYPE fis_gMFO0Coeff3[] = { 1, 2, 3 };
FIS_TYPE fis_gMFO0Coeff4[] = { 2, 3, 4 };
FIS_TYPE* fis_gMFO0Coeff[] = { fis_gMFO0Coeff1, fis_gMFO0Coeff2, fis_gMFO0Coeff3, fis_gMFO0Coeff4 };
FIS_TYPE fis_gMFO1Coeff1[] = { 0, 0, 1 };
FIS_TYPE fis_gMFO1Coeff2[] = { 0, 1, 2 };
FIS_TYPE fis_gMFO1Coeff3[] = { 1, 2, 3 };
FIS_TYPE fis_gMFO1Coeff4[] = { 2, 3, 4 };
FIS_TYPE* fis_gMFO1Coeff[] = { fis_gMFO1Coeff1, fis_gMFO1Coeff2, fis_gMFO1Coeff3, fis_gMFO1Coeff4 };
FIS_TYPE** fis_gMFOCoeff[] = { fis_gMFO0Coeff, fis_gMFO1Coeff };

// Conjuntos de Funciones de Miembro de Entrada
int fis_gMFI0[] = { 0, 1, 1, 1, 0 };
int fis_gMFI1[] = { 0, 1, 1, 1, 0 };
int fis_gMFI2[] = { 0, 1, 0 };
int* fis_gMFI[] = { fis_gMFI0, fis_gMFI1, fis_gMFI2};

// Conjuntos de Funciones de Miembro de Salida
int fis_gMFO0[] = { 1, 1, 1, 1 };
int fis_gMFO1[] = { 1, 1, 1, 1 };
int* fis_gMFO[] = { fis_gMFO0, fis_gMFO1};

// Pesos de regla
FIS_TYPE fis_gRWeight[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Tipo de regla
int fis_gRType[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

// Reglas de Entrada
int fis_gRI0[] = { 2, 3, 3 };
int fis_gRI1[] = { 1, 1, 3 };
int fis_gRI2[] = { 5, 1, 2 };
int fis_gRI3[] = { 2, 5, 3 };
int fis_gRI4[] = { 2, 1, 2 };
int fis_gRI5[] = { 2, 2, 3 };
int fis_gRI6[] = { 1, 5, 2 };
int fis_gRI7[] = { 2, 4, 1 };
int fis_gRI8[] = { 3, 2, 1 };
int fis_gRI9[] = { 1, 2, 3 };
int fis_gRI10[] = { 5, 5, 3 };
int fis_gRI11[] = { 4, 2, 3 };
int fis_gRI12[] = { 1, 3, 1 };
int fis_gRI13[] = { 3, 5, 2 };
int fis_gRI14[] = { 4, 5, 2 };
int fis_gRI15[] = { 5, 4, 2 };
int fis_gRI16[] = { 2, 5, 1 };
int fis_gRI17[] = { 4, 1, 2 };
int fis_gRI18[] = { 1, 3, 1 };
int fis_gRI19[] = { 4, 4, 3 };
int* fis_gRI[] = { fis_gRI0, fis_gRI1, fis_gRI2, fis_gRI3, fis_gRI4, fis_gRI5, fis_gRI6, fis_gRI7, fis_gRI8, fis_gRI9, fis_gRI10, fis_gRI11, fis_gRI12, fis_gRI13, fis_gRI14, fis_gRI15, fis_gRI16, fis_gRI17, fis_gRI18, fis_gRI19 };

// Reglas de salida
int fis_gRO0[] = { 3, 3 };
int fis_gRO1[] = { 3, 4 };
int fis_gRO2[] = { 2, 2 };
int fis_gRO3[] = { 4, 2 };
int fis_gRO4[] = { 2, 4 };
int fis_gRO5[] = { 2, 1 };
int fis_gRO6[] = { 2, 1 };
int fis_gRO7[] = { 1, 3 };
int fis_gRO8[] = { 2, 1 };
int fis_gRO9[] = { 1, 1 };
int fis_gRO10[] = { 2, 4 };
int fis_gRO11[] = { 4, 3 };
int fis_gRO12[] = { 2, 1 };
int fis_gRO13[] = { 3, 4 };
int fis_gRO14[] = { 4, 3 };
int fis_gRO15[] = { 2, 4 };
int fis_gRO16[] = { 3, 4 };
int fis_gRO17[] = { 4, 1 };
int fis_gRO18[] = { 4, 1 };
int fis_gRO19[] = { 2, 3 };
int* fis_gRO[] = { fis_gRO0, fis_gRO1, fis_gRO2, fis_gRO3, fis_gRO4, fis_gRO5, fis_gRO6, fis_gRO7, fis_gRO8, fis_gRO9, fis_gRO10, fis_gRO11, fis_gRO12, fis_gRO13, fis_gRO14, fis_gRO15, fis_gRO16, fis_gRO17, fis_gRO18, fis_gRO19 };

// Entrada rango minimo
FIS_TYPE fis_gIMin[] = { -15, -15, 0 };

// Entrada rango maximo
FIS_TYPE fis_gIMax[] = { 40, 40, 100 };

// Salida rango minimo
FIS_TYPE fis_gOMin[] = { 0, 0 };

// salida rango maximo
FIS_TYPE fis_gOMax[] = { 4, 4 };

//***********************************************************************
// Funciones de soporte dependientes de datos para el sistema de inferencia difusa       
//***********************************************************************
//salida de una función de membresía para un valor de entrada dado

FIS_TYPE fis_MF_out(FIS_TYPE** fuzzyRuleSet, FIS_TYPE x, int o)
{
    FIS_TYPE mfOut;
    int r;

    for (r = 0; r < fis_gcR; ++r)
    {
        int index = fis_gRO[r][o];
        if (index > 0)
        {
            index = index - 1;
            mfOut = (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else if (index < 0)
        {
            index = -index - 1;
            mfOut = 1 - (fis_gMF[fis_gMFO[o][index]])(x, fis_gMFOCoeff[o][index]);
        }
        else
        {
            mfOut = 0;
        }

        fuzzyRuleSet[0][r] = fis_min(mfOut, fuzzyRuleSet[1][r]);
    }
    return fis_array_operation(fuzzyRuleSet[0], fis_gcR, fis_max);
}
//centroide del conjunto difuso
FIS_TYPE fis_defuzz_centroid(FIS_TYPE** fuzzyRuleSet, int o)
{
    FIS_TYPE step = (fis_gOMax[o] - fis_gOMin[o]) / (FIS_RESOLUSION - 1);
    FIS_TYPE area = 0;
    FIS_TYPE momentum = 0;
    FIS_TYPE dist, slice;
    int i;

    // calcular el área bajo la curva formada por las salidas MF
    for (i = 0; i < FIS_RESOLUSION; ++i){
        dist = fis_gOMin[o] + (step * i);
        slice = step * fis_MF_out(fuzzyRuleSet, dist, o);
        area += slice;
        momentum += slice*dist;
    }

    return ((area == 0) ? ((fis_gOMax[o] + fis_gOMin[o]) / 2) : (momentum / area));
}

//***********************************************************************
// Sistema de inferencia difusa                                             
//***********************************************************************
void fis_evaluate()
{
   // Arreglos para almacenar los valores de entrada y salida difusa
    FIS_TYPE fuzzyInput0[] = { 0, 0, 0, 0, 0 };
    FIS_TYPE fuzzyInput1[] = { 0, 0, 0, 0, 0 };
    FIS_TYPE fuzzyInput2[] = { 0, 0, 0 };
    FIS_TYPE* fuzzyInput[fis_gcI] = { fuzzyInput0, fuzzyInput1, fuzzyInput2, };
    FIS_TYPE fuzzyOutput0[] = { 0, 0, 0, 0 };
    FIS_TYPE fuzzyOutput1[] = { 0, 0, 0, 0 };
    FIS_TYPE* fuzzyOutput[fis_gcO] = { fuzzyOutput0, fuzzyOutput1, };
    FIS_TYPE fuzzyRules[fis_gcR] = { 0 };
    FIS_TYPE fuzzyFires[fis_gcR] = { 0 };
    FIS_TYPE* fuzzyRuleSet[] = { fuzzyRules, fuzzyFires };
    // Suma de los pesos de los fuegos difusos
    FIS_TYPE sW = 0;

    // Transformar la entrada en entrada difusa
    int i, j, r, o;
    for (i = 0; i < fis_gcI; ++i)
    {
        for (j = 0; j < fis_gIMFCount[i]; ++j)
        {
            fuzzyInput[i][j] =
                (fis_gMF[fis_gMFI[i][j]])(g_fisInput[i], fis_gMFICoeff[i][j]);
        }
    }

    int index = 0;
      // Calcular los fuegos difusos para cada regla
    for (r = 0; r < fis_gcR; ++r)
    {
        if (fis_gRType[r] == 1)
        {
            fuzzyFires[r] = FIS_MAX;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_min(fuzzyFires[r], 1);
            }
        }
        else
        {
            fuzzyFires[r] = FIS_MIN;
            for (i = 0; i < fis_gcI; ++i)
            {
                index = fis_gRI[r][i];
                if (index > 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], fuzzyInput[i][index - 1]);
                else if (index < 0)
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 1 - fuzzyInput[i][-index - 1]);
                else
                    fuzzyFires[r] = fis_max(fuzzyFires[r], 0);
            }
        }
        // Aplicar el peso de la regla y acumular los fuegos difusos
        fuzzyFires[r] = fis_gRWeight[r] * fuzzyFires[r];
        sW += fuzzyFires[r];
    }
     // Calcular la salida defusa
    if (sW == 0)
    {
        // Si la suma de los pesos es cero, establecer la salida en el punto medio del rango de salida
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = ((fis_gOMax[o] + fis_gOMin[o]) / 2);
        }
    }
    else
    {
        // Calcular el centroide del conjunto difuso para cada salida
        for (o = 0; o < fis_gcO; ++o)
        {
            g_fisOutput[o] = fis_defuzz_centroid(fuzzyRuleSet, o);
        }
    }
  
}