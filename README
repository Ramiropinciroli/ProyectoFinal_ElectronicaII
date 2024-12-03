# **Proyecto final - Contador de Vehículos con ESP32**

## **Cátedra de Electrónica II**

**Universidad Tecnológica Nacional - Facultad Regional Rosario**  
**Año:** 2024  
**Proyecto final**  

---

## **Índice**
1. [Introducción](#introducción)
2. [Circuito](#circuito)
3. [Requisitos del Proyecto](#requisitos-del-proyecto)
4. [Procedimiento](#procedimiento)
    - [Clonar el Repositorio](#clonar-el-repositorio)
    - [Cargar el Proyecto](#cargar-el-proyecto)
    - [Compilar el Proyecto](#compilar-el-proyecto)
    - [Grabar el Microcontrolador](#grabar-el-microcontrolador)
5. [Funcionamiento](#funcionamiento)


---

## **Introducción**

Este proyecto implementa un sistema para contar vehículos utilizando un **ESP32**, sensores ultrasónicos **HC-SR04**, un **display OLED SSD1306** y un **relé** para controlar una luz de 220V. Forma parte del proyecto final de la asignatura **Electrónica II** de la **UTN-FRRo**.

El sistema detecta el ingreso y egreso de vehículos, muestra el número actual de vehículos en un display y controla una luz que se activa según la lógica especificada.

---

## **Circuito**

En la siguiente figura se muestra el diagrama de conexión del proyecto:

![Figura: Circuito del Contador de Vehículos](figures/circuito.png)

**Figura 1:** Circuito del Contador de Vehículos con ESP32.

### **Especificaciones de conexión:**
- **HC-SR04 (sensor ultrasónico):**
  - TRIG_PIN1: GPIO5 (Sensor de ingreso)
  - ECHO_PIN1: GPIO18 (Sensor de ingreso)
  - TRIG_PIN2: GPIO17 (Sensor de egreso)
  - ECHO_PIN2: GPIO16 (Sensor de egreso)
- **Display OLED SSD1306:**
  - SDA: GPIO21
  - SCL: GPIO22
- **Relé:**
  - Controlado por GPIO23

---

## **Requisitos del Proyecto**

### **Hardware:**
1. ESP32 DevKit
2. 2 Sensores ultrasónicos HC-SR04
3. Display OLED SSD1306
4. Módulo Relé
5. Cables de conexión y fuente de alimentación adecuada

### **Software:**
1. **Visual Studio Code** con extensión **PlatformIO**
2. Librerías requeridas:
   - Adafruit GFX Library
   - Adafruit SSD1306
   - Adafruit BusIO
   - U8g2 (opcional)

---

## **Procedimiento**

### **Clonar el Repositorio**
1. Abrir una terminal en tu computadora.
2. Navegar al directorio donde deseas clonar el proyecto.
3. Ejecutar el siguiente comando:

   ```bash
   git clone https://github.com/usuario/repositorio contador_vehiculos

