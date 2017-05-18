/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2015 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * $Date:        13. April 2015
 * $Revision:    V1.1.0
 *
 * Project:      RTE Device Configuration for Freescale MK60
 * -------------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __RTE_DEVICE_H
#define __RTE_DEVICE_H


// <e> ENET (10/100-Mbps Ethernet MAC) [Driver_ETH_MAC0]
// <i> Configuration settings for Driver_ETH_MAC0 in component ::CMSIS Driver:Ethernet MAC
#define RTE_ENET                        0

//   <h> Media Interface
//     <o0> Mode <0=>MII <1=>RMII
//     <o1> Clock Source <0=>MII0_TXCLK/MII0_RXCLK <1=>OSCERCLK <2=>ENET_1588_CLKIN
#define RTE_ENET_MII_MODE               0
#define RTE_ENET_MII_CLOCK_SOURCE       0
//   </h>

// </e>

// <e> I2C0 (Inter-Integrated Circuit Interface 0) [Driver_I2C0]
// <i> Enable or disable Driver_I2C0 in component ::CMSIS Driver:I2C
#define RTE_I2C0                        0

//   <o0> I2C0_SDA <0=>PTB1 <1=>PTB3 <2=>PTD3 <3=>PTD9 <4=>PTE25
//   <o1> I2C0_SCL <0=>PTB0 <1=>PTB2 <2=>PTD2 <3=>PTD8 <4=>PTE24
#define RTE_I2C0_SDA_PIN_ID             0
#define RTE_I2C0_SCL_PIN_ID             0

#if   (RTE_I2C0_SDA_PIN_ID == 0) /* PTB1  */
  #define RTE_I2C0_SDA_PIN    1
  #define RTE_I2C0_SDA_PORT   PORTB_BASE
  #define RTE_I2C0_SDA_MUX    kPortMuxAlt2

#elif (RTE_I2C0_SDA_PIN_ID == 1) /* PTB3  */
  #define RTE_I2C0_SDA_PIN    3
  #define RTE_I2C0_SDA_PORT   PORTB_BASE
  #define RTE_I2C0_SDA_MUX    kPortMuxAlt2

#elif (RTE_I2C0_SDA_PIN_ID == 2) /* PTD3  */
  #define RTE_I2C0_SDA_PIN    3
  #define RTE_I2C0_SDA_PORT   PORTD_BASE
  #define RTE_I2C0_SDA_MUX    kPortMuxAlt7

#elif (RTE_I2C0_SDA_PIN_ID == 3) /* PTD9  */
  #define RTE_I2C0_SDA_PIN    9
  #define RTE_I2C0_SDA_PORT   PORTD_BASE
  #define RTE_I2C0_SDA_MUX    kPortMuxAlt2

#elif (RTE_I2C0_SDA_PIN_ID == 4) /* PTE25 */
  #define RTE_I2C0_SDA_PIN    25
  #define RTE_I2C0_SDA_PORT   PORTE_BASE
  #define RTE_I2C0_SDA_MUX    kPortMuxAlt5

#else
  #error "Invalid I2C0_SDA pin configuration in RTE_Device.h"
#endif

#if   (RTE_I2C0_SCL_PIN_ID == 0) /* PTB0  */
  #define RTE_I2C0_SCL_PIN    0
  #define RTE_I2C0_SCL_PORT   PORTB_BASE
  #define RTE_I2C0_SCL_MUX    kPortMuxAlt2

#elif (RTE_I2C0_SCL_PIN_ID == 1) /* PTB2  */
  #define RTE_I2C0_SCL_PIN    2
  #define RTE_I2C0_SCL_PORT   PORTB_BASE
  #define RTE_I2C0_SCL_MUX    kPortMuxAlt2

#elif (RTE_I2C0_SCL_PIN_ID == 2) /* PTD2  */
  #define RTE_I2C0_SCL_PIN    2
  #define RTE_I2C0_SCL_PORT   PORTD_BASE
  #define RTE_I2C0_SCL_MUX    kPortMuxAlt7

#elif (RTE_I2C0_SCL_PIN_ID == 3) /* PTD8  */
  #define RTE_I2C0_SCL_PIN    8
  #define RTE_I2C0_SCL_PORT   PORTD_BASE
  #define RTE_I2C0_SCL_MUX    kPortMuxAlt2

#elif (RTE_I2C0_SCL_PIN_ID == 4) /* PTE24 */
  #define RTE_I2C0_SCL_PIN    24
  #define RTE_I2C0_SCL_PORT   PORTE_BASE
  #define RTE_I2C0_SCL_MUX    kPortMuxAlt5

#else
  #error "Invalid I2C0_SCL pin configuration in RTE_Device.h"
#endif

//   <o0> DMA Rx <0=>Disable <1=>Enable
//   <o1> DMA Tx <0=>Disable <1=>Enable
#define RTE_I2C0_DMA_RX                 0
#define RTE_I2C0_DMA_TX                 0

// </e>

// <e> I2C1 (Inter-Integrated Circuit Interface 1) [Driver_I2C1]
// <i> Enable or disable Driver_I2C1 in component ::CMSIS Driver:I2C
#define RTE_I2C1                        0

//   <o0> I2C1_SDA <0=>PTC11 <1=>PTE0
//   <o1> I2C1_SCL <0=>PTC10 <1=>PTE1
#define RTE_I2C1_SDA_PIN_ID             0
#define RTE_I2C1_SCL_PIN_ID             0

#if   (RTE_I2C1_SDA_PIN_ID == 0) /* PTC11 */
  #define RTE_I2C1_SDA_PIN    11
  #define RTE_I2C1_SDA_PORT   PORTC_BASE
  #define RTE_I2C1_SDA_MUX    kPortMuxAlt2

#elif (RTE_I2C1_SDA_PIN_ID == 1) /* PTE0  */
  #define RTE_I2C1_SDA_PIN    0
  #define RTE_I2C1_SDA_PORT   PORTE_BASE
  #define RTE_I2C1_SDA_MUX    kPortMuxAlt6

#else
  #error "Invalid I2C1_SDA pin configuration in RTE_Device.h"
#endif

#if   (RTE_I2C1_SCL_PIN_ID == 0) /* PTC10 */
  #define RTE_I2C1_SCL_PIN    10
  #define RTE_I2C1_SCL_PORT   PORTC_BASE
  #define RTE_I2C1_SCL_MUX    kPortMuxAlt2

#elif (RTE_I2C1_SCL_PIN_ID == 1) /* PTE1  */
  #define RTE_I2C1_SCL_PIN    1
  #define RTE_I2C1_SCL_PORT   PORTE_BASE
  #define RTE_I2C1_SCL_MUX    kPortMuxAlt6

#else
  #error "Invalid I2C1_SCL pin configuration in RTE_Device.h"
#endif

//   <o0> DMA Rx <0=>Disable <1=>Enable
//   <o1> DMA Tx <0=>Disable <1=>Enable
#define RTE_I2C1_DMA_RX                 0
#define RTE_I2C1_DMA_TX                 0

// </e>

// <e> I2C2 (Inter-Integrated Circuit Interface 2) [Driver_I2C2]
// <i> Enable or disable Driver_I2C2 in component ::CMSIS Driver:I2C
#define RTE_I2C2                        0

//   <o0> I2C2_SDA <0=>PTA11 <1=>PTA13
//   <o1> I2C2_SCL <0=>PTA12 <1=>PTA14
#define RTE_I2C2_SDA_PIN_ID             0
#define RTE_I2C2_SCL_PIN_ID             0

#if   (RTE_I2C2_SDA_PIN_ID == 0) /* PTA11 */
  #define RTE_I2C2_SDA_PIN    11
  #define RTE_I2C2_SDA_PORT   PORTA_BASE
  #define RTE_I2C2_SDA_MUX    kPortMuxAlt5

#elif (RTE_I2C2_SDA_PIN_ID == 1) /* PTA13 */
  #define RTE_I2C2_SDA_PIN    13
  #define RTE_I2C2_SDA_PORT   PORTA_BASE
  #define RTE_I2C2_SDA_MUX    kPortMuxAlt5

#else
  #error "Invalid I2C2_SDA pin configuration in RTE_Device.h"
#endif

#if   (RTE_I2C2_SCL_PIN_ID == 0) /* PTA12 */
  #define RTE_I2C2_SCL_PIN    12
  #define RTE_I2C2_SCL_PORT   PORTA_BASE
  #define RTE_I2C2_SCL_MUX    kPortMuxAlt5

#elif (RTE_I2C2_SCL_PIN_ID == 1) /* PTA14 */
  #define RTE_I2C2_SCL_PIN    14
  #define RTE_I2C2_SCL_PORT   PORTA_BASE
  #define RTE_I2C2_SCL_MUX    kPortMuxAlt5

#else
  #error "Invalid I2C2_SCL pin configuration in RTE_Device.h"
#endif

//   <o0> DMA Rx <0=>Disable <1=>Enable
//   <o1> DMA Tx <0=>Disable <1=>Enable
#define RTE_I2C2_DMA_RX                 0
#define RTE_I2C2_DMA_TX                 0

// </e>

// <e> SPI0 (Serial Peripheral Interface 0) [Driver_SPI0]
// <i> Enable or disable Driver_SPI0 in component ::CMSIS Driver:SPI
#define RTE_SPI0                        0

//   <o> Peripheral Chip Select <0=>PCS0 <1=>PCS1 <2=>PCS2
//                              <3=>PCS3 <4=>PCS4 <5=>PCS5
#define RTE_SPI0_PCS                    0

// </e>

// <e> SPI1 (Serial Peripheral Interface 1) [Driver_SPI1]
// <i> Enable or disable Driver_SPI1 in component ::CMSIS Driver:SPI
#define RTE_SPI1                        0

//   <o> Peripheral Chip Select <0=>PCS0 <1=>PCS1 <2=>PCS2
//                              <3=>PCS3
#define RTE_SPI1_PCS                    0

// </e>

// <e> SPI2 (Serial Peripheral Interface 0) [Driver_SPI2]
// <i> Enable or disable Driver_SPI2 in component ::CMSIS Driver:SPI
#define RTE_SPI2                        0

//   <o> Peripheral Chip Select <0=>PCS0 <1=>PCS1
#define RTE_SPI2_PCS                    0

// </e>

// <e> UART0 (Universal Asynchronous Receiver/Transmiter 0) [Driver_USART0]
// <i> Enable or disable Driver_USART0 in component ::CMSIS Driver:USART
#define RTE_UART0                       0

//   <o0> DMA Rx <0=>Disable <1=>Enable
//   <o1> DMA Tx <0=>Disable <1=>Enable
#define RTE_UART0_DMA_RX                0
#define RTE_UART0_DMA_TX                0

// </e>

// <e> UART1 (Universal Asynchronous Receiver/Transmiter 1) [Driver_USART1]
// <i> Enable or disable Driver_USART1 in component ::CMSIS Driver:USART
#define RTE_UART1                       0

//   <o0> DMA Rx <0=>Disable <1=>Enable
//   <o1> DMA Tx <0=>Disable <1=>Enable
#define RTE_UART1_DMA_RX                0
#define RTE_UART1_DMA_TX                0

// </e>

// <e> UART2 (Universal Asynchronous Receiver/Transmiter 2) [Driver_USART2]
// <i> Enable or disable Driver_USART2 in component ::CMSIS Driver:USART
#define RTE_UART2                       0

//   <o0> DMA Rx <0=>Disable <1=>Enable
//   <o1> DMA Tx <0=>Disable <1=>Enable
#define RTE_UART2_DMA_RX                0
#define RTE_UART2_DMA_TX                0

// </e>

// <e> UART3 (Universal Asynchronous Receiver/Transmiter 3) [Driver_USART3]
// <i> Enable or disable Driver_USART3 in component ::CMSIS Driver:USART
#define RTE_UART3                       0

//   <o0> DMA Rx <0=>Disable <1=>Enable
//   <o1> DMA Tx <0=>Disable <1=>Enable
#define RTE_UART3_DMA_RX                0
#define RTE_UART3_DMA_TX                0

// </e>

// <e> UART4 (Universal Asynchronous Receiver/Transmiter 4) [Driver_USART4]
// <i> Enable or disable Driver_USART4 in component ::CMSIS Driver:USART
#define RTE_UART4                       0

//   <o0> DMA Rx <0=>Disable <1=>Enable
//   <o1> DMA Tx <0=>Disable <1=>Enable
#define RTE_UART4_DMA_RX                0
#define RTE_UART4_DMA_TX                0

// </e>

// <e> UART5 (Universal Asynchronous Receiver/Transmiter 5) [Driver_USART5]
// <i> Enable or disable Driver_USART5 in component ::CMSIS Driver:USART
#define RTE_UART5                       0

//   <o0> DMA Rx <0=>Disable <1=>Enable
//   <o1> DMA Tx <0=>Disable <1=>Enable
#define RTE_UART5_DMA_RX                0
#define RTE_UART5_DMA_TX                0

// </e>

// <e> SAI (Integrated Interchip Sound 0) [Driver_SAI0]
// <i> Enable or disable Driver_SAI0 in component ::CMSIS Driver:SAI
#define RTE_SAI0                        0

//   <o0> DMA Rx <0=>Disable <1=>Enable
//   <o1> DMA Tx <0=>Disable <1=>Enable
#define RTE_SAI0_DMA_RX                 0
#define RTE_SAI0_DMA_TX                 0

// </e>

// <e> SDHC (Secured Digital Host Controller) [Driver_MCI0]
// <i> Enable or disable Driver_MCI0 in component ::CMSIS Driver:MCI
#define RTE_SDHC                        0

//   <o> Data Bus Width <0=>1-bit <1=>4-bit <2=>8-bit
#define RTE_SDHC_BUS_WIDTH_ID           0

//   <e> Card Detect Pin
//   <i> Enable Card Detect Pin if exists.
//   <i> Input GPIO pin named kGpioSdhc0Cd must be defined in Processor Expert
//     <o1> Active State <0=>Low <1=>High
//     <i>  Selects Active State Logical Level
//   </e>
#define RTE_SDHC_CD_PIN_EN              0
#define RTE_SDHC_CD_ACTIVE              0

//   <e> Write Protect Pin
//   <i> Enable Write Protect Pin if exists.
//   <i> Input GPIO pin named kGpioSdhc0Wp must be defined in Processor Expert
//     <o1> Active State <0=>Low <1=>High
//     <i>  Selects Active State Logical Level
//   </e>
#define RTE_SDHC_WP_PIN_EN              0
#define RTE_SDHC_WP_ACTIVE              1

// </e>

#endif  /* __RTE_DEVICE_H */
