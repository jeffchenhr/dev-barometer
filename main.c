/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>,
 * Copyright (C) 2011 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>
#include <stdio.h>
#include <errno.h>

#include "actuators_pwm_arch.h"
#include "radio_control.h"
#include "imu_aspirin2.h"
#include "baro_ms5611_spi.h"

#include "ahrs.h"
#include "ahrs_aligner.h"

#include "controls.h"
#include "autopilot_arming_throttle.h"
#include "airframe.h"

#include "ahrs_int_cmpl_quat.h"

#include "i2c_arch.h"
#include "gpio_arch.h"

#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f1/adc.h>


#define UNUSED(x) (void)(x)

/******************************************************************************
 * Simple ringbuffer implementation from open-bldc's libgovernor that
 * you can find at:
 * https://github.com/open-bldc/open-bldc/tree/master/source/libgovernor
 *****************************************************************************/

typedef int32_t ring_size_t;
static uint32_t crc32_table[256];
void crc32_init(void);
uint32_t crc32_compute(uint32_t crc, const char *buf, size_t len);

struct ring {
	uint8_t *data;
	ring_size_t size;
	uint32_t begin;
	uint32_t end;

	uint32_t crc;
};

struct traffic_monitor {
	uint32_t tx_counter, rx_counter, tx_last_sec, rx_last_sec;
};

//#define AUTOPILOT_ON() SWITCH_FLAP_ON()

#define RING_SIZE(RING)  ((RING)->size - 1)
#define RING_DATA(RING)  (RING)->data
#define RING_EMPTY(RING) ((RING)->begin == (RING)->end)

int _write(int file, char *ptr, int len);
int _crc_reset(int file, uint32_t crc);
int _crc_write(int file);

void on_rc_frame(void);
void stabilization_attitude_read_rc_setpoint_eulers(struct Int32Eulers *sp, bool_t in_flight);
void position_control_read_rc_setpoint_eulers(struct Int32Eulers *sp, bool_t in_flight);
static inline void send_status_message(void);

static inline void ___disable_irq(void)  { __asm__ volatile ("cpsid i"); }
static inline void ___enable_irq(void)   { __asm__ volatile ("cpsie i"); }

static void position_control_loop_run(struct Int32Eulers *sp, int32_t vicon_ticks_since_last_update);
int32_t position_control_thrust_correction = 0;
static struct Int32Vect3 prev_body_pos_err;
static struct Int32Vect3 prev_vicon_pos_err;
int32_t vicon_control_time_running = -1;
static inline void on_imu_event( void );
//static inline void on_baro_abs_event( void );
//static inline void on_baro_dif_event( void );
//static inline void on_gps_event( void );
struct Int32Eulers global_sp;

struct Int32Rates gyro_integrator;
struct Int32Vect3 acc_integrator;
struct Int32Vect3 mag_integrator;
#define IMU_HIST_SIZE 20
int32_t imu_hist_index = 0;
int32_t acc_x[IMU_HIST_SIZE];
int32_t acc_y[IMU_HIST_SIZE];
int32_t acc_z[IMU_HIST_SIZE];
int32_t gyro_p[IMU_HIST_SIZE];
int32_t gyro_q[IMU_HIST_SIZE];
int32_t gyro_r[IMU_HIST_SIZE];

int integrator_count = 0;
static int mag_in_use = 0;

#define CONTROL_MODE_MANUAL 0
#define CONTROL_MODE_AUTOPILOT 1

#define COMMAND_MODE_TRASHCAN 0
#define COMMAND_MODE_HYBRID 1
#define COMMAND_MODE_RPY_THRUST 2
#define COMMAND_MODE_QUAT_THRUST 3
#define COMMAND_MODE_MOTORS 4
#define COMMAND_MODE_BODY_POSE 5

int32_t control_mode = CONTROL_MODE_MANUAL;
int32_t autopilot_command_mode = COMMAND_MODE_TRASHCAN;
#define ERROR_NONE 0
#define ERROR_ILLEGAL_COMMAND_MODE_SWITCH 1
#define ERROR_WTF 2
#define ERROR_BAD_CRC 15
#define ERROR_UNKNOWN_MSG 16

#define ERROR_SGAINS_SET 32
#define ERROR_MCOEF_SET 33
#define ERROR_PGAINS_SET 34
#define ERROR_AILEVON_SET 35
#define ERROR_GAINS_SET 36


int32_t last_error = ERROR_NONE;
int32_t crc_fails = 0;

struct Int32Rates cmd_rates;
struct Int32HybridCommand cmd_hybrid;
struct Int32MotorCommand cmd_motors;

int32_t fb_commands_u[6];

//Int32AttitudeGains
struct PositionControlGains {
	struct Int32Vect3  p;
	struct Int32Vect3  d;
	struct Int32Vect3  dd;
	struct Int32Vect3  i;
};

struct ControlErrors {
	struct Int32Quat att_err;
	struct Int32Quat att_err_quat;
	struct Int32Vect3 att_err_exp_coords;
	struct Int32Rates rate_err;
	struct Int32Quat sum_err;
};

struct ControlErrors errors;

int32_t posctrl_max_rc_phi = (int32_t) ANGLE_BFP_OF_REAL(0.5 * STABILIZATION_ATTITUDE_SP_MAX_PHI);
int32_t posctrl_max_rc_theta = (int32_t) ANGLE_BFP_OF_REAL(0.5 * STABILIZATION_ATTITUDE_SP_MAX_THETA);
int32_t posctrl_max_rc_r = (int32_t) ANGLE_BFP_OF_REAL(0.5 * STABILIZATION_ATTITUDE_SP_MAX_R);
int32_t posctrl_max_thrust = 100;

int32_t max_rc_roll_phi = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_PHI);
int32_t max_rc_pitch_theta = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_THETA);
int32_t max_rc_yaw_r = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_R);
int32_t max_rc_thrust = 100;

struct PositionControlGains position_control_gains = { { 8, 8, -128}, { 192, 128, -32 }, { 0, 0, 0}, { 0, 0, 0 } };

static uint32_t tick_counter = 0;
int32_t vicon_valid_msg_age = 255;

static void ring_init(struct ring *ring, uint8_t *buf, ring_size_t size) {
	ring->data = buf;
	ring->size = size;
	ring->begin = 0;
	ring->end = 0;
}

static int32_t ring_write_ch(struct ring *ring, uint8_t ch) {
	if (((ring->end + 1) % ring->size) != ring->begin) {
		ring->data[ring->end++] = ch;
		ring->end %= ring->size;

		ring->crc = (ring->crc >> 8) ^ crc32_table[(ring->crc & 0xff) ^ ch];

		return (uint32_t) ch;
	}

	return -1;
}

static int32_t ring_write(struct ring *ring, uint8_t *data, ring_size_t size) {
	int32_t i;

	for (i = 0; i < size; i++) {
		if (ring_write_ch(ring, data[i]) < 0)
			return -i;
	}

	return i;
}

static int32_t ring_read_ch(struct ring *ring, uint8_t *ch) {
	int32_t ret = -1;

	if (ring->begin != ring->end) {
		ret = ring->data[ring->begin++];
		ring->begin %= ring->size;
		if (ch)
			*ch = ret;
	}

	return ret;
}

/* Not used!
 static int32_t ring_read(struct ring *ring, uint8_t *data, ring_size_t size)
 {
 int32_t i;

 for (i = 0; i < size; i++) {
 if (ring_read_ch(ring, data + i) < 0)
 return i;
 }

 return -i;
 }
 */

static int32_t ring_crc_reset(struct ring *ring, uint32_t crc) {
	ring->crc = ~crc;
	return 0;
}

static int32_t ring_crc_write(struct ring *ring) {
	int32_t tmp = (ring->crc);
	return ring_write(ring, (uint8_t*)&tmp, 4);
}

/******************************************************************************
 * The example implementation
 *****************************************************************************/

struct traffic_monitor uart2_traffic;

#define BUFFER_SIZE 512

struct ring output_ring;
uint8_t output_ring_buffer[BUFFER_SIZE];

struct ring output_ring_uart3;
uint8_t output_ring_buffer_uart3[BUFFER_SIZE];

static void clock_setup(void) {
	rcc_clock_setup_in_hse_12mhz_out_72mhz();

	/* Enable GPIOA clock (for LED GPIOs). */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);

	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
			RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN);
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART2EN);

	/* Enable GPIOA, GPIOB, GPIOC, and AFIO clocks. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
}


uint8_t channel_array[4]; /* for injected sampling, 4 channels max, for regular, 16 max */

static void tim4_setup(void)
{
	/* Set up the timer TIM4 for injected sampling */
	uint32_t timer;
	volatile uint32_t *rcc_apbenr;
	uint32_t rcc_apb;

	timer   = TIM4;
	rcc_apbenr = &RCC_APB1ENR;
	rcc_apb = RCC_APB1ENR_TIM4EN;

	rcc_peripheral_enable_clock(rcc_apbenr, rcc_apb);

	/* Time Base configuration */
	timer_reset(timer);
	timer_set_mode(timer, TIM_CR1_CKD_CK_INT,
			TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_period(timer, 0xFF);
	timer_set_prescaler(timer, 0x8);
	timer_set_clock_division(timer, 0x0);
	/* Generate TRGO on every update. */
	timer_set_master_mode(timer, TIM_CR2_MMS_UPDATE);
	timer_enable_counter(timer);
}

static void adc_setup(void)
{
	/* Setup Lisa/M v2 ADC1,2 on ANALOG1 connector */
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG,                                \
			GPIO3 | GPIO0 );

	/* setup TIM4 */
	tim4_setup();

	/* Enable the adc1_2_isr() routine */
	nvic_set_priority(NVIC_ADC1_2_IRQ, 0);
	nvic_enable_irq(NVIC_ADC1_2_IRQ);

	int i;

	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);

	/* Make sure the ADC doesn't run during config. */
	adc_off(ADC1);

	/* We configure everything for one single timer triggered injected conversion with interrupt generation. */
	/* While not needed for a single channel, try out scan mode which does all channels in one sweep and
	 * generates the interrupt/EOC/JEOC flags set at the end of all channels, not each one.
	 */
	adc_enable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	/* We want to start the injected conversion with the TIM4 TRGO */
	adc_enable_external_trigger_injected(ADC1,ADC_CR2_JEXTSEL_TIM4_TRGO);
	/* Generate the ADC1_2_IRQ */
	adc_enable_eoc_interrupt_injected(ADC1);
	adc_set_right_aligned(ADC1);
	/* We want to read the temperature sensor, so we have to enable it. */
	adc_enable_temperature_sensor(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	/* Select the channels we want to convert.
	 * 16=temperature_sensor, 17=Vrefint, 13=ADC1, 10=ADC2
	 */
	channel_array[0] = 16; // temperature
	channel_array[1] = 17; // Vref
	channel_array[2] = 13; // ADC 1 connector
	channel_array[3] = 14; // 10 for ADC 2 connector
	adc_set_injected_sequence(ADC1, 4, channel_array);

	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	for (i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(ADC1);
	while ((ADC_CR2(ADC1) & ADC_CR2_RSTCAL) != 0); //added this check
	adc_calibration(ADC1);
	while ((ADC_CR2(ADC1) & ADC_CR2_CAL) != 0); //added this check
}

uint16_t temperature = 0;
uint16_t v_refint = 0;
uint16_t lisam_adc1 = 0;
uint16_t lisam_adc2 = 0;
uint16_t battery_voltage = 0;

void adc1_2_isr(void)
{
	/* Clear Injected End Of Conversion (JEOC) */
	ADC_SR(ADC1) &= ~ADC_SR_JEOC;
	temperature = adc_read_injected(ADC1,1);
	v_refint = adc_read_injected(ADC1,2);
	lisam_adc1 = adc_read_injected(ADC1,3);
	//    lisam_adc2 = adc_read_injected(ADC1,4);
	uint16_t battery_voltage_tmp = adc_read_injected(ADC1,4);
	battery_voltage = (battery_voltage_tmp * 9) >> 1;
}



//#define TIM_FREQ_1000000 1000000
#define TIM2_FREQ 1000000
#define TIM2_WRAP 50000

/*void tim2_setup(void) {

	// enable TIM6 clock
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);

	// TIM6 configuration
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
			TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	// 100 microseconds ie 0.1 millisecond
	timer_set_period(TIM2, TIM2_WRAP-1);
	timer_set_prescaler(TIM2, ((AHB_CLK / TIM2_FREQ) - 1));

	// Originally I wanted to keep an acurate global us counter using TIM2.
	// However it has proven that because of processor load this is not a good idea:
	// The added accuracy in RPM computation is way below the noise and we are only adding one
	// extra interrupt to the queue...

	// Enable TIM2 interrupts
	//  nvic_set_priority(NVIC_TIM2_IRQ, 2);
	//  nvic_enable_irq(NVIC_TIM2_IRQ);

	// Enable TIM2 Update interrupt
	//  timer_enable_irq(TIM2, TIM_DIER_UIE);
	//  timer_clear_flag(TIM2, TIM_SR_UIF);

	// TIM2 enable counter
	timer_enable_counter(TIM2);
}*/

uint32_t motor_global_tick = 0;
/*void tim2_isr(void) {
	motor_global_tick += TIM2_WRAP;
	timer_clear_flag(TIM2, TIM_SR_UIF);
}*/

/*static void exti_setup(void)
{
	// Enable GPIOA clock.
	//done in clock_setup:
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN);

	// Enable AFIO clock.
	//done in clock_setup:
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);

	// Enable EXTI0 interrupt.
	nvic_enable_irq(NVIC_EXTI9_5_IRQ);

	// Set GPIO7 (MOSI) (in GPIO port A) to 'input float'.
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO7);
	// Set GPIO6 (MISO) (in GPIO port A) to 'input float'.
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO6);
	// Set GPIO5 (SCK) (in GPIO port A) to 'input float'.
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO5);
	// Set GPIO4 (SS) (in GPIO port A) to 'input float'.
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO4);

	// Configure the EXTI subsystem.
	// This function provided by libopencm3 is buggy so doing my own thing here to select GPIOA for EXTI6 and EXTI7:
	// by default they are configured this way after a reset, but just to make sure:
	AFIO_EXTICR2 &= ~0xFF00;
	AFIO_EXTICR2 = 0;

	//exti_select_source(EXTI7, GPIOA);
	//exti_select_source(EXTI6, GPIOA);
	//	exti_set_trigger(EXTI7, EXTI_TRIGGER_RISING);
	exti_set_trigger(EXTI7, EXTI_TRIGGER_BOTH);
	exti_enable_request(EXTI7);
	exti_set_trigger(EXTI6, EXTI_TRIGGER_BOTH);
	exti_enable_request(EXTI6);
	exti_set_trigger(EXTI5, EXTI_TRIGGER_BOTH);
	exti_enable_request(EXTI5);
	exti_set_trigger(EXTI4, EXTI_TRIGGER_BOTH);
	exti_enable_request(EXTI4);
}*/

// the most current rpm estimate
uint32_t motor_rpm[4] = { 0, 0, 0, 0 };
// how many ticks were used for this estimate
uint32_t motor_rpm_confidence[4] = { 0, 0, 0, 0 };

// how many ticks were counted since the last estimate
uint32_t motor_rpm_ticks[4] = { 0, 0, 0, 0 };
// the accumulated tick time since the last estimate
uint32_t motor_tick_acc[4] = { 0, 0, 0, 0 };
// when did the last tick happen
uint32_t motor_last_tick[4] = { 0, 0, 0, 0 };
// state of the encoder lines (bit coded at 3..0)
uint8_t motor_rpm_encoder_state = 0;

#ifdef DEBUG_RPM_CODE
// purely for debugging:
uint32_t motor_rpm_debug[4][32];
uint32_t motor_rpm_mean[4] = { 0, 0, 0, 0 };
uint32_t motor_rpm_std[4] = { 0, 0, 0, 0 };
#endif

void exti9_5_isr(void)
{
	// resetting the counter every time does not seem super accurate but for rpm computation
	// but the effect of time lost is negligible:
	uint16_t delta_tick = TIM_CNT(TIM2);
	TIM_CNT(TIM2) = 0;
	motor_global_tick += delta_tick;

	uint8_t new_encoder_state = (GPIOA_IDR >> 4) & 0x0f;
	uint8_t state_diff = motor_rpm_encoder_state ^ new_encoder_state;
	motor_rpm_encoder_state = new_encoder_state;

	for (int i = 0; i < 4; i++) {
		if ((state_diff & (1 << i)) != 0) {
			uint32_t dt = motor_global_tick - motor_last_tick[i];
			motor_last_tick[i] = motor_global_tick;
			motor_tick_acc[i] += dt;
#ifdef DEBUG_RPM_CODE
			// motor_rpm_debug is for debugging, it allows estimation of the mean and std of tick times:
			motor_rpm_debug[i][motor_rpm_ticks[i]] = dt;
#endif
			motor_rpm_ticks[i]++;
		}
	}
	exti_reset_request(EXTI4 | EXTI5 | EXTI6 | EXTI7);
	//	exti_reset_request(EXTI6);
	//	exti_reset_request(EXTI7);
}

static void usart_setup(void) {
	/* Initialize output ring buffer. */
	ring_init(&output_ring, output_ring_buffer, BUFFER_SIZE);

	/* Enable the USART2 interrupt. */
	nvic_enable_irq(NVIC_USART2_IRQ);

	/* Setup GPIO pin GPIO_USART2_TX on GPIO port A for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

	/* Setup GPIO pin GPIO_USART2_RX on GPIO port A for receive. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200); //MD: 921600, 115200, 230400, 57600
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART2, USART_MODE_TX_RX);

	/* Enable USART2 Receive interrupt. */
	USART_CR1(USART2) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(USART2);


	/* init RCC */
	gpio_enable_clock(GPIO_BANK_USART3_PR_RX);
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_USART3EN);

	// uart 3
	/* Initialize output ring buffer. */
	ring_init(&output_ring_uart3, output_ring_buffer_uart3, BUFFER_SIZE);

	/* Enable the USART3 interrupt. */
	nvic_enable_irq(NVIC_USART3_IRQ);

	/* Setup GPIO pin GPIO_USART2_TX on GPIO port C for transmit. */
	gpio_set_mode(GPIO_BANK_USART3_PR_RX, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);

	/* Setup GPIO pin GPIO_USART2_RX on GPIO port C for receive. */
	gpio_set_mode(GPIO_BANK_USART3_PR_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);

	/* Init GPIOS */
	/* Primary UART Rx pin as floating input */
	gpio_setup_pin_af(GPIO_BANK_USART3_PR_RX, GPIO_USART3_PR_RX, AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP, FALSE);

	/* Setup UART parameters. */
	usart_set_baudrate(USART3, 57600); //MD: 115200, 230400, 57600
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART3, USART_MODE_TX_RX);

	/* Enable USART3 Receive interrupt. */
	USART_CR1(USART3) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(USART3);

}

static void gpio_setup(void) {
	gpio_set(GPIOA, GPIO8);

	/* Setup GPIO8 (in GPIO port A) for LED use. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
			GPIO8);

	// LEDS

	/* LED1 */
	/* Set GPIO8 (in GPIO port A) to 'output push-pull'. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
			GPIO8);

	/* LED2 */
	/* Set GPIO15 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
			GPIO15);

	/* JTAG_TRST */
	/* Set GPIO4 (in GPIO port B) to 'output push-pull'. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
			GPIO4);

	AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_FULL_SWJ_NO_JNTRST;

	/* ADC4 */
	/* Set GPIO5 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
			GPIO5);

	/* ADC6 */
	/* Set GPIO2 (in GPIO port C) to 'output push-pull'. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
			GPIO2);

	/* Preconfigure the LEDs. */
	gpio_set(GPIOA, GPIO8);
	gpio_set(GPIOC, GPIO15);
	gpio_set(GPIOB, GPIO4);
	gpio_set(GPIOC, GPIO5);
	gpio_set(GPIOC, GPIO2);
}

static void led_set(int id, int on) {
	if (on) {
		switch (id) {
		case 0:
			gpio_clear(GPIOA, GPIO8); /* LED1 On */
			break;
		case 1:
			gpio_clear(GPIOB, GPIO4); /* JTAG_TRST On */
			break;
		case 2:
			gpio_clear(GPIOC, GPIO2); /* ADC6 On */
			break;
		case 3:
			gpio_clear(GPIOC, GPIO5); /* ADC4 On */
			break;
		case 4:
			gpio_clear(GPIOC, GPIO15); /* LED2 On */
			break;
		}
	} else {
		switch (id) {
		case 0:
			gpio_set(GPIOA, GPIO8); /* LED1 On */
			break;
		case 1:
			gpio_set(GPIOB, GPIO4); /* JTAG_TRST On */
			break;
		case 2:
			gpio_set(GPIOC, GPIO2); /* ADC6 On */
			break;
		case 3:
			gpio_set(GPIOC, GPIO5); /* ADC4 On */
			break;
		case 4:
			gpio_set(GPIOC, GPIO15); /* LED2 On */
			break;
		}
	}
}

/*void usart2_isr_original(void) {
	// Check if we were called because of RXNE.
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0)
			&& ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

		// Indicate that we got data.
		gpio_toggle(GPIOA, GPIO8);

		// Retrieve the data from the peripheral.
		ring_write_ch(&output_ring, usart_recv(USART2));

		// Enable transmit interrupt so it sends back the data.
		USART_CR1(USART2) |= USART_CR1_TXEIE;
	}

	// Check if we were called because of TXE.
	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0)
			&& ((USART_SR(USART2) & USART_SR_TXE) != 0)) {

		int32_t data;

		data = ring_read_ch(&output_ring, NULL);

		if (data == -1) {
			// Disable the TXE interrupt, it's no longer needed.
			USART_CR1(USART2) &= ~USART_CR1_TXEIE;
		} else {
			// Put data into the transmit register.
			usart_send(USART2, data);
		}
	}
}*/

//static uint32_t crc32_table[256];

void crc32_init(void) {
	uint32_t rem;
	/* Calculate CRC table. */
	for (int i = 0; i < 256; i++) {
		rem = i;  /* remainder from polynomial division */
		for (int j = 0; j < 8; j++) {
			if (rem & 1) {
				rem >>= 1;
				rem ^= 0xedb88320;
			} else
				rem >>= 1;
		}
		crc32_table[i] = rem;
	}
}

uint32_t crc32_compute(uint32_t crc, const char *buf, size_t len)
{
	uint8_t octet;
	const char *p, *q;

	crc = ~crc;
	q = buf + len;
	for (p = buf; p < q; p++) {
		octet = *p;  /* Cast to unsigned octet. */
		crc = (crc >> 8) ^ crc32_table[(crc & 0xff) ^ octet];
	}
	return ~crc;
}

extern int32_t servo_ailevon_left_travel_down;
extern int32_t servo_ailevon_left_travel_up;
extern int32_t servo_ailevon_right_travel_down;
extern int32_t servo_ailevon_right_travel_up;

extern int32_t du_max_down;
extern int32_t du_max_up;
extern int32_t ds_max_down;
extern int32_t ds_max_up;

extern int32_t gyro_lowpass_filter;
int32_t report_raw_gyro = 1;

unsigned char setup_msg[38];
int setup_msgi = -1;

bool_t msg_send_full_state = FALSE;

void usart2_isr(void) {
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0)
			&& ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

		uart2_traffic.rx_counter++;
		/* Indicate that we got data. */
		gpio_toggle(GPIOA, GPIO8);
		// clear error led:
		gpio_set(GPIOB, GPIO4); /* JTAG_TRST On */

		if (setup_msgi < 0) {
			// look for two successive 0xff bytes
			char tmp = usart_recv(USART2);
			if ((tmp == 0xff) && (setup_msg[0] == 0xff)) setup_msgi = 0;
			setup_msg[0] = tmp;
		} else {
			setup_msg[setup_msgi++] = usart_recv(USART2);
			if (setup_msgi == 38) {
				uint32_t crc32 = ~crc32_compute(0xffffffff, (char*)setup_msg, 38);
				if (crc32 == 0) {
					//int32_t msg_type = 256 * setup_msg[1] + setup_msg[0];
					int32_t msg_type = ((setup_msg[1] << 8) & 0xff00) | (setup_msg[0] & 0x00ff);

					int32_t vals[16];

					for (int i = 0; i < 16; i++) {
						int tmp = setup_msg[2*i+3] * 256 + setup_msg[2*i+2];
						if (tmp > 32767) tmp -= 65536;
						vals[i] =  tmp;
					}

					if (msg_type == 0x0001) {
//						stabilization_gains.p.x	 = vals[ 0];
//						stabilization_gains.i.x	 = vals[ 1];
//						stabilization_gains.d.x	 = vals[ 2];
//						stabilization_gains.dd.x = vals[ 3];
//						stabilization_gains.p.y	 = vals[ 4];
//						stabilization_gains.i.y	 = vals[ 5];
//						stabilization_gains.d.y	 = vals[ 6];
//						stabilization_gains.dd.y = vals[ 7];
//						stabilization_gains.p.z	 = vals[ 8];
//						stabilization_gains.i.z	 = vals[ 9];
//						stabilization_gains.d.z	 = vals[10];
//						stabilization_gains.dd.z = vals[11];
						msg_send_full_state = TRUE;
						last_error = ERROR_SGAINS_SET;
					} else if (msg_type == 0x0002) {
//						for (int i = 0; i < 4; i++) {
//							roll_coef[i]   = vals[4*i];
//							pitch_coef[i]  = vals[4*i + 1];
//							yaw_coef[i]    = vals[4*i + 2];
//							thrust_coef[i] = vals[4*i + 3];
//						}
						msg_send_full_state = TRUE;
						last_error = ERROR_MCOEF_SET;
					} else if (msg_type == 0x0003) {
						position_control_gains.p.x	 = vals[ 0];
						position_control_gains.i.x	 = vals[ 1];
						position_control_gains.d.x	 = vals[ 2];
						position_control_gains.dd.x  = vals[ 3];
						position_control_gains.p.y	 = vals[ 4];
						position_control_gains.i.y	 = vals[ 5];
						position_control_gains.d.y	 = vals[ 6];
						position_control_gains.dd.y  = vals[ 7];
						position_control_gains.p.z	 = vals[ 8];
						position_control_gains.i.z	 = vals[ 9];
						position_control_gains.d.z	 = vals[10];
						position_control_gains.dd.z  = vals[11];
						msg_send_full_state = TRUE;
						last_error = ERROR_PGAINS_SET;
					} else if (msg_type == 0x0020) {
						for (int ui = 0; ui < 5; ui++) {
							gain_matrix[ui].p.x = vals[3*ui+0];
							gain_matrix[ui].p.y = vals[3*ui+1];
							gain_matrix[ui].p.z = vals[3*ui+2];
						}
						gain_matrix[5].p.x = vals[12];
						gain_matrix[5].p.y = vals[13];
						gain_matrix[5].p.z = vals[14];
					} else if (msg_type == 0x0021) {
						for (int ui = 0; ui < 5; ui++) {
							gain_matrix[ui].d.x = vals[3*ui+0];
							gain_matrix[ui].d.y = vals[3*ui+1];
							gain_matrix[ui].d.z = vals[3*ui+2];
						}
						gain_matrix[5].d.x = vals[12];
						gain_matrix[5].d.y = vals[13];
						gain_matrix[5].d.z = vals[14];
					} else if (msg_type == 0x0022) {
						for (int ui = 0; ui < 5; ui++) {
							gain_matrix[ui].i.x = vals[3*ui+0];
							gain_matrix[ui].i.y = vals[3*ui+1];
							gain_matrix[ui].i.z = vals[3*ui+2];
						}
						gain_matrix[5].i.x = vals[12];
						gain_matrix[5].i.y = vals[13];
						gain_matrix[5].i.z = vals[14];
					} else if (msg_type == 0x0023) {
						for (int ui = 0; ui < 6; ui++) {
							gain_matrix[ui].t = vals[ui];
						}
					} else if (msg_type == 0x0030) {
						for (int ui = 0; ui < 5; ui++) {
							gain_matrix_manual[ui].p.x = vals[3*ui+0];
							gain_matrix_manual[ui].p.y = vals[3*ui+1];
							gain_matrix_manual[ui].p.z = vals[3*ui+2];
						}
						gain_matrix_manual[5].p.x = vals[12];
						gain_matrix_manual[5].p.y = vals[13];
						gain_matrix_manual[5].p.z = vals[14];
					} else if (msg_type == 0x0031) {
						for (int ui = 0; ui < 5; ui++) {
							gain_matrix_manual[ui].d.x = vals[3*ui+0];
							gain_matrix_manual[ui].d.y = vals[3*ui+1];
							gain_matrix_manual[ui].d.z = vals[3*ui+2];
						}
						gain_matrix_manual[5].d.x = vals[12];
						gain_matrix_manual[5].d.y = vals[13];
						gain_matrix_manual[5].d.z = vals[14];
					} else if (msg_type == 0x0032) {
						for (int ui = 0; ui < 5; ui++) {
							gain_matrix_manual[ui].i.x = vals[3*ui+0];
							gain_matrix_manual[ui].i.y = vals[3*ui+1];
							gain_matrix_manual[ui].i.z = vals[3*ui+2];
						}
						gain_matrix_manual[5].i.x = vals[12];
						gain_matrix_manual[5].i.y = vals[13];
						gain_matrix_manual[5].i.z = vals[14];
					} else if (msg_type == 0x0033) {
						for (int ui = 0; ui < 6; ui++) {
							gain_matrix_manual[ui].t = vals[ui];
						}
					} else if (msg_type == 0x1004) {
						posctrl_max_rc_phi = vals[0];
						posctrl_max_rc_theta = vals[1];
						posctrl_max_rc_r = vals[2];
						posctrl_max_thrust = vals[3];
						msg_send_full_state = TRUE;
					} else if (msg_type == 0x0004) {
						max_rc_roll_phi = vals[0];
						max_rc_pitch_theta = vals[1];
						max_rc_yaw_r = vals[2];
						max_rc_thrust = vals[3];
						msg_send_full_state = TRUE;
					} else if (msg_type == 0x0005) {
						servo_ailevon_left_travel_down = vals[0];
						servo_ailevon_left_travel_up = vals[1];
						servo_ailevon_right_travel_down = vals[2];
						servo_ailevon_right_travel_up = vals[3];
						msg_send_full_state = TRUE;
						last_error = ERROR_AILEVON_SET;
					} else if (msg_type == 0x0006) {
						du_max_down = vals[0];
						du_max_up = vals[1];
						ds_max_down = vals[2];
						ds_max_up = vals[3];
					} else if (msg_type == 0x0007) {
						gyro_lowpass_filter = vals[0];
						report_raw_gyro	= vals[1];
					} else if (msg_type == 0x00C0) { // goal yaw, pitch, roll, thrust mode
						//position_control_read_rc_setpoint_eulers(&global_sp, autopilot_in_flight);
						//stabilization_attitude_read_rc_setpoint_eulers(&global_sp, autopilot_in_flight);
						if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode != COMMAND_MODE_RPY_THRUST)) {
							last_error = ERROR_ILLEGAL_COMMAND_MODE_SWITCH;
						} else {
							if (control_mode == CONTROL_MODE_AUTOPILOT) {
								global_sp.phi = vals[0];
								global_sp.theta = vals[1];
								global_sp.psi = vals[2];
								INT32_QUAT_OF_EULERS(ned_to_body_orientation_goal_quat_i, global_sp);
								stabilization_cmd[COMMAND_THRUST] = vals[3];
							}
							autopilot_command_mode = COMMAND_MODE_RPY_THRUST;
						}
					} else if (msg_type == 0x00C1) { // goal quaternion + thrust mode
						if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode != COMMAND_MODE_QUAT_THRUST)) {
							last_error = ERROR_ILLEGAL_COMMAND_MODE_SWITCH;
						} else {
							if (control_mode == CONTROL_MODE_AUTOPILOT) {
								ned_to_body_orientation_goal_quat_i.qi = vals[0];
								ned_to_body_orientation_goal_quat_i.qx = vals[1];
								ned_to_body_orientation_goal_quat_i.qy = vals[2];
								ned_to_body_orientation_goal_quat_i.qz = vals[3];
								stabilization_cmd[COMMAND_THRUST] = vals[4];
							}
							autopilot_command_mode = COMMAND_MODE_QUAT_THRUST;
						}
					} else if (msg_type == 0x00C2) { // hybrid ctrl message
						if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode != COMMAND_MODE_HYBRID)) {
							last_error = ERROR_ILLEGAL_COMMAND_MODE_SWITCH;
						} else {
							cmd_hybrid.roll   = vals[0];
							cmd_hybrid.pitch  = vals[1];
							cmd_hybrid.yaw    = vals[2];
							cmd_hybrid.thrust = vals[3];
							cmd_hybrid.rates.p = vals[5]; // pitch
							cmd_hybrid.rates.q = vals[4]; // roll
							cmd_hybrid.rates.r = vals[6]; // yaw
							autopilot_command_mode = COMMAND_MODE_HYBRID;
						}
					} else if (msg_type == 0x00C3) { // switch to trashcan mode msg
						if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode != COMMAND_MODE_TRASHCAN)) {
							last_error = ERROR_ILLEGAL_COMMAND_MODE_SWITCH;
						} else {
							autopilot_command_mode = COMMAND_MODE_TRASHCAN;
						}
					} else if (msg_type == 0x00C4) { // direct motor speeds message
						if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode != COMMAND_MODE_MOTORS)) {
							last_error = ERROR_ILLEGAL_COMMAND_MODE_SWITCH;
						} else {
							cmd_motors.u[0]   = vals[0];
							cmd_motors.u[1]   = vals[1];
							cmd_motors.u[2]   = vals[2];
							cmd_motors.u[3]   = vals[3];
							cmd_motors.rates.p = vals[5]; // pitch
							cmd_motors.rates.q = vals[4]; // roll
							cmd_motors.rates.r = vals[6]; // yaw
							cmd_motors.u[4] = vals[7]; // L
							cmd_motors.u[5] = vals[8]; // R

							// QUAT SET ADDED APRIL 9TH 2014:
							if ((vals[9] != 0) || (vals[10] != 0) || (vals[11] != 0) || (vals[12] != 0)) {
								ned_to_body_orientation_goal_quat_i.qi = vals[9];
								ned_to_body_orientation_goal_quat_i.qx = vals[10];
								ned_to_body_orientation_goal_quat_i.qy = vals[11];
								ned_to_body_orientation_goal_quat_i.qz = vals[12];
							}

							autopilot_command_mode = COMMAND_MODE_MOTORS;
						}
					} else if (msg_type == 0x00F0) {
						struct Int32Vect3 vicon_last_pos_i;
						INT32_VECT3_COPY(vicon_last_pos_i, vicon_pos_i);

						vicon_pos_i.x = vals[0];
						vicon_pos_i.y = vals[1];
						vicon_pos_i.z = vals[2];
						vicon_quat_i.qi = vals[3];
						vicon_quat_i.qx = vals[4];
						vicon_quat_i.qy = vals[5];
						vicon_quat_i.qz = vals[6];

						INT32_VECT3_DIFF(vicon_vel_i, vicon_pos_i, vicon_last_pos_i);
						// account for delta_t = 1/120:
						VECT3_SMUL(vicon_vel_i, vicon_vel_i, 120);

						if (ahrs.status != AHRS_UNINIT) {
							ahrs_update_vicon();
						}

						// this is basically replacing on_vicon_frame:
						if (vicon_control_time_running > 0) {
							position_control_loop_run(&global_sp, vicon_valid_msg_age);
							INT32_QUAT_OF_EULERS(ned_to_body_orientation_goal_quat_i, global_sp);
						}
						if ((vicon_quat_i.qi != 0) || (vicon_quat_i.qx != 0) || (vicon_quat_i.qy != 0) || (vicon_quat_i.qz != 0)) {
							vicon_valid_msg_age = 0;
						}
					} else if (msg_type == 0x00F5) { // RIGID BODY POSE / POSE_T
						if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode != COMMAND_MODE_BODY_POSE)) {
							last_error = ERROR_ILLEGAL_COMMAND_MODE_SWITCH;
						} else {
							if (control_mode == CONTROL_MODE_AUTOPILOT) {
								ned_to_body_orientation_goal_quat_i.qi = vals[3];
								ned_to_body_orientation_goal_quat_i.qx = vals[4];
								ned_to_body_orientation_goal_quat_i.qy = vals[5];
								ned_to_body_orientation_goal_quat_i.qz = vals[6];
								if (SWITCH_GEAR_ON()) {
									stabilization_cmd[COMMAND_THRUST] = vals[8];
								}
								cmd_rates.p = vals[ 9];
								cmd_rates.q = vals[10];
								cmd_rates.r = vals[11];
							}
							autopilot_command_mode = COMMAND_MODE_BODY_POSE;
						}
					} else {
						last_error = ERROR_UNKNOWN_MSG;
					}

				} else {
					// Indicate that we got bad data.
//					gpio_toggle(GPIOA, GPIO8);
//					gpio_toggle(GPIOB, GPIO4);
					gpio_clear(GPIOB, GPIO4); /* JTAG_TRST On */
//					gpio_toggle(GPIOC, GPIO2);
//					gpio_toggle(GPIOC, GPIO5);
//					gpio_toggle(GPIOC, GPIO15);
					last_error = ERROR_BAD_CRC;
					crc_fails++;
				}
				setup_msgi = -1;
			}
		}
	} else {
		// check for the lovely overrun error...
		if (USART_SR(USART2) & USART_SR_ORE) {
			uint8_t b = USART_DR(USART2);
			UNUSED(b);
		}
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0)
			&& ((USART_SR(USART2) & USART_SR_TXE) != 0)) {

		int32_t data;

		data = ring_read_ch(&output_ring, NULL);

		if (data == -1) {
			/* Disable the TXE interrupt, it's no longer needed. */
			USART_CR1(USART2) &= ~USART_CR1_TXEIE;
		} else {
			/* Put data into the transmit register. */
			uart2_traffic.tx_counter++;
			usart_send(USART2, data);
		}
	}
}

char vicon_msg[32];
int vicon_msgi = -1;

void usart3_isr(void) {

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0)
			&& ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		//gpio_toggle(GPIOA, GPIO8);

		/*gpio_toggle(GPIOA, GPIO8);
		gpio_toggle(GPIOB, GPIO4);
		gpio_toggle(GPIOC, GPIO2);
		gpio_toggle(GPIOC, GPIO5);
		gpio_toggle(GPIOC, GPIO15);*/

		if (vicon_msgi < 0) {
			// look for two successive 0xff bytes
			char tmp = usart_recv(USART3);
			if ((tmp == 0xff) && (vicon_msg[0] == 0xff)) vicon_msgi = 0;
			vicon_msg[0] = tmp;
		} else {
			vicon_msg[vicon_msgi++] = usart_recv(USART3);
			if (vicon_msgi >= 15) { // 7*2 data (pos, quat) & one byte "crc"
				char crc = vicon_msg[0] + vicon_msg[1] + vicon_msg[2] + vicon_msg[3] + vicon_msg[4] + vicon_msg[5] + vicon_msg[6] + vicon_msg[7];
				crc += vicon_msg[8] + vicon_msg[9] + vicon_msg[10] + vicon_msg[11] + vicon_msg[12] + vicon_msg[13] + vicon_msg[14];
				if (crc == 0) {

					struct Int32Vect3 vicon_last_pos_i;
					INT32_VECT3_COPY(vicon_last_pos_i, vicon_pos_i);

					int32_t tmp;
					tmp = vicon_msg[1] * 256 + vicon_msg[0]; if (tmp > 32767) tmp -= 65536;
					vicon_pos_i.x = tmp;
					tmp = vicon_msg[3] * 256 + vicon_msg[2]; if (tmp > 32767) tmp -= 65536;
					vicon_pos_i.y = tmp;
					tmp = vicon_msg[5] * 256 + vicon_msg[4]; if (tmp > 32767) tmp -= 65536;
					vicon_pos_i.z = tmp;

					tmp = vicon_msg[ 7] * 256 + vicon_msg[ 6]; if (tmp > 32767) tmp -= 65536;
					vicon_quat_i.qi = tmp;
					tmp = vicon_msg[ 9] * 256 + vicon_msg[ 8]; if (tmp > 32767) tmp -= 65536;
					vicon_quat_i.qx = tmp;
					tmp = vicon_msg[11] * 256 + vicon_msg[10]; if (tmp > 32767) tmp -= 65536;
					vicon_quat_i.qy = tmp;
					tmp = vicon_msg[13] * 256 + vicon_msg[12]; if (tmp > 32767) tmp -= 65536;
					vicon_quat_i.qz = tmp;

					INT32_VECT3_DIFF(vicon_vel_i, vicon_pos_i, vicon_last_pos_i);
					// account for delta_t = 1/120:
					VECT3_SMUL(vicon_vel_i, vicon_vel_i, 120);

					if (ahrs.status != AHRS_UNINIT) {
						ahrs_update_vicon();
					}

					// this is basically replacing on_vicon_frame:
					if (vicon_control_time_running > 0) {
						position_control_loop_run(&global_sp, vicon_valid_msg_age);
						INT32_QUAT_OF_EULERS(ned_to_body_orientation_goal_quat_i, global_sp);
					}
					if ((vicon_quat_i.qi != 0) || (vicon_quat_i.qx != 0) || (vicon_quat_i.qy != 0) || (vicon_quat_i.qz != 0)) {
						vicon_valid_msg_age = 0;
					}

				} else {
					gpio_toggle(GPIOA, GPIO8);
					gpio_toggle(GPIOB, GPIO4);
					gpio_toggle(GPIOC, GPIO2);
					gpio_toggle(GPIOC, GPIO5);
					gpio_toggle(GPIOC, GPIO15);
				}
				vicon_msgi = -1;
			}
		}
	} else {
		// check for the lovely overrun error...
		if (USART_SR(USART3) & USART_SR_ORE) {
			uint8_t b = USART_DR(USART3);
			UNUSED(b);
		}
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART3) & USART_CR1_TXEIE) != 0)
			&& ((USART_SR(USART3) & USART_SR_TXE) != 0)) {

		int32_t data;

		data = ring_read_ch(&output_ring_uart3, NULL);

		if (data == -1) {
			/* Disable the TXE interrupt, it's no longer needed. */
			USART_CR1(USART3) &= ~USART_CR1_TXEIE;
		} else {
			/* Put data into the transmit register. */
			usart_send(USART3, data);
		}
	}
}

int _write(int file, char *ptr, int len) {
	int ret;

	if (file == 1) {
		ret = ring_write(&output_ring, (uint8_t *) ptr, len);

		if (ret < 0)
			ret = -ret;

		USART_CR1(USART2) |= USART_CR1_TXEIE;

		return ret;
	}

	if (file == 2) {
		ret = ring_write(&output_ring_uart3, (uint8_t *) ptr, len);

		if (ret < 0)
			ret = -ret;

		USART_CR1(USART3) |= USART_CR1_TXEIE;

		return ret;
	}

	errno = EIO;
	return -1;
}

int _crc_reset(int file, uint32_t crc) {
//	int ret;

	if (file == 1) {
		return ring_crc_reset(&output_ring, crc);
	}

	return -1;
}

int _crc_write(int file) {
//	int ret;

	if (file == 1) {
		return ring_crc_write(&output_ring);
	}

	return -1;
}

static void systick_setup(void) {
	/* 72MHz / 8 => 9000000 counts per second. */
	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8);

	/* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
	/* 9000000/18000 = 500 overflows per second - every 2ms one interrupt */
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(8999); //44999
	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();
}

/*static void led_advance(void) {
	static int state = 0;

	if (state < 5) {
		led_set(state, 1);
	} else if (state < 10) {
		led_set(state - 5, 0);
	} else if (state < 15) {
		led_set(14 - state, 1);
	} else if (state < 20) {
		led_set(19 - state, 0);
	}

	state++;
	if (state == 20)
		state = 0;

}*/

bool_t read_imu = FALSE;
bool_t read_baro = FALSE;
bool_t baro_check = FALSE;
bool_t read_rc = FALSE;
bool_t tick_led_advance = FALSE;
bool_t send_i2c = FALSE;
bool_t tick_send_status_message = FALSE;

int32_t samples_idx_global;

#define MOTOR_RPM_FACTOR (60 * TIM2_FREQ / 14)

void sys_tick_handler(void)
{
	tick_counter++;

	if (tick_counter % 10 == 0) {
		for (int i = 0; i < 4; i++) {
			//motor_rpm[i] += (60 * TIM2_FREQ * motor_rpm_ticks[i]) / (14 * motor_tick_acc[i]);
			if (motor_rpm_ticks[i] == 0) {
				motor_rpm[i] = 0;
			} else {
				motor_rpm[i] = (MOTOR_RPM_FACTOR * motor_rpm_ticks[i]) / (motor_tick_acc[i]);

#ifdef DEBUG_RPM_CODE
				// debugging:
				motor_rpm_mean[i] = motor_tick_acc[i] / motor_rpm_ticks[i];
				uint32_t std = 0;
				for (int k = 0; k < motor_rpm_ticks[i]; k++) {
					std += (motor_rpm_debug[i][k] - motor_rpm_mean[i]) * (motor_rpm_debug[i][k] - motor_rpm_mean[i]);
				}
				uint32_t tmp = std / motor_rpm_ticks[i];
				INT32_SQRT(motor_rpm_std[i], tmp);
#endif
			}
			motor_rpm_confidence[i] = motor_rpm_ticks[i];
			motor_rpm_ticks[i] = 0;
			motor_tick_acc[i] = 0;
		}
	}

	if ((tick_counter > 2000) && (tick_counter % 2 == 0))
		send_i2c = TRUE;

	if (vicon_valid_msg_age < 63) vicon_valid_msg_age++;

	//if (tick_counter % 2 == 0) { /// 500 Hz
	read_imu = TRUE;
	//}

	if (tick_counter % 30 == 0) { /// 500 Hz
		baro_check = TRUE;
		}

	if (tick_counter % 250 == 0) { /// 500 Hz
		read_baro = TRUE;
		}

	if (tick_counter % 16 == 0) { // 60 Hz
		read_rc = TRUE;
	}

	/*
	 * We call this handler every 1ms so we are sending hello world
	 * every 50ms / 20Hz.
	 */
	if (tick_counter % 500 == 0) { // 50 -> 2Hz -> ON/OFF cycle 1/sec
		//led_advance();
		tick_led_advance = TRUE;
	}

	if (tick_counter % 10 == 0) { // 100 Hz
		tick_send_status_message = TRUE;
	}
	/*	}*/
	if (tick_counter % 1000 == 0) {
		uart2_traffic.rx_last_sec = uart2_traffic.rx_counter;
		uart2_traffic.rx_counter = 0;
		uart2_traffic.tx_last_sec = uart2_traffic.tx_counter;
		uart2_traffic.tx_counter = 0;
	}
}

//struct Int32Quat vicon_quat_i;
struct Int32Vect3 vicon_control_point_pos_i;

void on_rc_frame(void) {
	// check if AP is on or off:
	if (SWITCH_FLAP_ON()) {
		control_mode = CONTROL_MODE_AUTOPILOT;
	} else {
		control_mode = CONTROL_MODE_MANUAL;
	}

	if (control_mode == CONTROL_MODE_AUTOPILOT) {
		if (autopilot_command_mode == COMMAND_MODE_TRASHCAN) {
			// TRASHCAN FLIGHT MODE
			// run control loop here
			if (vicon_control_time_running++ < 0) {
				INT32_VECT3_COPY(vicon_control_point_pos_i, vicon_pos_i);

				// compute body pos error to preset for derivate computation
				struct Int32Vect3 vicon_pos_err;
				VECT3_DIFF(vicon_pos_err, vicon_pos_i, vicon_control_point_pos_i);
				INT32_VECT3_BOUND_NORM(prev_vicon_pos_err, vicon_pos_err, 1000);

				struct Int32RMat vicon_to_body_rmat;
				INT32_RMAT_OF_QUAT(vicon_to_body_rmat, ned_to_body_orientation_quat_i);
				struct Int32Vect3 body_pos_err;
				INT32_RMAT_VMULT(body_pos_err, vicon_to_body_rmat, vicon_pos_err);
				INT32_VECT3_BOUND_NORM(prev_body_pos_err, body_pos_err, 1000);

			}
			//if (vicon_control_time_running > 100) vicon_control_time_running = 6;
			position_control_read_rc_setpoint_eulers(&global_sp, autopilot_in_flight);
			//position_control_loop_run(&global_sp, vicon_valid_msg_age); // NORMAL NOT SUPPOSED TO BE HERE!
			INT32_QUAT_OF_EULERS(ned_to_body_orientation_goal_quat_i, global_sp);
		} else {
			// HYBRID COMMAND, QUAT+THRUST, RPY+THRUST
			// do nothing as we assume all commands come from the serial port
			vicon_control_time_running = -1;
		}
	} else {
		// MANUAL FLIGHT MODE
		vicon_control_time_running = -1;
		stabilization_attitude_read_rc_setpoint_eulers(&global_sp, autopilot_in_flight);
		INT32_QUAT_OF_EULERS(ned_to_body_orientation_goal_quat_i, global_sp);
	}
}

#define WRITE_QUAT_FULL_RES(_q) { \
		_write(1, (char*) &((_q).qi), 4); \
		_write(1, (char*) &((_q).qx), 4); \
		_write(1, (char*) &((_q).qy), 4); \
		_write(1, (char*) &((_q).qz), 4); \
}

//extern struct Int32Quat residualq;
//extern struct Int32Quat residualqfromacc;

uint16_t data_id_global = 0;

static inline uint16_t get_debug_data(uint16_t data_id) {
	uint16_t val;
	switch (data_id) {
	case   0 : // stabilization gains depracated
	case   1 :
	case   2 :
	case   3 :
	case   4 :
	case   5 :
	case   6 :
	case   7 :
	case   8 :
	case   9 :
	case  10 :
	case  11 :
	case  12 :
	case  13 :
	case  14 :
	case  15 :
	case  16 :
	case  17 :
	case  18 :
	case  19 :
	case  20 :
	case  21 :
	case  22 :
	case  23 :
	case  24 :
	case  25 :
	case  26 :
	case  27 : val = 0; break;
	case  28 : val = position_control_gains.i.x; break;
	case  29 : val = position_control_gains.d.x; break;
	case  30 : val = position_control_gains.dd.x; break;
	case  31 : val = position_control_gains.p.x; break;
	case  32 : val = position_control_gains.i.y; break;
	case  33 : val = position_control_gains.d.y; break;
	case  34 : val = position_control_gains.dd.y; break;
	case  35 : val = position_control_gains.p.y; break;
	case  36 : val = position_control_gains.i.z; break;
	case  37 : val = position_control_gains.d.z; break;
	case  38 : val = position_control_gains.dd.z; break;
	case  39 : val = position_control_gains.p.z; break;
	case  40 : val = max_rc_roll_phi; break; //posctrl_max_rc_phi
	case  41 : val = max_rc_pitch_theta; break; //posctrl_max_rc_theta
	case  42 : val = max_rc_yaw_r; break; //posctrl_max_rc_r
	case  43 : val = max_rc_thrust; break; //posctrl_max_thrust
	case  44 : val = servo_ailevon_left_travel_down; break;
	case  45 : val = servo_ailevon_left_travel_up; break;
	case  46 : val = servo_ailevon_right_travel_down; break;
	case  47 : val = servo_ailevon_right_travel_up; break;

	case  48 : val = cmd_hybrid.roll; break;
	case  49 : val = cmd_hybrid.pitch; break;
	case  50 : val = cmd_hybrid.yaw; break;
	case  51 : val = cmd_hybrid.thrust; break;
	case  52 : val = cmd_hybrid.rates.p; break;
	case  53 : val = cmd_hybrid.rates.q; break;
	case  54 : val = cmd_hybrid.rates.r; break;
	case  55 : val = crc_fails; break;
	case  56 : val = imu.body_to_imu_rmat.m[0]; break;
	case  57 : val = imu.body_to_imu_rmat.m[1]; break;
	case  58 : val = imu.body_to_imu_rmat.m[2]; break;
	case  59 : val = imu.body_to_imu_rmat.m[3]; break;
	case  60 : val = imu.body_to_imu_rmat.m[4]; break;
	case  61 : val = imu.body_to_imu_rmat.m[5]; break;
	case  62 : val = imu.body_to_imu_rmat.m[6]; break;
	case  63 : val = imu.body_to_imu_rmat.m[7]; break;
	case  64 : val = imu.body_to_imu_rmat.m[8]; break;
	case  65 : val = imu.body_to_imu_quat.qi; break;
	case  66 : val = imu.body_to_imu_quat.qx; break;
	case  67 : val = imu.body_to_imu_quat.qy; break;
	case  68 : val = imu.body_to_imu_quat.qz; break;
	case  69 : val = du_max_down; break;
	case  70 : val = du_max_up; break;
	case  71 : val = ds_max_down; break;
	case  72 : val = ds_max_up; break;
	case  73 : val = gyro_lowpass_filter; break;
	case  74 : val = report_raw_gyro; break;
	case  75 :
	case  76 :
	case  77 :
	case  78 :
	case  79 :
	case  80 : if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode == COMMAND_MODE_MOTORS))
	{
		val = gain_matrix[data_id - 75].p.x;
	} else {
		val = gain_matrix_manual[data_id - 75].p.x;
	} break;
	case  81 :
	case  82 :
	case  83 :
	case  84 :
	case  85 :
	case  86 : if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode == COMMAND_MODE_MOTORS))
	{
		val = gain_matrix[data_id - 81].p.y; break;
	} else {
		val = gain_matrix_manual[data_id - 81].p.y; break;
	} break;
	case  87 :
	case  88 :
	case  89 :
	case  90 :
	case  91 :
	case  92 : if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode == COMMAND_MODE_MOTORS))
	{
		val = gain_matrix[data_id - 87].p.z; break;
	} else {
		val = gain_matrix_manual[data_id - 87].p.z; break;
	} break;
	case  93 :
	case  94 :
	case  95 :
	case  96 :
	case  97 :
	case  98 : if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode == COMMAND_MODE_MOTORS))
	{
		val = gain_matrix[data_id - 93].d.x; break;
	} else {
		val = gain_matrix_manual[data_id - 93].d.x; break;
	} break;
	case  99 :
	case 100 :
	case 101 :
	case 102 :
	case 103 :
	case 104 : if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode == COMMAND_MODE_MOTORS))
	{
		val = gain_matrix[data_id - 99].d.y; break;
	} else {
		val = gain_matrix_manual[data_id - 99].d.y; break;
	} break;
	case 105 :
	case 106 :
	case 107 :
	case 108 :
	case 109 :
	case 110 : if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode == COMMAND_MODE_MOTORS))
	{
		val = gain_matrix[data_id - 105].d.z; break;
	} else {
		val = gain_matrix_manual[data_id - 105].d.z; break;
	} break;
	case 111 :
	case 112 :
	case 113 :
	case 114 :
	case 115 :
	case 116 : if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode == COMMAND_MODE_MOTORS))
	{
		val = gain_matrix[data_id - 111].i.x; break;
	} else {
		val = gain_matrix_manual[data_id - 111].i.x; break;
	} break;
	case 117 :
	case 118 :
	case 119 :
	case 120 :
	case 121 :
	case 122 : if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode == COMMAND_MODE_MOTORS))
	{
		val = gain_matrix[data_id - 117].i.y; break;
	} else {
		val = gain_matrix_manual[data_id - 117].i.y; break;
	} break;
	case 123 :
	case 124 :
	case 125 :
	case 126 :
	case 127 :
	case 128 : if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode == COMMAND_MODE_MOTORS))
	{
		val = gain_matrix[data_id - 123].i.z; break;
	} else {
		val = gain_matrix_manual[data_id - 123].i.z; break;
	} break;
	case 129 :
	case 130 :
	case 131 :
	case 132 :
	case 133 :
	case 134 : if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode == COMMAND_MODE_MOTORS))
	{
		val = gain_matrix[data_id - 129].t; break;
	} else {
		val = gain_matrix_manual[data_id - 129].t; break;
	} break;
	case 135 : val = uart2_traffic.rx_last_sec; break;
	case 136 : val = uart2_traffic.tx_last_sec; break;
	case 137 : val = 0; break;
	default : val = 0;
	}
	return val;
}


static inline void send_status_message(void) {

    if (tick_counter % 50==0){
    
	_crc_reset(1, 0xffffffff);
	_write(1, (char*) &tick_counter, 4);

/*	struct Int32Vect3 tmp_acc;
	VECT3_SDIV(tmp_acc, acc_integrator, integrator_count);
	VECT3_ASSIGN(acc_integrator, 0, 0, 0);
	struct Int32Vect3 tmp_mag;
	VECT3_SDIV(tmp_mag, mag_integrator, integrator_count);
	VECT3_ASSIGN(mag_integrator, 0, 0, 0);
	struct Int32Rates tmp_gyro;
	RATES_SDIV(tmp_gyro, gyro_integrator, integrator_count);
	RATES_ASSIGN(gyro_integrator, 0, 0, 0);
	integrator_count = 0;

	_write(1, (char*) &tmp_mag.x, 2);
	_write(1, (char*) &tmp_mag.y, 2);
	_write(1, (char*) &tmp_mag.z, 2);

	if (report_raw_gyro) {
		_write(1, (char*) &tmp_gyro.p, 2);
		_write(1, (char*) &tmp_gyro.q, 2);
		_write(1, (char*) &tmp_gyro.r, 2);
	} else {
		_write(1, (char*) &ahrs_impl.imu_rate.p, 2);
		_write(1, (char*) &ahrs_impl.imu_rate.q, 2);
		_write(1, (char*) &ahrs_impl.imu_rate.r, 2);
	}
	_write(1, (char*) &tmp_acc.x, 2);
	_write(1, (char*) &tmp_acc.y, 2);
	_write(1, (char*) &tmp_acc.z, 2); // 22
*/

	_write(1, (char*) &(ned_to_body_orientation_quat_i.qi), 2);
	_write(1, (char*) &(ned_to_body_orientation_quat_i.qx), 2);
	_write(1, (char*) &(ned_to_body_orientation_quat_i.qy), 2);
	_write(1, (char*) &(ned_to_body_orientation_quat_i.qz), 2); // 46

	_write(1, (char*) &(ned_to_body_orientation_goal_quat_i.qi), 2);
	_write(1, (char*) &(ned_to_body_orientation_goal_quat_i.qx), 2);
	_write(1, (char*) &(ned_to_body_orientation_goal_quat_i.qy), 2);
	_write(1, (char*) &(ned_to_body_orientation_goal_quat_i.qz), 2); // 50
    
    _write(1, (char*) &actuators_pwm_values[0], 2);
	_write(1, (char*) &actuators_pwm_values[1], 2);
	_write(1, (char*) &actuators_pwm_values[2], 2);
	_write(1, (char*) &actuators_pwm_values[3], 2);
	_write(1, (char*) &actuators_pwm_values[4], 2);
	_write(1, (char*) &actuators_pwm_values[5], 2); // 34
    
/*
	_write(1, (char*) &(vicon_pos_i.x), 2);
	_write(1, (char*) &(vicon_pos_i.y), 2);
	_write(1, (char*) &(vicon_pos_i.z), 2); // 56

#ifdef DEBUG_RPM_CODE
	_write(1, (char*) &motor_rpm_mean[2], 2);
	_write(1, (char*) &motor_rpm_std[2], 2);
	_write(1, (char*) &motor_rpm_mean[3], 2);
	_write(1, (char*) &motor_rpm_std[3], 2);
#else
	//_write(1, (char*) &tmp, 2);
//	_write(1, (char*) &motor_rpm[0], 2);
//	_write(1, (char*) &motor_rpm[1], 2);
//	_write(1, (char*) &motor_rpm[2], 2);
//	_write(1, (char*) &motor_rpm[3], 2); // 64
	_write(1, (char*) &fb_commands_u[0], 2);
	_write(1, (char*) &fb_commands_u[1], 2);
	_write(1, (char*) &fb_commands_u[4], 2);
	_write(1, (char*) &fb_commands_u[5], 2); // 64
#endif



	_write(1, (char*) &battery_voltage, 2); // 66

	int32_t control_mode_status = ((control_mode & 0x0f) << 4) | (autopilot_command_mode & 0x0f);
	_write(1, (char*) &control_mode_status, 1); // +1
	_write(1, (char*) &last_error, 1); // +1
	_write(1, "XX", 2); // 74 - 4

	_write(1, (char*) &data_id_global, 2);
	uint16_t val = get_debug_data(data_id_global);
	_write(1, (char*) &val, 2); // +4
	val = get_debug_data(data_id_global+1);
	_write(1, (char*) &val, 2); //
	val = get_debug_data(data_id_global+2);
	_write(1, (char*) &val, 2); // +4
	data_id_global = (data_id_global + 3) % 150; // 0.5*3 Hz
*/

	uint16_t status = (radio_control.status & 0x03); // radio working?
	status         |= (ahrs_aligner.status & 0x03) << 2; // status of the estimation system aligner?
	status         |= (ahrs.status & 0x01) << 4; // how is our estimation system doing?
	status         |= (autopilot_arming_state & 0x07) << 5; // are motors armed and ready to go?
	status         |= (autopilot_motors_on & 0x01) << 8; // motors on?
	if (vicon_control_time_running > 0)
		status     |= 1 << 9;
	if (SWITCH_GEAR_ON())
		status     |= 1 << 10;
	if (SWITCH_ELEVDR_ON())
		status     |= 1 << 11;
	if (SWITCH_FLAP_ON())
		status     |= 1 << 12;
	if (SWITCH_AILDR_ON())
		status     |= 1 << 13;
	status         |= ((vicon_valid_msg_age >> 4) & 0x03) << 14;

	_write(1, (char*) &(status), 2); // 76
	_write(1, (char*) &(baro_ms5611.data.pressure),2);
        
/*      _write(1, (char*) &(stabilization_att_sum_err.phi), 2);
        _write(1, (char*) &(stabilization_att_sum_err.theta), 2);
        _write(1, (char*) &(stabilization_att_sum_err.psi), 2); // 46
*/
/*
    _write(1, (char*) &(errors.rate_err.p), 2);
    _write(1, (char*) &(errors.rate_err.q), 2);
    _write(1, (char*) &(errors.rate_err.r), 2); // 40
        
    _write(1, (char*) &(errors.att_err.qx), 2);
    _write(1, (char*) &(errors.att_err.qy), 2);
    _write(1, (char*) &(errors.att_err.qz), 2); // 46

    _write(1, (char*) &fb_commands_u[0], 2);
	_write(1, (char*) &fb_commands_u[1], 2);
	_write(1, (char*) &fb_commands_u[2], 2);
	_write(1, (char*) &fb_commands_u[3], 2);
	_write(1, (char*) &fb_commands_u[4], 2);
	_write(1, (char*) &fb_commands_u[5], 2); // ++12
        
    _write(1, (char*) &radio_control.values[RADIO_ROLL], 2);
    _write(1, (char*) &radio_control.values[RADIO_PITCH], 2);
    _write(1, (char*) &radio_control.values[RADIO_YAW], 2);

 ////wireserial
 
	// !!!! DROPPING FACTOR OF 2 !!!! -> effective BPF of INT32_TRIG_FRAC-1 = 13

	_write(1, (char*) &(stabilization_cmd[COMMAND_ROLL]), 2);
	_write(1, (char*) &(stabilization_cmd[COMMAND_PITCH]), 2);
	_write(1, (char*) &(stabilization_cmd[COMMAND_YAW]), 2);
	_write(1, (char*) &(stabilization_cmd[COMMAND_THRUST]), 2);

	_write(1, (char*) &(cmd_rates.p), 2);
	_write(1, (char*) &(cmd_rates.q), 2);
	_write(1, (char*) &(cmd_rates.r), 2); // +6

	_write(1, (char*) &(cmd_motors.u[0]), 2);
	_write(1, (char*) &(cmd_motors.u[1]), 2);
	_write(1, (char*) &(cmd_motors.u[2]), 2);
	_write(1, (char*) &(cmd_motors.u[3]), 2);
	_write(1, (char*) &(cmd_motors.u[4]), 2);
	_write(1, (char*) &(cmd_motors.u[5]), 2); // ++ 12


	_write(1, (char*) &(errors.rate_err.p), 2);
	_write(1, (char*) &(errors.rate_err.q), 2);
	_write(1, (char*) &(errors.rate_err.r), 2); // ++6

	_write(1, (char*) &(errors.att_err_quat.qx), 2);
	_write(1, (char*) &(errors.att_err_quat.qy), 2);
	_write(1, (char*) &(errors.att_err_quat.qz), 2); // ++6

	int exptmp = errors.att_err_exp_coords.x / 2;
	_write(1, (char*) &(exptmp), 2);
	exptmp = errors.att_err_exp_coords.y / 2;
	_write(1, (char*) &(exptmp), 2);
	exptmp = errors.att_err_exp_coords.z / 2;
	_write(1, (char*) &(exptmp), 2); // ++6

	_write(1, (char*) &fb_commands_u[0], 2);
	_write(1, (char*) &fb_commands_u[1], 2);
	_write(1, (char*) &fb_commands_u[2], 2);
	_write(1, (char*) &fb_commands_u[3], 2);
	_write(1, (char*) &fb_commands_u[4], 2);
	_write(1, (char*) &fb_commands_u[5], 2); // ++12


	_write(1, (char*) &imu_hist_index, 2); // +2
	for (int i = 0; i < 10; i++) {
		_write(1, (char*) &acc_x[i], 2);
		_write(1, (char*) &acc_y[i], 2);
		_write(1, (char*) &acc_z[i], 2);
		_write(1, (char*) &gyro_p[i], 2);
		_write(1, (char*) &gyro_q[i], 2);
		_write(1, (char*) &gyro_r[i], 2); // +12
	}*/
	// +122!!

	_crc_write(1); // +4
	_write(1, "    ", 4); // 80

//	_crc_reset(1, 0xffffffff);
//
//	_crc_write(1); // +4
//	_write(1, "    ", 4); // 80
    }
    imu_hist_index = 0;
}

#define APP_ADDRESS	0x08002000

struct i2c_transaction i2c_trans;

int main(void) {

	/* Set vector table base address. */
	SCB_VTOR = APP_ADDRESS & 0xFFFF;
	/* Initialise master stack pointer. */
	//asm volatile("msr msp, %0"::"g"
	//		     (*(volatile uint32_t *)APP_ADDRESS));
	/* Jump to application. */
	//(*(void (**)())(APP_ADDRESS + 4))();

	clock_setup();
	gpio_setup();
	usart_setup();
	spi2_init();
	spi_init_slaves();
	actuators_pwm_arch_init();
	radio_control_init();
	//radio_control_spektrum_try_bind();
	imu_init();
	baro_ms5611_init();
	systick_setup();
	//// REANABLE FOR RPM COUNTER:
	////exti_setup();
	////tim2_setup();

	adc_setup();

#ifdef USE_I2C1
	i2c1_init();
	i2c_setbitrate(&i2c1, 400000);
#endif

	ahrs_init();
	ahrs_aligner_init();

	autopilot_arming_init();

	crc32_init();

	QUAT_ASSIGN(ned_to_body_orientation_goal_quat_i, 1, 0, 0, 0);
	INT32_QUAT_NORMALIZE(ned_to_body_orientation_goal_quat_i);

	led_set(0, 0); // 1
	led_set(1, 1); // 1 <-- error led to indicate no data yet
	led_set(2, 0); // 1
	led_set(3, 0);
	led_set(4, 1); // 1

	int dometick = 0;
	while (1) {
		if (read_imu) {
			read_imu = FALSE;
			imu_periodic();
		}
		if (baro_check) {
			baro_check = FALSE;
			baro_ms5611_periodic_check();
				}
		if (read_rc) {
			read_rc = FALSE;
			radio_control_periodic_task();
		}
		if (tick_led_advance) {
			tick_led_advance = FALSE;
			gpio_toggle(GPIOC, GPIO15);

		}
		if (tick_send_status_message) {
			send_status_message();
			tick_send_status_message = FALSE;
		}

		switch (dometick++ % 4) {
		case 0 : 	RadioControlEventImp();
		break;
		case 1 :	autopilot_check_in_flight(autopilot_motors_on);
		break;
		case 2 :	/* an arming sequence is used to start/stop motors */
			if ((autopilot_arming_state == STATE_MOTORS_ON) && (!(radio_control.status == RC_OK) || !(ahrs_aligner.status == AHRS_ALIGNER_LOCKED))) {
				autopilot_arming_set(FALSE);
			}
			autopilot_arming_check_motors_on();
			break;
		case 3 : 	//ImuEvent(on_gyro_event, on_accel_event, on_mag_event);
			ImuEvent(on_imu_event);
			baro_ms5611_event();
			break;
		case 4:		i2c_event();
		break;
		default:	break;
		}

	}

	return 0;
}

bool_t autopilot_motors_on;

bool_t autopilot_in_flight;
uint32_t autopilot_in_flight_counter;
uint16_t autopilot_flight_time;

#ifdef STABILIZATION_ATTITUDE_DEADBAND_A
#define ROLL_DEADBAND_EXCEEDED()                                        \
		(radio_control.values[RADIO_ROLL] >  STABILIZATION_ATTITUDE_DEADBAND_A || \
				radio_control.values[RADIO_ROLL] < -STABILIZATION_ATTITUDE_DEADBAND_A)
#else
#define ROLL_DEADBAND_EXCEEDED() (TRUE)
#endif /* STABILIZATION_ATTITUDE_DEADBAND_A */

#ifdef STABILIZATION_ATTITUDE_DEADBAND_E
#define PITCH_DEADBAND_EXCEEDED()                                       \
		(radio_control.values[RADIO_PITCH] >  STABILIZATION_ATTITUDE_DEADBAND_E || \
				radio_control.values[RADIO_PITCH] < -STABILIZATION_ATTITUDE_DEADBAND_E)
#else
#define PITCH_DEADBAND_EXCEEDED() (TRUE)
#endif /* STABILIZATION_ATTITUDE_DEADBAND_E */

#define YAW_DEADBAND_EXCEEDED()                                         \
		(radio_control.values[RADIO_YAW] >  STABILIZATION_ATTITUDE_DEADBAND_R || \
				radio_control.values[RADIO_YAW] < -STABILIZATION_ATTITUDE_DEADBAND_R)


#ifndef RC_UPDATE_FREQ
#define RC_UPDATE_FREQ 40
#endif

/** Read attitude setpoint from RC as euler angles.
 * @param[in]  in_flight  true if in flight
 * @param[out] sp         attitude setpoint as euler angles
 */
void stabilization_attitude_read_rc_setpoint_eulers(struct Int32Eulers *sp, bool_t in_flight) {

	sp->phi = (int32_t) ((radio_control.values[RADIO_ROLL] * max_rc_roll_phi) /  MAX_PPRZ);
	sp->theta = (int32_t) ((radio_control.values[RADIO_PITCH] * max_rc_pitch_theta) /  MAX_PPRZ);

	if (in_flight) {
		if (YAW_DEADBAND_EXCEEDED()) {
			sp->psi += (int32_t) ((radio_control.values[RADIO_YAW] * max_rc_yaw_r) /  MAX_PPRZ / RC_UPDATE_FREQ);
			INT32_ANGLE_NORMALIZE(sp->psi);
		}
		/*if (autopilot_mode == AP_MODE_FORWARD) {
      //Coordinated turn
      //feedforward estimate angular rotation omega = g*tan(phi)/v
      //Take v = 9.81/1.3 m/s
      int32_t omega;
      const int32_t max_phi = ANGLE_BFP_OF_REAL(RadOfDeg(85.0));
      if(abs(sp->phi) < max_phi)
        omega = ANGLE_BFP_OF_REAL(1.3*tanf(ANGLE_FLOAT_OF_BFP(sp->phi)));
      else //max 60 degrees roll, then take constant omega
        omega = ANGLE_BFP_OF_REAL(1.3*1.72305* ((sp->phi > 0) - (sp->phi < 0)));

      sp->psi += omega/RC_UPDATE_FREQ;
    }*/
#ifdef STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT
		// Make sure the yaw setpoint does not differ too much from the real yaw
		// to prevent a sudden switch at 180 deg
		const int32_t delta_limit = ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_PSI_DELTA_LIMIT);

		int32_t heading = stabilization_attitude_get_heading_i();

		int32_t delta_psi = sp->psi - heading;
		INT32_ANGLE_NORMALIZE(delta_psi);
		if (delta_psi > delta_limit){
			sp->psi = heading + delta_limit;
		}
		else if (delta_psi < -delta_limit){
			sp->psi = heading - delta_limit;
		}
		INT32_ANGLE_NORMALIZE(sp->psi);
#endif
	}
	else { /* if not flying, use current yaw as setpoint */

		struct Int32Eulers tmp;
		INT32_EULERS_OF_QUAT(tmp, ned_to_body_orientation_quat_i);

		sp->psi = tmp.psi;
	}
    
    struct Int32Eulers tmp;
    INT32_EULERS_OF_QUAT(tmp, ned_to_body_orientation_quat_i);
    
    sp->psi = tmp.psi;
}

#define MAX_SUM_ERR RadOfDeg(56000)

static void stabilization_attitude_run_new(bool_t enable_integrator, bool_t enable_p, int32_t thrust_cmd, struct Int32AttitudeGainMatrixRow gains[]) {


	//struct Int32Quat tmp_quat_i;

	//	INT32_QUAT_NORMALIZE(ned_to_body_orientation_goal_quat_i);

	//	INT32_EULERS_OF_QUAT(_e, _q);

	/*
	 * Update reference
	 */
	//stabilization_attitude_ref_update();
	INT32_QUAT_NORMALIZE(ned_to_body_orientation_goal_quat_i);

	/*
	 * Compute errors for feedback
	 */

	/* attitude error                          */
	struct Int32Quat att_err;
	//struct Int32Quat* att_quat = stateGetNedToBodyQuat_i();
	//INT32_QUAT_INV_COMP(att_err, *att_quat, stab_att_ref_quat);
	INT32_QUAT_INV_COMP(att_err, ned_to_body_orientation_quat_i, ned_to_body_orientation_goal_quat_i);
	/* wrap it in the shortest direction       */
	INT32_QUAT_WRAP_SHORTEST(att_err);
	INT32_QUAT_NORMALIZE(att_err);

	/* convert quat error to exp coord error */
	//error_exp_coords
	int32_t y;
	int32_t x = att_err.qi;
	INT32_QUAT_VEC_NORM(y, att_err);
	float fx = (float)x / (1<<INT32_TRIG_FRAC);
	float fy = (float)y / (1<<INT32_TRIG_FRAC);
	float atan2norm;
	if ((y > -64) && (y < 64)) {
		atan2norm = (1 - fy * fy / (3*fx*fx))/fx;
	} else {
		atan2norm = atanf(fy/fx)/fy;
	}
	int32_t at2n = (2.0 * atan2norm)*(1<<INT32_TRIG_FRAC);
	// DROP FACTOR OF 2: int32_t at2n = (atan2norm)*(1<<INT32_TRIG_FRAC);
	struct Int32Vect3 error_exp_coords;
	error_exp_coords.x = INT_MULT_RSHIFT(at2n, att_err.qx, INT32_TRIG_FRAC);
	error_exp_coords.y = INT_MULT_RSHIFT(at2n, att_err.qy, INT32_TRIG_FRAC);
	error_exp_coords.z = INT_MULT_RSHIFT(at2n, att_err.qz, INT32_TRIG_FRAC);


	QUAT_COPY(errors.att_err_quat, att_err);
	VECT3_COPY(errors.att_err_exp_coords, error_exp_coords);

	if (SWITCH_ELEVDR_ON()) {
		// quaternions are BFP 15 and exp_coords BFP 14:
		att_err.qx = 2 * error_exp_coords.x;
		att_err.qy = 2 * error_exp_coords.y;
		att_err.qz = 2 * error_exp_coords.z;
	}

	/*  rate error                */
	struct Int32Rates rate_err;
	RATES_DIFF(rate_err, cmd_rates, body_rates_i);

	/* integrated error */
	if (enable_integrator && autopilot_in_flight) {
		struct Int32Quat new_sum_err, scaled_att_err;
        struct Int32Eulers new_att_sum_err;
		/* update accumulator */
		scaled_att_err.qi = att_err.qi;
		scaled_att_err.qx = att_err.qx / IERROR_SCALE;
		scaled_att_err.qy = att_err.qy / IERROR_SCALE;
		scaled_att_err.qz = att_err.qz / IERROR_SCALE;
		INT32_QUAT_COMP(new_sum_err, stabilization_att_sum_err_quat, scaled_att_err);
		INT32_QUAT_NORMALIZE(new_sum_err);
		QUAT_COPY(stabilization_att_sum_err_quat, new_sum_err);
		INT32_EULERS_OF_QUAT(new_att_sum_err, att_err);
        EULERS_ADD(stabilization_att_sum_err,new_att_sum_err);
        EULERS_BOUND_CUBE(stabilization_att_sum_err, -30000, 30000);
	} else {
		/* reset accumulator */
		INT32_QUAT_ZERO( stabilization_att_sum_err_quat );
		INT_EULERS_ZERO( stabilization_att_sum_err );
	}

	/* compute the feed forward command */
	// attitude_run_ff(stabilization_att_ff_cmd, &stabilization_gains, &stab_att_ref_accel);

	/* compute the feed back command */
	//INT32_QUAT_ZERO( att_err );

	if (!enable_p) {
		QUAT_ASSIGN(att_err, 1, 0, 0, 0);
		INT32_QUAT_NORMALIZE(att_err);
	}

	struct Int32Quat sum_err;
	QUAT_COPY(sum_err, stabilization_att_sum_err_quat);


	QUAT_COPY(errors.sum_err, sum_err);
	QUAT_COPY(errors.att_err, att_err);
	RATES_COPY(errors.rate_err, rate_err);

	for (int i = 0; i < 6; i++) {
		/*  PID feedback combined with motor map */
         fb_commands_u[i] = ((gains[i].p.x  * att_err.qx +
        		 	 	 	 	 gains[i].p.y  * att_err.qy +
        		 	 	 	 	 gains[i].p.z  * att_err.qz) / 8 +
                             (gains[i].d.x  * rate_err.p +
                              gains[i].d.y  * rate_err.q +
                              gains[i].d.z  * rate_err.r) +
					         (gains[i].i.x  * stabilization_att_sum_err.phi +
					          gains[i].i.y  * stabilization_att_sum_err.theta +
					          gains[i].i.z  * stabilization_att_sum_err.psi) / 8) / (1 << 12) +
					          gains[i].t * thrust_cmd +
					          gains[i].offset;
//         if (fb_commands_u[i] < 0) fb_commands_u[i] = 0;
//         if (fb_commands_u[i] > 10000) fb_commands_u[i] = 10000;
	}
}

void position_control_read_rc_setpoint_eulers(struct Int32Eulers *sp, bool_t in_flight) {
	const int32_t max_rc_phi = 2000;
	const int32_t max_rc_theta = 2000;
	const int32_t max_rc_r = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_R);

	if ((radio_control.values[RADIO_ROLL] >  250) || (radio_control.values[RADIO_ROLL] < -250)) {
		if (SWITCH_ELEVDR_ON()) {
			vicon_control_point_pos_i.x -= (int32_t) ((radio_control.values[RADIO_ROLL] * max_rc_phi * 2) /  MAX_PPRZ / RC_UPDATE_FREQ);
		} else {
			vicon_control_point_pos_i.x -= (int32_t) ((radio_control.values[RADIO_ROLL] * max_rc_phi) /  MAX_PPRZ / RC_UPDATE_FREQ);
		}
	}
	if ((radio_control.values[RADIO_PITCH] >  250) || (radio_control.values[RADIO_PITCH] < -250)) {
		if (SWITCH_FLAP_ON()) {
			if (SWITCH_ELEVDR_ON()) {
				vicon_control_point_pos_i.z -= (int32_t) ((radio_control.values[RADIO_PITCH] * max_rc_theta * 2) /  MAX_PPRZ / RC_UPDATE_FREQ);
			} else {
				vicon_control_point_pos_i.z -= (int32_t) ((radio_control.values[RADIO_PITCH] * max_rc_theta) /  MAX_PPRZ / RC_UPDATE_FREQ);
			}
		} else {
			if (SWITCH_ELEVDR_ON()) {
				vicon_control_point_pos_i.y += (int32_t) ((radio_control.values[RADIO_PITCH] * max_rc_theta * 2) /  MAX_PPRZ / RC_UPDATE_FREQ);
			} else {
				vicon_control_point_pos_i.y += (int32_t) ((radio_control.values[RADIO_PITCH] * max_rc_theta) /  MAX_PPRZ / RC_UPDATE_FREQ);
			}
		}
	}
	/*if (radio_control.values[RADIO_THRUST] > 1000) {
	  vicon_control_point_pos_i.z = (int32_t) ((radio_control.values[RADIO_THRUST] - 1000) / 4500);
  }*/

	if (in_flight) {
		if (YAW_DEADBAND_EXCEEDED()) {
			sp->psi += (int32_t) ((radio_control.values[RADIO_YAW] * max_rc_r) /  MAX_PPRZ / RC_UPDATE_FREQ);
			INT32_ANGLE_NORMALIZE(sp->psi);
		}
	}
	else { /* if not flying, use current yaw as setpoint */

		struct Int32Eulers tmp;
		INT32_EULERS_OF_QUAT(tmp, ned_to_body_orientation_quat_i);

		sp->psi = tmp.psi;
	}
}

static void position_control_loop_run(struct Int32Eulers *sp, int32_t vicon_ticks_since_last_update) {
	struct Int32Vect3 vicon_pos_err;
	VECT3_DIFF(vicon_pos_err, vicon_pos_i, vicon_control_point_pos_i);
	INT32_VECT3_BOUND_NORM(vicon_pos_err, vicon_pos_err, 1000);
	struct Int32Vect3 vicon_pos_err_diff;
	VECT3_DIFF(vicon_pos_err_diff, vicon_pos_err, prev_vicon_pos_err);
	VECT3_SMUL(vicon_pos_err_diff, vicon_pos_err_diff, PERIODIC_FREQUENCY / vicon_ticks_since_last_update);
	// save for next comp:
	VECT3_COPY(prev_vicon_pos_err, vicon_pos_err);

	struct Int32RMat vicon_to_body_rmat;
	INT32_RMAT_OF_QUAT(vicon_to_body_rmat, ned_to_body_orientation_quat_i);

	struct Int32Vect3 body_pos_err;
	INT32_RMAT_VMULT(body_pos_err, vicon_to_body_rmat, vicon_pos_err);

	INT32_VECT3_BOUND_NORM(body_pos_err, body_pos_err, 1000);


	struct Int32Vect3 body_pos_err_diff;
	VECT3_DIFF(body_pos_err_diff, body_pos_err, prev_body_pos_err);
	VECT3_SMUL(body_pos_err_diff, body_pos_err_diff, vicon_ticks_since_last_update); // we know vicon runs at 120 Hz, so fix this?

	// Note that we should be scaling by 1/PERIODIC_FREQUENCY but this causes the value to basically be 0 each time.
	// As long as PERIODIC_FREQUENCY remains constant the Diff constants are fine
	// I currently scale by 1/PERIODIC_FREQUENCY below!!!

	// save for next comp:
	VECT3_COPY(prev_body_pos_err, body_pos_err);

	/*    const int32_t max_rc_phi = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_PHI);
    const int32_t max_rc_theta = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_THETA);
    const int32_t max_rc_r = (int32_t) ANGLE_BFP_OF_REAL(STABILIZATION_ATTITUDE_SP_MAX_R);*/

	int32_t cmd_roll  = 0;
	int32_t cmd_pitch = 0;
	int32_t cmd_thrust = 0;

	//if (SWITCH_ELEVDR_ON()) {
	cmd_roll  += (position_control_gains.p.x * body_pos_err.x) / 16;
	cmd_pitch += (position_control_gains.p.y * body_pos_err.y) / 16;
	cmd_thrust += (position_control_gains.p.z * body_pos_err.z) / 16;
	//}
	//if (SWITCH_FLAP_ON()) {
	cmd_roll  += (position_control_gains.d.x * body_pos_err_diff.x) / 16;
	cmd_pitch += (position_control_gains.d.y * body_pos_err_diff.y) / 16;
	cmd_thrust += (position_control_gains.d.z * body_pos_err_diff.z) / 16;
	//}

	sp->phi = (int32_t) ((-cmd_roll * posctrl_max_rc_phi) / 1000);
	sp->theta = (int32_t) ((cmd_pitch * posctrl_max_rc_theta) / 1000);

	BoundAbs(sp->phi, posctrl_max_rc_phi);
	BoundAbs(sp->theta, posctrl_max_rc_theta);


	position_control_gains.dd.z = vicon_to_body_rmat.m[8]; // 1<14 = 1
	// if every it is 0 it means we have no thrust in z direction:
	uint32_t thrust_factor = 1024;
	if (vicon_to_body_rmat.m[8] != 0)
		thrust_factor = (1024 * 16384) / vicon_to_body_rmat.m[8];

	cmd_thrust = (cmd_thrust * thrust_factor) / 1024;

	BoundAbs(cmd_thrust, posctrl_max_thrust);
	position_control_thrust_correction = cmd_thrust;

	/*  PID feedback */
	/*    fb_commands[COMMAND_ROLL] =
      GAIN_PRESCALER_P * gains->p.x  * QUAT1_FLOAT_OF_BFP(att_err->qx) / 4 +
      GAIN_PRESCALER_D * gains->d.x  * RATE_FLOAT_OF_BFP(rate_err->p) / 16 +
      GAIN_PRESCALER_I * gains->i.x  * QUAT1_FLOAT_OF_BFP(sum_err->qx) / 2;

    fb_commands[COMMAND_PITCH] =
      GAIN_PRESCALER_P * gains->p.y  * QUAT1_FLOAT_OF_BFP(att_err->qy) / 4 +
      GAIN_PRESCALER_D * gains->d.y  * RATE_FLOAT_OF_BFP(rate_err->q)  / 16 +
      GAIN_PRESCALER_I * gains->i.y  * QUAT1_FLOAT_OF_BFP(sum_err->qy) / 2;

    fb_commands[COMMAND_YAW] =
      GAIN_PRESCALER_P * gains->p.z  * QUAT1_FLOAT_OF_BFP(att_err->qz) / 4 +
      GAIN_PRESCALER_D * gains->d.z  * RATE_FLOAT_OF_BFP(rate_err->r)  / 16 +
      GAIN_PRESCALER_I * gains->i.z  * QUAT1_FLOAT_OF_BFP(sum_err->qz) / 2;

    sp->phi = (int32_t) ((radio_control.values[RADIO_ROLL] * max_rc_phi) /  MAX_PPRZ);
    sp->theta = (int32_t) ((radio_control.values[RADIO_PITCH] * max_rc_theta) /  MAX_PPRZ);

    // we better be flying at this point:
    if (YAW_DEADBAND_EXCEEDED()) {
      sp->psi += (int32_t) ((radio_control.values[RADIO_YAW] * max_rc_r) /  MAX_PPRZ / RC_UPDATE_FREQ);
      INT32_ANGLE_NORMALIZE(sp->psi);
    }

	 */
}

static inline void on_imu_event( void ) {

	// need to disable irq to prevent access. alternatively move the message sending into the main loop
	___disable_irq();
	VECT3_ADD(acc_integrator, imu.accel_unscaled);
	VECT3_ADD(mag_integrator, imu.mag_unscaled);
	RATES_ADD(gyro_integrator, imu.gyro_unscaled);

	acc_x[imu_hist_index] = imu.accel_unscaled.x;
	acc_y[imu_hist_index] = imu.accel_unscaled.y;
	acc_z[imu_hist_index] = imu.accel_unscaled.z;
	gyro_p[imu_hist_index] = imu.gyro_unscaled.p;
	gyro_q[imu_hist_index] = imu.gyro_unscaled.q;
	gyro_r[imu_hist_index] = imu.gyro_unscaled.r;
	imu_hist_index++;

	//  VECT3_ADD(acc_integrator, imu.accel);
	//  VECT3_ADD(mag_integrator, imu.mag);
	//  RATES_ADD(gyro_integrator, imu.gyro);
	integrator_count++;
	___enable_irq();

	ImuScaleGyro(imu);
	ImuScaleAccel(imu);
	ImuScaleMag(imu);

	if (ahrs.status == AHRS_UNINIT) {
		ahrs_aligner_run();
		if (ahrs_aligner.status == AHRS_ALIGNER_LOCKED)
			ahrs_align();
	}
	else {
		ahrs_propagate();
		//ins_propagate();

		if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode == COMMAND_MODE_MOTORS)) {
//			stabilization_motors_command_run();
//			BoundAbs(stabilization_cmd[COMMAND_YAW], 256);
			// no I, no P, no thrust:
//			stabilization_attitude_run_new(0, 0, 0, gain_matrix);
			// no I, WITH P, no thrust:
			stabilization_attitude_run_new(0, 1, 0, gain_matrix);

			// add in the sent in motor commands:
			for (int i = 0; i < 6; i++) {
				 fb_commands_u[i] += cmd_motors.u[i];

				 // offset for ailevon neutral:
				 if (i >= 4) fb_commands_u[i] -= 5000;

//				 if (fb_commands_u[i] < 0) fb_commands_u[i] = 0;
//				 if (fb_commands_u[i] > 10000) fb_commands_u[i] = 10000;
			}

			actuators_set_i2c_new(autopilot_motors_on, fb_commands_u);
//			actuators_direct_cmd_set_i2c(autopilot_motors_on, stabilization_cmd, &cmd_motors);
		} else {
			if ((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode == COMMAND_MODE_HYBRID)) {
//				stabilization_hybrid_command_run();
//				DELETE DURING CLEANUP, NEEDS TO BE PORTED TO CURRENT CONTROL STRUCTURE IF USED
			} else {
				if (control_mode == CONTROL_MODE_MANUAL) {
					RATES_ASSIGN(cmd_rates, 0, 0, 0);
				}
				//stabilization_attitude_run(0);

				int set_thrust = ((radio_control.values[RADIO_THROTTLE] - MAX_PPRZ / 20) * 10000) / MAX_PPRZ;
				stabilization_attitude_run_new(1, 1, set_thrust, gain_matrix_manual);

				if ((control_mode == CONTROL_MODE_MANUAL) ||
						((control_mode == CONTROL_MODE_AUTOPILOT) && (autopilot_command_mode == COMMAND_MODE_BODY_POSE))) {
					//stabilization_cmd[COMMAND_THRUST] = radio_control.values[RADIO_THROTTLE] / 8 + 800;
					if (!SWITCH_GEAR_ON()) {
						stabilization_cmd[COMMAND_THRUST] = ((radio_control.values[RADIO_THROTTLE] - MAX_PPRZ / 20) * 10000) / MAX_PPRZ;
					}
					/*if (SWITCH_GEAR_ON()) {
						stabilization_cmd[COMMAND_THRUST] += position_control_thrust_correction;
					}*/

				}
			}

			BoundAbs(stabilization_cmd[COMMAND_THRUST], MAX_PPRZ);
			if (stabilization_cmd[COMMAND_THRUST] < 0) stabilization_cmd[COMMAND_THRUST] = 0;

			BoundAbs(stabilization_cmd[COMMAND_YAW], 256);

			//actuators_set_i2c(autopilot_motors_on, stabilization_cmd);
			actuators_set_i2c_new(autopilot_motors_on, fb_commands_u);

		}

		// ACCEL
		ahrs_update_accel();

		// MAG
		if (ahrs.status == AHRS_RUNNING) {
			/* TURN MAG OF IN VICON ROOM! */
			if (vicon_valid_msg_age >= 63) {
				ahrs_update_mag();
				mag_in_use = 1;
			} else {
				mag_in_use = 0;
			}
		}
	}
}

#define MAG_MASK 0x0fff
#define GYRO_MASK 0x0fff
#define ACCEL_MASK 0x0fff

/*void reset_handler(void);
void nmi_handler(void);
void hard_fault_handler(void);
void mem_manage_handler(void);
void bus_fault_handler(void);
void usage_fault_handler(void);
void sv_call_handler(void);
void debug_monitor_handler(void);
void pend_sv_handler(void);
void sys_tick_handler(void); */

static void broken_irq(int which) {
	while (1) {
		led_set(0, 0); led_set(1, 0); led_set(2, 0); led_set(3, 0); led_set(4, 1);
		for (int i = 0; i < 1000000; i++) __asm__("nop");
		led_set(0, 0); led_set(1, 0); led_set(2, 1); led_set(3, 1); led_set(4, 1);
		for (int i = 0; i < 1000000; i++) __asm__("nop");
		led_set(0, 1); led_set(1, 1); led_set(2, 1); led_set(3, 1); led_set(4, 1);
		for (int i = 0; i < 1000000; i++) __asm__("nop");
		led_set(0, (which & 0x10) > 0); led_set(1, (which & 0x08) > 0); led_set(2, (which & 0x04) > 0); led_set(3, (which & 0x02) > 0); led_set(4, (which & 0x01) > 0);
		for (int i = 0; i < 1000000; i++) __asm__("nop");
		led_set(0, 0); led_set(1, 0); led_set(2, (which & 0x80) > 0); led_set(3, (which & 0x40) > 0); led_set(4, (which & 0x20) > 0);
		for (int i = 0; i < 1000000; i++) __asm__("nop");
	}
}

void wwdg_isr(void)			{ broken_irq(1); }
void pvd_isr(void)			{ broken_irq(2); }
void tamper_isr(void)		{ broken_irq(3); }
void rtc_isr(void) 			{ broken_irq(4); }
void flash_isr(void)		{ broken_irq(5); }
void rcc_isr(void) 			{ broken_irq(6); }
void exti0_isr(void) 		{ broken_irq(7); }
void exti1_isr(void) 		{ broken_irq(8); }
void exti2_isr(void) 		{ broken_irq(9); }
void exti3_isr(void) 		{ broken_irq(10); }
void exti4_isr(void) 		{ broken_irq(11); }
void dma1_channel1_isr(void) { broken_irq(12); }
void dma1_channel2_isr(void) { broken_irq(13); }
void dma1_channel3_isr(void) { broken_irq(14); }
// in spi_arch.c: void dma1_channel4_isr(void) { broken_irq(15); }
// in spi_arch.c: void dma1_channel5_isr(void) { broken_irq(16); }
void dma1_channel6_isr(void) { broken_irq(17); }
void dma1_channel7_isr(void) { broken_irq(18); }
//void adc1_2_isr(void) 		{ broken_irq(19); }
void usb_hp_can_tx_isr(void) { broken_irq(20); }
void usb_lp_can_rx0_isr(void) { broken_irq(21); }
void can_rx1_isr(void) 		{ broken_irq(22); }
void can_sce_isr(void) 		{ broken_irq(23); }
//void exti9_5_isr(void) 		{ broken_irq(24); }
void tim1_brk_isr(void)		{ broken_irq(25); }
void tim1_up_isr(void) 		{ broken_irq(26); }
void tim1_trg_com_isr(void) { broken_irq(27); }
void tim1_cc_isr(void) 		{ broken_irq(28); }
void tim2_isr(void) 		{ broken_irq(29); }
void tim3_isr(void) 		{ broken_irq(30); }
void tim4_isr(void) 		{ broken_irq(31); }
//void i2c1_ev_isr(void) 		{ broken_irq(32); }
//void i2c1_er_isr(void) 		{ broken_irq(33); }
void i2c2_ev_isr(void) 		{ broken_irq(34); }
void i2c2_er_isr(void) 		{ broken_irq(35); }
void spi1_isr(void) 		{ broken_irq(36); }
void spi2_isr(void) 		{ broken_irq(37); }
// in spektrum_arch.c: void usart1_isr(void) 		{ broken_irq(38); }
// in main.c: void usart2_isr(void) 	{ broken_irq(39); }
// in main.c: void usart3_isr(void) 		{ broken_irq(40); }
// in main.c: void usart3_isr(void) 		{ broken_irq(41); }
void exti15_10_isr(void) 	{ broken_irq(42); }
void rtc_alarm_isr(void) 	{ broken_irq(43); }
void usb_wakeup_isr(void) 	{ broken_irq(44); }
void tim8_brk_isr(void) 	{ broken_irq(45); }
void tim8_up_isr(void) 		{ broken_irq(46); }
void tim8_trg_com_isr(void) { broken_irq(47); }
void tim8_cc_isr(void) 		{ broken_irq(48); }
void adc3_isr(void) 		{ broken_irq(49); }
void fsmc_isr(void) 		{ broken_irq(50); }
void sdio_isr(void) 		{ broken_irq(51); }
void tim5_isr(void) 		{ broken_irq(52); }
void spi3_isr(void) 		{ broken_irq(53); }
void uart4_isr(void) 		{ broken_irq(54); }
// in spektrum_arch.c (for some reason remains undetected): void uart5_isr(void) 		{ broken_irq(55); }
// in spektrum_arch.c: void tim6_isr(void) 		{ broken_irq(56); }
void tim7_isr(void) 		{ broken_irq(57); }
void dma2_channel1_isr(void) { broken_irq(58); }
void dma2_channel2_isr(void) { broken_irq(59); }
void dma2_channel3_isr(void) { broken_irq(60); }
void dma2_channel4_5_isr(void) { broken_irq(61); }
void dma2_channel5_isr(void) { broken_irq(62); }
void eth_isr(void) 			{ broken_irq(63); }
void eth_wkup_isr(void) 	{ broken_irq(64); }
void can2_tx_isr(void) 		{ broken_irq(65); }
void can2_rx0_isr(void) 	{ broken_irq(66); }
void can2_rx1_isr(void) 	{ broken_irq(67); }
void can2_sce_isr(void) 	{ broken_irq(68); }
void otg_fs_isr(void) 		{ broken_irq(69); }
