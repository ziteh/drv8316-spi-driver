/**
 * @file drv8316.h
 * @brief A platform independent library for TI DRV8316/DRV8316C
 *        3-phase motor driver SPI communication.
 * @author ZiTe (honmonoh@gmail.com)
 */

#ifndef DRV8316_H_
#define DRV8316_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/* Registers. */
#define DRV8316_REG_IC_STAT (0x00) /* IC Status Resgiter. */
#define DRV8316_REG_STAT1 (0x01)   /* Status Register 1. */
#define DRV8316_REG_STAT2 (0x02)   /* Status Register 2. */
#define DRV8316_REG_CTRL1 (0x03)   /* Control Register 1. */
#define DRV8316_REG_CTRL2 (0x04)   /* Control Register 2. */
#define DRV8316_REG_CTRL3 (0x05)   /* Control Register 3. */
#define DRV8316_REG_CTRL4 (0x06)   /* Control Register 4. */
#define DRV8316_REG_CTRL5 (0x07)   /* Control Register 5. */
#define DRV8316_REG_CTRL6 (0x08)   /* Control Register 6. */
#define DRV8316_REG_CTRL10 (0x0C)  /* Control Register 10. */

  typedef void (*drv8316_spi_send_t)(uint16_t data);
  typedef uint16_t (*drv8316_spi_read_t)(void);
  typedef void (*drv8316_spi_select_t)(void);
  typedef void (*drv8316_spi_deselect_t)(void);
  typedef void (*drv8316_delay_t)(void);

  typedef uint8_t drv8316_enable_t;
#define DRV8316_DISABLE (0)
#define DRV8316_ENABLE (1)

  typedef struct
  {
    drv8316_spi_send_t spi_send;
    drv8316_spi_read_t spi_read;
    drv8316_spi_select_t spi_select;
    drv8316_spi_deselect_t spi_deselect;
    drv8316_delay_t delay;
  } drv8316_handle_t;

  typedef enum
  {
    pwm_6x = 0,      /* 6x PWM mode. */
    pwm_6x_cur_lim,  /* 6x PMW mode with current limit. */
    pwm_3x,          /* 3x PWM mode. */
    pwm_3x_cur_lim,  /* 3x PMW mode with current limit. */
    default = pwm_6x /* Reset value. */
  } drv8316_pwm_mode_t;

  typedef enum
  {
    sr_25 = 0,      /* Slew rate is 25 V/µs. */
    sr_50,          /* Slew rate is 50 V/µs. */
    sr_125,         /* Slew rate is 125 V/µs. */
    sr_200,         /* Slew rate is 200 V/µs. */
    default = sr_25 /* Reset value. */
  } drv8316_slew_rate_t;

  typedef enum
  {
    f_20khz = 0,      /* Freqency of PWM at 100% duty cycle is 20 kHz. */
    f_40khz,          /* Freqency of PWM at 100% duty cycle is 40 kHz. */
    default = f_20khz /* Reset value. */
  } drv8316_pwm100dc_freq_t;

  typedef enum
  {
    ovp_34v = 0,      /* VM overvoltage level is 34 V. */
    ovp_22v,          /* VM overvoltage level is 22 V. */
    default = ovp_34v /* Reset value. */
  } drv8316_ovp_t;

  typedef enum
  {
    ocp_deglitch_0_2us = 0,      /* OCP deglitch time is 0.2 µs. */
    ocp_deglitch_0_6us,          /* OCP deglitch time is 0.6 µs. */
    ocp_deglitch_1_25us,         /* OCP deglitch time is 1.25 µs. */
    ocp_deglitch_1_6us,          /* OCP deglitch time is 1.6 µs. */
    default = ocp_deglitch_0_6us /* Reset value. */
  } drv8316_ocp_deglitch_t;

  typedef enum
  {
    ocp_retry_5ms = 0,      /* OCP retry time is 5 ms. */
    ocp_retry_500ms,        /* OCP retry time is 500 ms. */
    default = ocp_retry_5ms /* Reset value. */
  } drv8316_ocp_retry_t;
  typedef enum
  {
    ocp_16a = 0,      /* OCP level is 16 A. */
    ocp_24a,          /* OCP level is 24 A. */
    default = ocp_16a /* Reset value. */
  } drv8316_ocp_lv_t;

  typedef enum
  {
    fet = 0,      /* Current recirculation through FETs (Brake Mode). */
    diodes,       /* Current recirculation through diodes (Coast Mode). */
    default = fet /* Reset value. */
  } drv8316_i_lim_recir_t;

  typedef enum
  {
    x0_15 = 0,      /* Current sense amplifier's gain is 0.15 V/A. */
    x0_3,           /* Current sense amplifier's gain is 0.3 V/A. */
    x0_6,           /* Current sense amplifier's gain is 0.6 V/A. */
    x1_2,           /* Current sense amplifier's gain is 1.2 V/A. */
    default = x0_15 /* Reset value. */
  } drv8316_csa_gain_t;

  typedef enum
  {
    cl_600ma = 0,      /* Buck regulator current limit is set to 600 mA. */
    cl_150ma,          /* Buck regulator current limit is set to 150 mA. */
    default = cl_600ma /* Reset value. */
  } drv8316_buck_cl_t;

  typedef enum
  {
    buck_3v3 = 0,      /* Buck regulator voltage is 3.3 V. */
    buck_5v,           /* Buck regulator voltage is 5.0 V. */
    buck_4v,           /* Buck regulator voltage is 4.0 V. */
    buck_5v7,          /* Buck regulator voltage is 5.7 V. */
    default = buck_3v3 /* Reset value. */
  } drv8316_buck_voltage_t;

  typedef enum
  {
    dly_0_0us = 0,      /* Driver delay compensation target is 0 µs. */
    dly_0_4us,          /* Driver delay compensation target is 0.4 µs. */
    dly_0_6us,          /* Driver delay compensation target is 0.6 µs. */
    dly_0_8us,          /* Driver delay compensation target is 0.8 µs. */
    dly_1_0us,          /* Driver delay compensation target is 1.0 µs. */
    dly_1_2us,          /* Driver delay compensation target is 1.2 µs. */
    dly_1_4us,          /* Driver delay compensation target is 1.4 µs. */
    dly_1_6us,          /* Driver delay compensation target is 1.6 µs. */
    dly_1_8us,          /* Driver delay compensation target is 1.8 µs. */
    dly_2_0us,          /* Driver delay compensation target is 2.0 µs. */
    dly_2_2us,          /* Driver delay compensation target is 2.2 µs. */
    dly_2_4us,          /* Driver delay compensation target is 2.4 µs. */
    dly_2_6us,          /* Driver delay compensation target is 2.6 µs. */
    dly_2_8us,          /* Driver delay compensation target is 2.8 µs. */
    dly_3_0us,          /* Driver delay compensation target is 3.0 µs. */
    dly_3_2us,          /* Driver delay compensation target is 3.2 µs. */
    default = dly_0_0us /* Reset value. */
  } drv8316_drv_delay_target_t;

  int drv8316_lock_reg(const drv8316_handle_t *drv8316_handle, uint8_t lock);

  int drv8316_pwm_mode(const drv8316_handle_t *drv8316_handle, drv8316_pwm_mode_t pwm_mode);
  int drv8316_sdo_mode(const drv8316_handle_t *drv8316_handle, uint8_t push_pull_output);
  int drv8316_slew_rate(const drv8316_handle_t *drv8316_handle, drv8316_slew_rate_t sr);
  int drv8316_clear_fault(const drv8316_handle_t *drv8316_handle);

  int drv8316_pwm100dc_sel(const drv8316_handle_t *drv8316_handle, drv8316_pwm100dc_freq_t freqency_sel);
  int drv8316_ovp_sel(const drv8316_handle_t *drv8316_handle, drv8316_ovp_t ovp);
  int drv8316_ovp_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable);
  int drv8316_spi_fault_rep_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable);
  int drv8316_otw_rep_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable);

  int drv8316_driver_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable);
  int drv8316_ocp_cbc_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable);
  int drv8316_ocp_deglitch_time(const drv8316_handle_t *drv8316_handle, drv8316_ocp_deglitch_t deglitch);
  int drv8316_ocp_retry_time(const drv8316_handle_t *drv8316_handle, drv8316_ocp_retry_t retry);
  int drv8316_ocp_level(const drv8316_handle_t *drv8316_handle, drv8316_ocp_lv_t lv);
  int drv8316_ocp_mode(const drv8316_handle_t *drv8316_handle, uint8_t ocp_mode);

  int drv8316_current_limt_sel(const drv8316_handle_t *drv8316_handle, drv8316_i_lim_recir_t i_lim_recir);
  int drv8316_aar_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable);
  int drv8316_asr_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable);
  int drv8316_csa_gain(const drv8316_handle_t *drv8316_handle, drv8316_csa_gain_t csa_gain);

  int drv8316_buck_ps_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable);
  int drv8316_buck_current_limt(const drv8316_handle_t *drv8316_handle, drv8316_buck_cl_t cl);
  int drv8316_buck_voltage(const drv8316_handle_t *drv8316_handle, drv8316_buck_voltage_t buck_voltage);
  int drv8316_buck_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable);

  int drv8316_drv_delay_compensation(const drv8316_handle_t *drv8316_handle,
                                     drv8316_enable_t enable,
                                     drv8316_drv_delay_target_t delay_target);

  int drv8316_get_status(const drv8316_handle_t *drv8316_handle,
                         uint8_t *ic_status,
                         uint8_t *status1,
                         uint8_t *status2)

#ifdef __cplusplus
}
#endif

#endif /* DRV8316_H_ */
