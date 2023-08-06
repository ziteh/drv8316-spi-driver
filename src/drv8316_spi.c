
#include "drv8316_spi.h"

/**
 * @brief Modify only certain bits.
 */
#define SET_BITS(src, out, start, len, newval)                           \
  do                                                                     \
  {                                                                      \
    uint16_t mask = (1 << (len)) - 1;                                    \
    (out) = ((src) & ~(mask << (start))) | (((newval)&mask) << (start)); \
  } while (0)

uint8_t is_even_parity(uint16_t src);

typedef struct
{
  uint8_t status;
  uint8_t data;
} drv8316_response_t;

/**
 * @brief Writing data to register.
 *
 * @param drv8316_handle DRV8316 handle.
 * @param reg_address Register address.
 * @param data Data.
 * @return Response.
 */
drv8316_response_t drv8316_reg_write(const drv8316_handle_t *drv8316_handle, uint8_t reg_address, uint8_t data)
{
  reg_address &= 0x3F; /* Mask, address only 6 bit. */

  uint16_t data_frame = 0x00;              /* MSB=0: write command. */
  data_frame += (reg_address << 9) + data; /* bit-0~7: data, bit-9~14: address. */

  if (!is_even_parity(data_frame))
  {
    data_frame ^= 1UL << 8; /* Toggle parity bit (bit-8). */
  }

  drv8316_spi_transmit(drv8316_handle, data_frame);
  uint16_t ret = drv8316_spi_receive(drv8316_handle);

  drv8316_response_t response;
  response.status = ret >> 8;
  response.data = ret & 0xFF;
  return response;
}

/**
 * @brief Reading data from register.
 *
 * @param drv8316_handle DRV8316 handle.
 * @param reg_address Register address.
 * @return Response.
 */
drv8316_response_t drv8316_reg_read(const drv8316_handle_t *drv8316_handle, uint8_t reg_address)
{
  reg_address &= 0x3F; /* Mask, address only 6 bit. */

  uint16_t data_frame = 0x80;       /* MSB=1: read command. */
  data_frame += (reg_address << 9); /* bit-9~14: address. */

  if (!is_even_parity(data_frame))
  {
    data_frame ^= 1UL << 8; /* Toggle parity bit (bit-8). */
  }

  drv8316_spi_transmit(drv8316_handle, data_frame);
  uint16_t ret = drv8316_spi_receive(drv8316_handle);

  drv8316_response_t response;
  response.status = ret >> 8;
  response.data = ret & 0xFF;
  return response;
}

drv8316_response_t drv8316_reg_modify(const drv8316_handle_t *drv8316_handle,
                                      uint8_t reg_address,
                                      uint8_t start_bit,
                                      uint8_t len,
                                      uint8_t new_value)
{
  drv8316_response_t response = drv8316_reg_read(drv8316_handle, reg_address); /* Read old data. */
  uint16_t new_data = 0;
  SET_BITS(response.data, new_data, start_bit, len, new_value);
  return drv8316_reg_write(drv8316_handle, reg_address, new_data);
}

void drv8316_spi_transmit(const drv8316_handle_t *drv8316_handle, uint16_t data)
{
  drv8316_handle->delay();

  drv8316_handle->spi_select();
  drv8316_handle->spi_send(data);
  drv8316_handle->spi_deselect();
}

uint16_t drv8316_spi_receive(const drv8316_handle_t *drv8316_handle)
{
  drv8316_handle->delay();

  drv8316_handle->spi_select();
  uint16_t response = drv8316_handle->spi_read();
  drv8316_handle->spi_deselect();

  return response;
}

int drv8316_lock_reg(const drv8316_handle_t *drv8316_handle, uint8_t lock)
{
  /*
   *  REG_LOCK: CTRL1 bit 0~2.
   *  011b: Unlock all registers.
   *  110b: Lock the settings by ignoring further register writes
   *        except to these bits and address 0x03h bits 2-0.
   *  Others: no effect unless locked or unlocked.
   */
  uint8_t reg_lock = (lock == 1) ? 0x06 : 0x03;
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL1, 0, 3, reg_lock);
}

int drv8316_pwm_mode(const drv8316_handle_t *drv8316_handle, drv8316_pwm_mode_t pwm_mode)
{
  /* PWM_MODE: CTRL2 bit 1~2. */
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL2, 1, 2, (uint8_t)pwm_mode);
}

/**
 * @brief SDO IO mode selection, Push-pull or Open-drain.
 *
 * @param drv8316_handle
 * @param push_pull_output 1: SDO in Push-pull mode. 0: SDO in Open-drain mode.
 * @return
 */
int drv8316_sdo_mode(const drv8316_handle_t *drv8316_handle, uint8_t push_pull_output)
{
  /* SDO_MODE: CTRL2 bit 5. */
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL2, 5, 1, push_pull_output);
}

int drv8316_slew_rate(const drv8316_handle_t *drv8316_handle, drv8316_slew_rate_t sr)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL2, 3, 2, (uint8_t)sr);
}

int drv8316_clear_fault(const drv8316_handle_t *drv8316_handle)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL2, 0, 1, 0x1);
}

int drv8316_pwm100dc_sel(const drv8316_handle_t *drv8316_handle, drv8316_pwm100dc_freq_t freqency_sel)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL3, 4, 1, (uint8_t)freqency_sel);
}

int drv8316_ovp_sel(const drv8316_handle_t *drv8316_handle, drv8316_ovp_t ovp)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL3, 3, 1, (uint8_t)ovp);
}

int drv8316_ovp_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL3, 2, 1, enable);
}

int drv8316_spi_fault_rep_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL3, 1, 1, !enable); /* 1 to desable. */
}

int drv8316_otw_rep_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL3, 0, 1, enable);
}

int drv8316_driver_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL4, 7, 1, !enable); /* 1 to Hi-Z FETs. */
}

int drv8316_ocp_cbc_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL4, 6, 1, enable);
}

int drv8316_ocp_deglitch_time(const drv8316_handle_t *drv8316_handle, drv8316_ocp_deglitch_t deglitch)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL4, 4, 2, (uint8_t)deglitch);
}

int drv8316_ocp_retry_time(const drv8316_handle_t *drv8316_handle, drv8316_ocp_retry_t retry)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL4, 3, 1, (uint8_t)retry);
}

int drv8316_ocp_level(const drv8316_handle_t *drv8316_handle, drv8316_ocp_lv_t lv)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL4, 2, 1, (uint8_t)lv);
}

/**
 * @brief Overcurrent protection (OCP) fault options.
 *
 * @param drv8316_handle
 * @param ocp_mode 0: Overcurrent causes a latched fault.
 *                 1: OVercurrent causes an automatic retrying fault.
 *                 2: Overcurrent is report only but no action is taken.
 *                 3: Overcurrent is not reported and no action is taken.
 * @return
 */
int drv8316_ocp_mode(const drv8316_handle_t *drv8316_handle, uint8_t ocp_mode)
{
  if (ocp_mode > 3)
  {
    return -9999;
  }

  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL4, 0, 2, ocp_mode);
}

int drv8316_current_limt_sel(const drv8316_handle_t *drv8316_handle, drv8316_i_lim_recir_t i_lim_recir)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL5, 6, 1, (uint8_t)i_lim_recir);
}

int drv8316_aar_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL5, 3, 1, enable);
}

int drv8316_asr_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL5, 2, 1, enable);
}

int drv8316_csa_gain(const drv8316_handle_t *drv8316_handle, drv8316_csa_gain_t csa_gain)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL5, 0, 2, (uint8_t)csa_gain);
}

int drv8316_buck_ps_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL6, 4, 1, !enable); /* 1 to disable. */
}

int drv8316_buck_current_limt(const drv8316_handle_t *drv8316_handle, drv8316_buck_cl_t cl)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL6, 3, 1, (uint8_t)cl);
}

int drv8316_buck_voltage(const drv8316_handle_t *drv8316_handle, drv8316_buck_voltage_t buck_voltage)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL6, 1, 2, (uint8_t)buck_voltage);
}

int drv8316_buck_en(const drv8316_handle_t *drv8316_handle, drv8316_enable_t enable)
{
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL6, 0, 1, !enable); /* 1 to disable. */
}

int drv8316_drv_delay_compensation(const drv8316_handle_t *drv8316_handle,
                                   drv8316_enable_t enable,
                                   drv8316_drv_delay_target_t delay_target)
{
  drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL10, 4, 1, enable);
  return drv8316_reg_modify(drv8316_handle, DRV8316_REG_CTRL10, 0, 4, (uint8_t)delay_target);
}

int drv8316_get_status(const drv8316_handle_t *drv8316_handle,
                       uint8_t *ic_status,
                       uint8_t *status1,
                       uint8_t *status2)
{
  drv8316_response_t response = drv8316_reg_read(drv8316_handle, DRV8316_REG_IC_STAT);
  ic_status = response.data;

  response = drv8316_reg_read(drv8316_handle, DRV8316_REG_STAT1);
  status1 = response.data;

  response = drv8316_reg_read(drv8316_handle, DRV8316_REG_STAT2);
  status2 = response.data;

  return response.status;
}

/**
 * @brief Check data even parity.
 *
 * @param src Source data.
 * @return 0 if the source data is not even parity.
 */
uint8_t is_even_parity(uint16_t src)
{
  uint8_t shift = 1;
  while (shift < (sizeof(src) * 8))
  {
    src ^= (src >> shift);
    shift <<= 1;
  }
  return !(src & 0x1);
}
