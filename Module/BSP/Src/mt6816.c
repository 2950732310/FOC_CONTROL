#include "mt6816.h"

/* ========== DMA 连续采样缓冲区 (8bit × 4) ========== */
static uint8_t  mt6816_tx_buf[MT6816_DMA_BUF_SIZE];
static uint8_t  mt6816_rx_buf[MT6816_DMA_BUF_SIZE];

volatile uint16_t mt6816_latest_raw  = 0;
volatile uint8_t  mt6816_data_ready  = 0;


/**
 * @brief       启动 MT6816 持续 DMA 采样（仅初始化时调用一次）
 * @param       None
 * @retval      None
 * @note        SPI 8bit / DMA NORMAL 模式。
 *              DMA 传输完成后回调自动重启，形成连续采样链。
 *              CS 保持低电平，FOC 线程只读缓存不碰 SPI。
 */
void MT6816_DMA_Start(void)
{
    /* TX: [angle_reg, dummy, warning_reg, dummy] */
    mt6816_tx_buf[0] = MT6816_Angle_Reg;        /* 0x83 */
    mt6816_tx_buf[1] = 0x00;                    /* dummy */
    mt6816_tx_buf[2] = MT6816_Warning_Reg;      /* 0x84 */
    mt6816_tx_buf[3] = 0x00;                    /* dummy */

    mt6816_data_ready = 0;
    MT6816_SPI_CS_L();                  /* CS 保持低 */

    HAL_SPI_TransmitReceive_DMA(&MT6816_SPI_Get_HSPI,
                                mt6816_tx_buf,
                                mt6816_rx_buf,
                                MT6816_DMA_BUF_SIZE);
}


/**
 * @brief       SPI3 DMA 全传回调
 * @param       hspi SPI 句柄
 * @note        DMA NORMAL 模式自动停止。在此解析角度并重启 DMA。
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3)
    {   
        
        /*
         * 4字节传输时序 (CS 全程低):
         *   MOSI: 0x83  0x00  0x84  0x00
         *   MISO: junk  AH    junk  AL
         * rawAngle = (AH << 8) | AL
         */
        mt6816_latest_raw = ((uint16_t)mt6816_rx_buf[1] << 8)
                            | mt6816_rx_buf[3];
        mt6816_data_ready = 1;
        MT6816_SPI_CS_H();
        MT6816_SPI_CS_L();
        /* 重启 DMA，维持连续采样（DMA NORMAL 会再次自动停止） */
        HAL_SPI_TransmitReceive_DMA(&MT6816_SPI_Get_HSPI,
                                    mt6816_tx_buf,
                                    mt6816_rx_buf,
                                    MT6816_DMA_BUF_SIZE);
    }
}


/**
 * @brief       获取最新缓存角度原始值 (非阻塞，不进 SPI)
 * @param       None
 * @retval      最新 16 位角度原始值
 */
uint16_t mt6816_get_raw(void)
{
    return mt6816_latest_raw;
}


/**
 * @brief       获取电机角度 — 从缓存解析，不进 SPI
 * @param       encoder 编码器结构指针
 * @retval      true  = 数据有效
 *              false = 尚无数据 / 校验失败
 */
bool GetMotor_Angle(Encoder_Date *encoder)
{
    uint16_t rawAngle;
    uint8_t  h_count;

    if (!mt6816_data_ready) return false;

    rawAngle = mt6816_latest_raw;

    /* 奇偶校验（偶数个 1 为有效） */
    h_count = 0;
    for (uint8_t j = 0; j < 16; j++)
    {
        if (rawAngle & (0x01u << j)) h_count++;
    }
    if (h_count & 0x01u) return false;

    encoder->oriData  = rawAngle >> 2;
    encoder->PI_angle = (float)encoder->oriData / ENCODER_CPR_F;
    encoder->angle    = encoder->PI_angle * MAX_ANGLE;
    encoder->radian   = encoder->angle * M_pi / 180.0f;

    return true;
}

