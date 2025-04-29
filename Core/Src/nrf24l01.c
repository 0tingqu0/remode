/*
 * nrf24l01.c
 *
 *  Created on: Mar 21, 2025
 *      Author: zhang
 */
#include <nrf24l01.h>

const char *g_ErrorString = "RF24L01 is not find !...";
//volatile uint8_t nrf24l01_state=0;

/**
 * @brief :SPI收发一个字节
 * @param :
 *         @TxByte: 发送的数据字节
 * @note  :非堵塞式，一旦等待超时，函数会自动退出
 * @retval:接收到的字节
 */
uint8_t drv_spi_read_write_byte(uint8_t TxByte)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1 , &TxByte , &rx_data , 1 , HAL_MAX_DELAY);
//    HAL_SPI_TransmitReceive_DMA(&hspi1, &TxByte, &rx_data, 1);
//    while (HAL_SPI_GetState(&SPI1))!=HAL_SPI_STATE_READY);
    return rx_data;      //返回
}

/*​
 * @brief :SPI收发一个字节(DMA版本)
 * @param :
 *         @TxByte: 发送的数据字节
 * @note  :使用DMA进行SPI传输，需要等待传输完成
 * @retval:接收到的字节
 */
uint8_t drv_spi_read_write_byte_dma(uint8_t TxByte)
{
    uint8_t rx_data;
    HAL_StatusTypeDef status;

    // 使用DMA进行SPI传输
    status = HAL_SPI_TransmitReceive_DMA(&hspi1, &TxByte, &rx_data, 1);

    if(status != HAL_OK) {
        // 处理错误情况
        Error_Handler();
    }

    // 等待DMA传输完成
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

    return rx_data;
}

/*​
 * @brief SPI DMA方式读写缓冲区
 */
HAL_StatusTypeDef drv_spi_read_write_buffer_dma(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf, rx_buf, len);
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
    return status;
}

/**
 * @brief :NRF24L01读寄存器
 * @param :
 @Addr:寄存器地址
 * @note  :地址在设备中有效
 * @retval:读取的数据
 */
uint8_t NRF24L01_Read_Reg(uint8_t RegAddr)
{
    uint8_t btmp;

    RF24L01_SET_CS_LOW();          //片选

    drv_spi_read_write_byte( NRF_READ_REG | RegAddr);  //读命令 地址
    btmp = drv_spi_read_write_byte(0xFF);             //读数据

    RF24L01_SET_CS_HIGH();         //取消片选

    return btmp;
}

/*​
 * @brief :NRF24L01读寄存器(DMA版本)
 * @param :
 *         @RegAddr:寄存器地址
 * @note  :使用DMA进行SPI传输，提高效率
 * @retval:读取的数据
 */
uint8_t NRF24L01_Read_Reg_DMA(uint8_t RegAddr)
{
    uint8_t tx_buf[2] = {NRF_READ_REG | RegAddr, 0xFF};  // 命令+空数据
    uint8_t rx_buf[2] = {0};                            // 接收缓冲区

    RF24L01_SET_CS_LOW();          // 片选

    // 使用DMA进行SPI传输
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf, rx_buf, 2);

    // 等待DMA传输完成
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

    RF24L01_SET_CS_HIGH();         // 取消片选

    return rx_buf[1];              // 返回接收到的数据
}

/**
 * @brief :NRF24L01读指定长度的数据
 * @param :
 *         @reg:地址
 *         @pBuf:数据存放地址
 *         @len:数据长度
 * @note  :数据长度不超过255，地址在设备中有效
 * @retval:读取状态
 */
void NRF24L01_Read_Buf(uint8_t RegAddr, uint8_t *pBuf, uint8_t len)
{
    uint8_t btmp;

    RF24L01_SET_CS_LOW();          //片选

    drv_spi_read_write_byte( NRF_READ_REG | RegAddr);  //读命令 地址
    for (btmp = 0; btmp < len; btmp++)
    {
        *(pBuf + btmp) = drv_spi_read_write_byte(0xFF); //读数据
    }
    RF24L01_SET_CS_HIGH();     //取消片选
}

/*​
 * @brief :NRF24L01读指定长度的数据(DMA版本)
 * @param :
 *         @RegAddr: 地址
 *         @pBuf: 数据存放地址
 *         @len: 数据长度
 * @note  :数据长度不超过255，地址在设备中有效
 * @retval:无
 */
void NRF24L01_Read_Buf_DMA(uint8_t RegAddr, uint8_t *pBuf, uint8_t len)
{
    uint8_t tx_buf[32];  // 最大32字节
    uint8_t rx_buf[32];

    // 准备发送数据
    tx_buf[0] = NRF_READ_REG | RegAddr;
    memset(&tx_buf[1], 0xFF, len);  // 填充0xFF用于读取

    RF24L01_SET_CS_LOW();          // 片选

    // 使用DMA批量传输
    drv_spi_read_write_buffer_dma(tx_buf, rx_buf, len + 1);

    // 复制接收到的数据到输出缓冲区
    memcpy(pBuf, &rx_buf[1], len);

    RF24L01_SET_CS_HIGH();         // 取消片选
}

/**
 * @brief :NRF24L01写寄存器
 * @param :无
 * @note  :地址在设备中有效
 * @retval:读写状态
 */
void NRF24L01_Write_Reg(uint8_t RegAddr, uint8_t Value)
{
    RF24L01_SET_CS_LOW();      //片选

    drv_spi_read_write_byte( NRF_WRITE_REG | RegAddr); //写命令 地址
    drv_spi_read_write_byte(Value);           //写数据

    RF24L01_SET_CS_HIGH();     //取消片选
}

/**​
 * @brief :NRF24L01写寄存器(DMA版本)
 * @param :
 *         @RegAddr:寄存器地址
 *         @Value:要写入的值
 * @note  :使用DMA进行SPI传输，提高效率
 * @retval:无
 */
void NRF24L01_Write_Reg_DMA(uint8_t RegAddr, uint8_t Value)
{
    uint8_t tx_buf[2] = {NRF_WRITE_REG | RegAddr, Value};  // 命令+数据

    RF24L01_SET_CS_LOW();      // 片选

    // 使用DMA进行SPI传输
    HAL_SPI_Transmit_DMA(&hspi1, tx_buf, 2);

    // 等待DMA传输完成
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

    RF24L01_SET_CS_HIGH();     // 取消片选
}

/**
 * @brief :NRF24L01写指定长度的数据
 * @param :
 *         @reg:地址
 *         @pBuf:写入的数据地址
 *         @len:数据长度
 * @note  :数据长度不超过255，地址在设备中有效
 * @retval:写状态
 */
void NRF24L01_Write_Buf(uint8_t RegAddr, uint8_t *pBuf, uint8_t len)
{
    uint8_t i;

    RF24L01_SET_CS_LOW();      //片选

    drv_spi_read_write_byte( NRF_WRITE_REG | RegAddr); //写命令 地址
    for (i = 0; i < len; i++)
    {
        drv_spi_read_write_byte(*(pBuf + i));       //写数据
    }

    RF24L01_SET_CS_HIGH();     //取消片选
}

/*​
 * @brief :NRF24L01写指定长度的数据(DMA版本)
 * @param :
 *         @RegAddr: 地址
 *         @pBuf: 写入的数据地址
 *         @len: 数据长度
 * @note  :数据长度不超过255，地址在设备中有效
 * @retval:无
 */
void NRF24L01_Write_Buf_DMA(uint8_t RegAddr, uint8_t *pBuf, uint8_t len)
{
    uint8_t tx_buf[32];  // 最大32字节

    // 准备发送数据
    tx_buf[0] = NRF_WRITE_REG | RegAddr;
    memcpy(&tx_buf[1], pBuf, len);

    RF24L01_SET_CS_LOW();          // 片选

    // 使用DMA批量传输
    drv_spi_read_write_buffer_dma(tx_buf, NULL, len + 1);

    RF24L01_SET_CS_HIGH();         // 取消片选
}

/**
 * @brief :清空TX缓冲区
 * @param :无
 * @note  :无
 * @retval:无
 */
void NRF24L01_Flush_Tx_Fifo(void)
{
    RF24L01_SET_CS_LOW();      //片选

    drv_spi_read_write_byte( FLUSH_TX);    //清TX FIFO命令

    RF24L01_SET_CS_HIGH();     //取消片选
}
/**
 * @brief :清空RX缓冲区
 * @param :无
 * @note  :无
 * @retval:无
 */
void NRF24L01_Flush_Rx_Fifo(void)
{
    RF24L01_SET_CS_LOW();      //片选

    drv_spi_read_write_byte( FLUSH_RX);    //清RX FIFO命令

    RF24L01_SET_CS_HIGH();     //取消片选
}

/**
 * @brief :重新使用上一包数据
 * @param :无
 * @note  :无
 * @retval:无
 */
void NRF24L01_Reuse_Tx_Payload(void)
{
    RF24L01_SET_CS_LOW();      //片选

    drv_spi_read_write_byte( REUSE_TX_PL);     //重新使用上一包命令

    RF24L01_SET_CS_HIGH();     //取消片选
}

/**
 * @brief :NRF24L01空操作
 * @param :无
 * @note  :无
 * @retval:无
 */
void NRF24L01_Nop(void)
{
    RF24L01_SET_CS_LOW();      //片选

    drv_spi_read_write_byte( NOP);     //空操作命令

    RF24L01_SET_CS_HIGH();     //取消片选
}

/**
 * @brief :NRF24L01读状态寄存器
 * @param :无
 * @note  :无
 * @retval:RF24L01状态
 */
uint8_t NRF24L01_Read_Status_Register(void)
{
    uint8_t Status;

    RF24L01_SET_CS_LOW();      //片选

    Status = drv_spi_read_write_byte( NRF_READ_REG + STATUS);  //读状态寄存器

    RF24L01_SET_CS_HIGH();     //取消片选

    return Status;
}

/**
 * @brief :NRF24L01清中断
 * @param :
 @IRQ_Source:中断源
 * @note  :无
 * @retval:清除后状态寄存器的值
 */
uint8_t NRF24L01_Clear_IRQ_Flag(uint8_t IRQ_Source)
{
    uint8_t status;

    IRQ_Source &= (1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT); // 保留有效中断位
    status = NRF24L01_Read_Status_Register();                  // 读取当前状态

    HAL_GPIO_WritePin(GPIOB , GPIO_PIN_1 , GPIO_PIN_RESET); // 拉低CSN
    drv_spi_read_write_byte(NRF_WRITE_REG + STATUS);          // 发送写命令
    drv_spi_read_write_byte(status | IRQ_Source);             // 写1清中断标志[1]
    HAL_GPIO_WritePin(GPIOB , GPIO_PIN_1 , GPIO_PIN_SET);   // 拉高CSN

    return NRF24L01_Read_Status_Register();                   // 返回新状态
}

/**
 * @brief :读RF24L01中断状态
 * @param :无
 * @note  :无
 * @retval:中断状态
 */
uint8_t RF24L01_Read_IRQ_Status(void)
{
    return (NRF24L01_Read_Status_Register() & ((1 << RX_DR) | (1 << TX_DS) | (1 << MAX_RT)));   //返回中断状态
}

/**
 * @brief :读FIFO中数据宽度
 * @param :无
 * @note  :无
 * @retval:数据宽度
 */
uint8_t NRF24L01_Read_Top_Fifo_Width(void)
{
    uint8_t btmp;

    RF24L01_SET_CS_LOW();      //片选

    drv_spi_read_write_byte( R_RX_PL_WID); //读FIFO中数据宽度命令
    btmp = drv_spi_read_write_byte(0xFF); //读数据

    RF24L01_SET_CS_HIGH();     //取消片选

    return btmp;
}

/**
 * @brief :读接收到的数据
 * @param :无
 * @note  :无
 * @retval:
 @pRxBuf:数据存放地址首地址
 */
uint8_t NRF24L01_Read_Rx_Payload(uint8_t *pRxBuf)
{
    uint8_t Width, PipeNum;

    PipeNum = (NRF24L01_Read_Reg( STATUS) >> 1) & 0x07;  //读接收状态
    Width = NRF24L01_Read_Top_Fifo_Width();        //读接收数据个数

    RF24L01_SET_CS_LOW();      //片选
    drv_spi_read_write_byte( RD_RX_PLOAD);         //读有效数据命令

    for (PipeNum = 0; PipeNum < Width; PipeNum++)
    {
        *(pRxBuf + PipeNum) = drv_spi_read_write_byte(0xFF);        //读数据
    }
    RF24L01_SET_CS_HIGH();     //取消片选
    NRF24L01_Flush_Rx_Fifo();  //清空RX FIFO

    return Width;
}

/**
 * @brief :发送数据（带应答）
 * @param :
 *         @pTxBuf:发送数据地址
 *         @len:长度
 * @note  :一次不超过32个字节
 * @retval:无
 */
void NRF24L01_Write_Tx_Payload_Ack(uint8_t *pTxBuf, uint8_t len)
{
    uint8_t btmp;
    uint8_t length = (len > 32) ? 32 : len;       //数据长达大约32 则只发送32个

    NRF24L01_Flush_Tx_Fifo();      //清TX FIFO

    RF24L01_SET_CS_LOW();          //片选
    drv_spi_read_write_byte( WR_TX_PLOAD); //发送命令

    for (btmp = 0; btmp < length; btmp++)
    {
        drv_spi_read_write_byte(*(pTxBuf + btmp));  //发送数据
    }
    RF24L01_SET_CS_HIGH();         //取消片选
}

/**
 * @brief :发送数据（不带应答）
 * @param :
 *         @pTxBuf:发送数据地址
 *         @len:长度
 * @note  :一次不超过32个字节
 * @retval:无
 */
void NRF24L01_Write_Tx_Payload_NoAck(uint8_t *pTxBuf, uint8_t len)
{
    if (len > 32 || len == 0)
    {
        return;        //数据长度大于32 或者等于0 不执行
    }

    RF24L01_SET_CS_LOW();  //片选
    drv_spi_read_write_byte( WR_TX_PLOAD_NACK);    //发送命令
    while (len--)
    {
        drv_spi_read_write_byte(*pTxBuf);         //发送数据
        pTxBuf++;
    }
    RF24L01_SET_CS_HIGH();     //取消片选
}

/**
 * @brief :在接收模式下向TX FIFO写数据(带ACK)
 * @param :
 *         @pData:数据地址
 *         @len:长度
 * @note  :一次不超过32个字节
 * @retval:无
 */
void NRF24L01_Write_Tx_Payload_InAck(uint8_t *pData, uint8_t len)
{
    uint8_t btmp;

    len = (len > 32) ? 32 : len;      //数据长度大于32个则只写32个字节

    RF24L01_SET_CS_LOW();          //片选
    drv_spi_read_write_byte( W_ACK_PLOAD);     //命令
    for (btmp = 0; btmp < len; btmp++)
    {
        drv_spi_read_write_byte(*(pData + btmp));   //写数据
    }
    RF24L01_SET_CS_HIGH();         //取消片选
}

/**
 * @brief :设置发送地址
 * @param :
 *         @pAddr:地址存放地址
 *         @len:长度
 * @note  :无
 * @retval:无
 */
void NRF24L01_Set_TxAddr(uint8_t *pAddr, uint8_t len)
{
    len = (len > 5) ? 5 : len;                    //地址不能大于5个字节
    NRF24L01_Write_Buf( TX_ADDR , pAddr , len);  //写地址
}

/*​
 * @brief :设置发送地址(DMA版本)
 * @param :
 *         @pAddr:地址存放地址
 *         @len:长度
 */
void NRF24L01_Set_TxAddr_DMA(uint8_t *pAddr, uint8_t len)
{
    uint8_t tx_buf[6];  // 命令+5字节地址

    len = (len > 5) ? 5 : len;
    tx_buf[0] = NRF_WRITE_REG | TX_ADDR;
    memcpy(&tx_buf[1], pAddr, len);

    RF24L01_SET_CS_LOW();
    HAL_SPI_Transmit_DMA(&hspi1, tx_buf, len + 1);
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
    RF24L01_SET_CS_HIGH();
}


/**
 * @brief :设置接收通道地址
 * @param :
 *         @PipeNum:通道
 *         @pAddr:地址存肥着地址
 *         @Len:长度
 * @note  :通道不大于5 地址长度不大于5个字节
 * @retval:无
 */
void NRF24L01_Set_RxAddr(uint8_t PipeNum, uint8_t *pAddr, uint8_t Len)
{
    Len = (Len > 5) ? 5 : Len;
    PipeNum = (PipeNum > 5) ? 5 : PipeNum;        //通道不大于5 地址长度不大于5个字节

    NRF24L01_Write_Buf( RX_ADDR_P0 + PipeNum , pAddr , Len); //写入地址
}

/*​
 * @brief :设置接收通道地址(DMA版本)
 * @param :
 *         @PipeNum:通道
 *         @pAddr:地址存放地址
 *         @Len:长度
 */
void NRF24L01_Set_RxAddr_DMA(uint8_t PipeNum, uint8_t *pAddr, uint8_t Len)
{
    uint8_t tx_buf[6];  // 命令+5字节地址

    Len = (Len > 5) ? 5 : Len;
    PipeNum = (PipeNum > 5) ? 5 : PipeNum;

    tx_buf[0] = NRF_WRITE_REG | (RX_ADDR_P0 + PipeNum);
    memcpy(&tx_buf[1], pAddr, Len);

    RF24L01_SET_CS_LOW();
    HAL_SPI_Transmit_DMA(&hspi1, tx_buf, Len + 1);
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
    RF24L01_SET_CS_HIGH();
}

/**
 * @brief :设置通信速度
 * @param :
 *         @Speed:速度
 * @note  :无
 * @retval:无
 */
void NRF24L01_Set_Speed(nRf24l01SpeedType Speed)
{
    uint8_t btmp = 0;

    btmp = NRF24L01_Read_Reg( RF_SETUP);
    btmp &= ~((1 << 5) | (1 << 3));

    if (Speed == SPEED_250K)       //250K
    {
        btmp |= (1 << 5);
    }
    else if (Speed == SPEED_1M)   //1M
    {
        btmp &= ~((1 << 5) | (1 << 3));
    }
    else if (Speed == SPEED_2M)   //2M
    {
        btmp |= (1 << 3);
    }

    NRF24L01_Write_Reg( RF_SETUP , btmp);
}

/**
 * @brief :设置功率
 * @param :
 *         @Speed:速度
 * @note  :无
 * @retval:无
 */
void NRF24L01_Set_Power(nRf24l01PowerType Power)
{
    uint8_t btmp;

    btmp = NRF24L01_Read_Reg( RF_SETUP) & ~0x07;
    switch (Power)
    {
        case POWER_F18DBM:
            btmp |= PWR_18DB;
            break;
        case POWER_F12DBM:
            btmp |= PWR_12DB;
            break;
        case POWER_F6DBM:
            btmp |= PWR_6DB;
            break;
        case POWER_0DBM:
            btmp |= PWR_0DB;
            break;
        default:
            break;
    }
    NRF24L01_Write_Reg( RF_SETUP , btmp);
}

/**
 * @brief :设置频率
 * @param :
 *         @FreqPoint:频率设置参数
 * @note  :值不大于127
 * @retval:无
 */
void RF24LL01_Write_Hopping_Point(uint8_t FreqPoint)
{
    NRF24L01_Write_Reg( RF_CH , FreqPoint & 0x7F);
}

/**
 * @brief :NRF24L01检测
 * @param :无
 * @note  :无
 * @retval:无
 */
void NRF24L01_check(void)
{
    uint8_t i;
    uint8_t buf[5] = { 0XA5 , 0XA5 , 0XA5 , 0XA5 , 0XA5 };
    uint8_t read_buf[5] = { 0 };

    while (1)
    {
        NRF24L01_Write_Buf( TX_ADDR , buf , 5);          //写入5个字节的地址
        NRF24L01_Read_Buf( TX_ADDR , read_buf , 5);      //读出写入的地址
        for (i = 0; i < 5; i++)
        {
            if (buf[i] != read_buf[i])
            {
                break;
            }
        }

        if (5 == i)
        {
            break;
        }
        else
        {
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                HAL_Delay(100);
        }
    }
}

/**​
 * @brief :NRF24L01检测(DMA版本)
 * @param :无
 * @note  :使用DMA进行SPI传输检测模块
 * @retval:检测结果(0:失败, 1:成功)
 */
uint8_t NRF24L01_check_DMA(void)
{
    uint8_t i;
    uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    uint8_t read_buf[5] = {0};
    uint8_t tx_buf[6], rx_buf[6];
    uint32_t start_time = HAL_GetTick();

    while(1) {
        // 使用DMA写入地址
        tx_buf[0] = NRF_WRITE_REG | TX_ADDR;
        memcpy(&tx_buf[1], buf, 5);

        RF24L01_SET_CS_LOW();
        HAL_SPI_Transmit_DMA(&hspi1, tx_buf, 6);
        while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
        RF24L01_SET_CS_HIGH();

        // 使用DMA读取地址
        tx_buf[0] = NRF_READ_REG | TX_ADDR;
        memset(&tx_buf[1], 0xFF, 5);

        RF24L01_SET_CS_LOW();
        HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf, rx_buf, 6);
        while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
        RF24L01_SET_CS_HIGH();

        // 比较结果
        for(i = 0; i < 5; i++) {
            if(buf[i] != rx_buf[i+1]) {
                break;
            }
        }

        if(i == 5) {
            return 1; // 检测成功
        }

        // 超时处理(5秒超时)
        if(HAL_GetTick() - start_time > 5000) {
            return 0; // 检测失败
        }

        // 错误指示
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(100);
    }
}

/**
 * @brief :设置模式
 * @param :
 *         @Mode:模式发送模式或接收模式
 * @note  :无
 * @retval:无
 */
void RF24L01_Set_Mode(nRf24l01ModeType Mode)
{
    uint8_t controlreg = 0;
    controlreg = NRF24L01_Read_Reg( CONFIG);

    if (Mode == MODE_TX)
    {
        controlreg &= ~(1 << PRIM_RX);
    }
    else
    {
        if (Mode == MODE_RX)
        {
            controlreg |= (1 << PRIM_RX);
        }
    }

    NRF24L01_Write_Reg( CONFIG , controlreg);
}

/*​
 * @brief :设置模式(DMA版本)
 * @param :
 *         @Mode:模式发送模式或接收模式
 * @note  :使用DMA进行寄存器读写
 * @retval:无
 */
void RF24L01_Set_Mode_DMA(nRf24l01ModeType Mode)
{
    uint8_t tx_buf[2], rx_buf[2];

    // 使用DMA读取CONFIG寄存器
    tx_buf[0] = NRF_READ_REG | CONFIG;
    tx_buf[1] = 0xFF;

    RF24L01_SET_CS_LOW();
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf, rx_buf, 2);
    while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
    RF24L01_SET_CS_HIGH();

    uint8_t controlreg = rx_buf[1];

    // 修改模式位
    if (Mode == MODE_TX) {
        controlreg &= ~(1 << PRIM_RX);
    }
    else if (Mode == MODE_RX) {
        controlreg |= (1 << PRIM_RX);
    }

    // 使用DMA写入CONFIG寄存器
    tx_buf[0] = NRF_WRITE_REG | CONFIG;
    tx_buf[1] = controlreg;

    RF24L01_SET_CS_LOW();
    HAL_SPI_Transmit_DMA(&hspi1, tx_buf, 2);
    while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
    RF24L01_SET_CS_HIGH();
}

/**
 * @brief :NRF24L01发送一次数据
 * @param :
 *         @txbuf:待发送数据首地址
 *         @Length:发送数据长度
 * @note  :无
 * @retval:
 *         MAX_TX：达到最大重发次数
 *         TX_OK：发送完成
 *         0xFF:其他原因
 */
uint8_t NRF24L01_TxPacket(uint8_t *txbuf, uint8_t Length)
{
    uint8_t l_Status = 0;
    uint16_t l_MsTimes = 0;

    RF24L01_SET_CS_LOW();      //片选
    drv_spi_read_write_byte_dma( FLUSH_TX);
    RF24L01_SET_CS_HIGH();

    RF24L01_SET_CE_LOW();
    NRF24L01_Write_Buf_DMA( WR_TX_PLOAD , txbuf , Length);   //写数据到TX BUF 32字节  TX_PLOAD_WIDTH
    RF24L01_SET_CE_HIGH();         //启动发送
//    HAL_GPIO_TogglePin(GPIOC , GPIO_PIN_13);
    while (0 != RF24L01_GET_IRQ_STATUS())
    {
        HAL_Delay(1);
        if (500 == l_MsTimes++)                        //500ms还没有发送成功，重新初始化设备
        {
            NRF24L01_Gpio_Init();
            RF24L01_Init();
            RF24L01_Set_Mode(MODE_TX);
//            HAL_GPIO_TogglePin(GPIOC , GPIO_PIN_13);
            break;
        }
    }
//    HAL_GPIO_TogglePin(GPIOC , GPIO_PIN_13);
    l_Status = NRF24L01_Read_Reg(STATUS);                       //读状态寄存器
    NRF24L01_Write_Reg( STATUS , l_Status);                     //清除TX_DS或MAX_RT中断标志
//    HAL_GPIO_TogglePin(GPIOC , GPIO_PIN_13);
    if (l_Status & MAX_TX) //达到最大重发次数
    {
        NRF24L01_Write_Reg( FLUSH_TX , 0xff);    //清除TX FIFO寄存器
//        HAL_GPIO_TogglePin(GPIOC , GPIO_PIN_13);
        return MAX_TX;
    }
    if (l_Status & TX_OK)  //发送完成
    {
        return TX_OK;
//        HAL_GPIO_TogglePin(GPIOC , GPIO_PIN_13);
    }
//    HAL_GPIO_TogglePin(GPIOC , GPIO_PIN_13);
    return 0xFF;    //其他原因发送失败
}

/*​
 * @brief :NRF24L01发送一次数据(DMA版本)
 * @param :
 *         @txbuf:待发送数据首地址
 *         @Length:发送数据长度
 * @note  :使用DMA进行SPI传输，提高效率
 * @retval:
 *         MAX_TX：达到最大重发次数
 *         TX_OK：发送完成
 *         0xFF:其他原因
 */
uint8_t NRF24L01_TxPacket_DMA(uint8_t *txbuf, uint8_t Length)
{
    uint8_t l_Status = 0;
    uint32_t start_time = HAL_GetTick();
    uint8_t tx_cmd = WR_TX_PLOAD;

    // 清空TX FIFO
    RF24L01_SET_CS_LOW();
    drv_spi_read_write_byte_dma(FLUSH_TX);
    RF24L01_SET_CS_HIGH();

    RF24L01_SET_CE_LOW();

    // 使用DMA发送数据
    RF24L01_SET_CS_LOW();

    // 先发送命令字节
    drv_spi_read_write_byte_dma(tx_cmd);

    // 然后使用DMA发送数据负载
    HAL_SPI_Transmit_DMA(&hspi1, txbuf, Length);

    // 等待DMA传输完成
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

    RF24L01_SET_CS_HIGH();
    RF24L01_SET_CE_HIGH();  // 启动发送

    // 等待传输完成或超时
    while (RF24L01_GET_IRQ_STATUS() != 0)
    {
        if (HAL_GetTick() - start_time > 100)  // 500ms超时
        {
            NRF24L01_Gpio_Init();
            RF24L01_Init_DMA();
            RF24L01_Set_Mode_DMA(MODE_TX);
            break;
        }
        HAL_Delay(1);
    }


    // 读取并清除状态
    l_Status = NRF24L01_Read_Reg_DMA(STATUS);
    NRF24L01_Write_Reg_DMA(STATUS, l_Status);

    if (l_Status & MAX_TX) // 达到最大重发次数
    {
        NRF24L01_Write_Reg_DMA(FLUSH_TX, 0xff);
        return MAX_TX;
    }
    if (l_Status & TX_OK)  // 发送完成
    {
        return TX_OK;
    }

    return 0xFF;  // 其他原因发送失败
}

/**
 * @brief :NRF24L01接收数据
 * @param :
 *         @rxbuf:接收数据存放地址
 * @note  :无
 * @retval:接收的数据个数
 */
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
    uint8_t l_Status = 0, l_RxLength = 0, l_100MsTimes = 0;

    RF24L01_SET_CS_LOW();      //片选
    drv_spi_read_write_byte( FLUSH_RX);
    RF24L01_SET_CS_HIGH();

    while (0 != RF24L01_GET_IRQ_STATUS())
    {
        HAL_Delay(100);

        if (30 == l_100MsTimes++)      //3s没接收过数据，重新初始化模块
        {
            NRF24L01_Gpio_Init();
            RF24L01_Init();
            RF24L01_Set_Mode(MODE_RX);
            break;
        }
    }

    l_Status = NRF24L01_Read_Reg( STATUS);     //读状态寄存器
    NRF24L01_Write_Reg( STATUS , l_Status);      //清中断标志
    if (l_Status & RX_OK)   //接收到数据
    {
        l_RxLength = NRF24L01_Read_Reg( R_RX_PL_WID);      //读取接收到的数据个数
        NRF24L01_Read_Buf( RD_RX_PLOAD , rxbuf , l_RxLength);  //接收到数据
        NRF24L01_Write_Reg( FLUSH_RX , 0xff);                //清除RX FIFO
        return l_RxLength;
    }

    return 0;               //没有收到数据
}

/**
 * @brief :RF24L01引脚初始化
 * @param :无
 * @note  :无
 * @retval:无
 */
void NRF24L01_Gpio_Init(void)
{

    HAL_GPIO_WritePin(GPIOB , GPIO_PIN_0 , GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA , GPIO_PIN_4 , GPIO_PIN_SET);

    RF24L01_SET_CE_LOW();      //??24L01
    RF24L01_SET_CS_HIGH();     //??SPI??

}

/**
 * @brief :RF24L01模块初始化
 * @param :无
 * @note  :无
 * @retval:无
 */
void RF24L01_Init(void)
{
    uint8_t addr[5] = { INIT_ADDR };

    RF24L01_SET_CE_HIGH();
    NRF24L01_Clear_IRQ_Flag( IRQ_ALL);
#if DYNAMIC_PACKET == 1

    NRF24L01_Write_Reg( DYNPD , (1 << 0));    //使能通道1动态数据长度
    NRF24L01_Write_Reg( FEATRUE , 0x07);
    NRF24L01_Read_Reg( DYNPD);
    NRF24L01_Read_Reg( FEATRUE);

#elif DYNAMIC_PACKET == 0

    L01_WriteSingleReg( L01REG_RX_PW_P0, FIXED_PACKET_LEN );    //固定数据长度

#endif  //DYNAMIC_PACKET

    NRF24L01_Write_Reg( CONFIG , /*( 1<<MASK_RX_DR ) |*/     //接收中断
            (1 << EN_CRC) |     //使能CRC 1个字节
                    (1 << PWR_UP));    //开启设备
    NRF24L01_Write_Reg( EN_AA , (1 << ENAA_P0));          //通道0自动应答
    NRF24L01_Write_Reg( EN_RXADDR , (1 << ERX_P0));       //通道0接收
    NRF24L01_Write_Reg( SETUP_AW , AW_5BYTES);              //地址宽度 5个字节
    NRF24L01_Write_Reg( SETUP_RETR , ARD_4000US | ( REPEAT_CNT & 0x0F));            //重复等待时间 250us
    NRF24L01_Write_Reg( RF_CH , 60);                        //初始化通道
    NRF24L01_Write_Reg( RF_SETUP , 0x26);

    NRF24L01_Set_TxAddr(&addr[0] , 5);                      //设置TX地址
    NRF24L01_Set_RxAddr(0 , &addr[0] , 5);                   //设置RX地址

//    NRF24L01_Set_Speed(SPEED_1M);
//    NRF24L01_Set_Power(POWER_F18DBM);
}

/*​
 * @brief :RF24L01模块初始化(DMA版本)
 * @param :无
 * @note  :使用DMA批量配置寄存器，提高初始化速度
 * @retval:无
 */
void RF24L01_Init_DMA(void)
{
    uint8_t addr[5] = { INIT_ADDR };

    // 初始化状态
    RF24L01_SET_CE_HIGH();
    NRF24L01_Clear_IRQ_Flag(IRQ_ALL);

    // 批量配置寄存器结构体
    typedef struct {
        uint8_t reg;
        uint8_t val;
    } reg_config_t;

    // 寄存器配置数组
    static const reg_config_t init_config[] = {
    #if DYNAMIC_PACKET == 1
        {DYNPD, (1 << 0)},      // 使能通道1动态数据长度
        {FEATRUE, 0x07},
    #elif DYNAMIC_PACKET == 0
        {RX_PW_P0, FIXED_PACKET_LEN}, // 固定数据长度
    #endif
        {CONFIG, (1 << EN_CRC) | (1 << PWR_UP)},
        {EN_AA, (1 << ENAA_P0)},
        {EN_RXADDR, (1 << ERX_P0)},
        {SETUP_AW, AW_5BYTES},
        {SETUP_RETR, ARD_4000US | (REPEAT_CNT & 0x0F)},
        {RF_CH, 60},
        {RF_SETUP, 0x26}
    };

    // 使用DMA批量写入寄存器配置
    for (int i = 0; i < sizeof(init_config)/sizeof(init_config[0]); i++) {
        uint8_t tx_buf[2] = {NRF_WRITE_REG | init_config[i].reg, init_config[i].val};

        RF24L01_SET_CS_LOW();
        HAL_SPI_Transmit_DMA(&hspi1, tx_buf, 2);
        while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
        RF24L01_SET_CS_HIGH();

        // 添加必要的延时(某些寄存器写入后需要稳定时间)
        if(init_config[i].reg == CONFIG || init_config[i].reg == RF_SETUP) {
            HAL_Delay(1);
        }
    }

    // 设置地址(使用DMA版本)
    NRF24L01_Set_TxAddr_DMA(&addr[0], 5);       // 设置TX地址
    NRF24L01_Set_RxAddr_DMA(0, &addr[0], 5);    // 设置RX地址

    // 可选的速度和功率设置
    // NRF24L01_Set_Speed_DMA(SPEED_1M);
    // NRF24L01_Set_Power_DMA(POWER_F18DBM);
}

