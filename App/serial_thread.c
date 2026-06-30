#include "serial_thread.h"
#include "cmsis_os.h"
#include "usart_printf.h"
#include "foc_thread.h"
<<<<<<< HEAD
=======
#include "mt6816_encoder.h"
>>>>>>> 57f1f94 (初次提交FOC代码)
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

<<<<<<< HEAD
=======


>>>>>>> 57f1f94 (初次提交FOC代码)
/*=================== 命令交互结构体 ===================*/
CommandFlag_t command_flag = {0}; // 命令交互结构体

/*=================== 参数表 ===================*/

/**
 * @brief  参数表: 定义所有可通过串口访问的参数
 *
 * 添加新参数只需在此表中增加一条记录:
 * { "参数名", 类型, &变量指针, "单位", "描述" }
 *
 * 类型可用: PARAM_TYPE_FLOAT / PARAM_TYPE_INT / PARAM_TYPE_UINT32
 */
static const ParamDef_t g_param_table[] =
{
    /* ---- 可读写参数 ---- */
    { "current_zero",   PARAM_TYPE_FLOAT,  &command_flag.current_zero,   "",   "1:电流环0点标定"       },
    { "vq_set",         PARAM_TYPE_FLOAT,  &motor.foc.vq_set,            "V",   "开环电压设置"       },
    { "ctrl_mode",      PARAM_TYPE_INT,    &motor.Control_Mode,          "", "[只读] 控制模式"      },
    /* ---- 只读参数 ---- */
<<<<<<< HEAD
    { "angle",          PARAM_TYPE_FLOAT,  &motor.mt6816.angle, "",   "[只读] 机械角度" },  
    { "speed",          PARAM_TYPE_FLOAT,  &motor.foc.theta, "",   "[只读] 电角度(rad)" },
=======
    { "angle",          PARAM_TYPE_FLOAT,  &encoder_data.mec_angle,        "",   "[只读] 机械角度(弧度)" },
    { "angle_deg",      PARAM_TYPE_FLOAT,  &encoder_data.mec_angle_deg,    "",   "[只读] 机械角度(角度)" },
    { "elec_angle",          PARAM_TYPE_FLOAT,  &encoder_data.elec_angle,  "",   "[只读] 电角度(rad)" },
    { "pos_estimate",   PARAM_TYPE_FLOAT,  &encoder_data.pos_estimate_,    "",   "[只读] 圈数" },
>>>>>>> 57f1f94 (初次提交FOC代码)
    { "i_d",            PARAM_TYPE_FLOAT,  &motor.foc.i_d,   "A",  "[只读] D轴电流"      },
    { "i_q",            PARAM_TYPE_FLOAT,  &motor.foc.i_q,   "A",  "[只读] Q轴电流"      },
    { "i_a",            PARAM_TYPE_FLOAT,  &motor.foc.i_a,   "A",  "[只读] A相电流"      },
    { "i_b",            PARAM_TYPE_FLOAT,  &motor.foc.i_b,   "A",  "[只读] B相电流"      },
    { "i_c",            PARAM_TYPE_FLOAT,  &motor.foc.i_c,   "A",  "[只读] C相电流"      },
    { "v_bus",          PARAM_TYPE_FLOAT,  &motor.foc.vbus,  "V",  "[只读] 母线电压"      },
    { "state_mode",     PARAM_TYPE_INT,    &motor.State_Mode,"",   "[只读] 运行状态"      },
    { "ia_zero",     PARAM_TYPE_INT,       &motor.foc.flash_data.ia_zero,"", "[只读] A相电流偏移"      },
    { "ib_zero",     PARAM_TYPE_INT,       &motor.foc.flash_data.ib_zero,"", "[只读] B相电流偏移"      },
};

#define PARAM_TABLE_SIZE (sizeof(g_param_table) / sizeof(g_param_table[0]))

/*=================== 数值读取 ===================*/

/**
 * @brief  从参数表中读取指定参数的数值（float格式）
 * @param  param: 参数表条目指针
 * @param  val:   输出数值
 * @retval 0: 成功, -1: 失败
 */
static int Param_ReadFloat(const ParamDef_t *param, float *val)
{
    if (param == NULL || param->var == NULL)
        return -1;

    switch (param->type)
    {
        case PARAM_TYPE_FLOAT:
            *val = *(float *)param->var;
            break;
        case PARAM_TYPE_INT:
            *val = (float)(*(int *)param->var);
            break;
        case PARAM_TYPE_UINT32:
            *val = (float)(*(uint32_t *)param->var);
            break;
        default:
            return -1;
    }
    return 0;
}

/**
 * @brief  向参数表条目写入数值（float格式转换为目标类型）
 * @param  param: 参数表条目指针
 * @param  val:   要写入的数值
 * @retval 0: 成功, -1: 失败
 */
static int Param_WriteFloat(const ParamDef_t *param, float val)
{
    if (param == NULL || param->var == NULL)
        return -1;

    switch (param->type)
    {
        case PARAM_TYPE_FLOAT:
            *(float *)param->var = val;
            break;
        case PARAM_TYPE_INT:
            *(int *)param->var = (int)val;
            break;
        case PARAM_TYPE_UINT32:
            *(uint32_t *)param->var = (uint32_t)val;
            break;
        default:
            return -1;
    }
    return 0;
}

/**
 * @brief  按名称查找参数表
 * @param  name: 参数名
 * @retval 参数表条目指针, NULL=未找到
 */
static const ParamDef_t *Param_Find(const char *name)
{
    for (int i = 0; i < PARAM_TABLE_SIZE; i++)
    {
        if (strcmp(g_param_table[i].name, name) == 0)
        {
            return &g_param_table[i];
        }
    }
    return NULL;
}

/*=================== 辅助函数 ===================*/

/**
 * @brief  不区分大小写的字符串比较
 */
static int str_icmp(const char *a, const char *b)
{
    while (*a && *b)
    {
        char ca = *a++;
        char cb = *b++;
        if (ca >= 'A' && ca <= 'Z') ca += 0x20;
        if (cb >= 'A' && cb <= 'Z') cb += 0x20;
        if (ca != cb) return ca - cb;
    }
    return (int)(*a - *b);
}

/**
 * @brief  将float格式化为字符串（避免依赖printf的%f支持）
 * @param  val:  要格式化的浮点数
 * @param  buf:  输出缓冲区
 * @param  dec:  小数位数
 * @note   newlib-nano默认不支持%f，用此函数替代
 */
static void FloatToStr(float val, char *buf, int dec)
{
    int int_part, dec_part, i, power = 1;
    char *p = buf;
    int neg = 0;

    if (val < 0) { neg = 1; val = -val; }

    int_part = (int)val;
    for (i = 0; i < dec; i++) power *= 10;
    dec_part = (int)((val - (float)int_part) * power + 0.5f);

    /* 处理小数进位 */
    if (dec_part >= power) { int_part++; dec_part -= power; }

    /* 符号 */
    if (neg) *p++ = '-';

    /* 整数部分 */
    p += sprintf(p, "%d", int_part);

    /* 小数部分 */
    if (dec > 0)
    {
        *p++ = '.';
        for (i = dec - 1; i > 0; i--)
        {
            int d = 1;
            for (int j = 0; j < i; j++) d *= 10;
            *p++ = (char)('0' + (dec_part / d) % 10);
        }
        *p++ = (char)('0' + dec_part % 10);
    }
    *p = '\0';
}

/*=================== 命令处理器 ===================*/

/**
 * @brief  处理 HELP 命令
 */
static void Cmd_Help(void)
{
    UART_Response_DMA("$HELP,Available commands:");
    UART_Response_DMA("$HELP,  $GET,<param>       - Read parameter");
    UART_Response_DMA("$HELP,  $SET,<param>,<val> - Write parameter");
    UART_Response_DMA("$HELP,  $INFO              - List all parameters");
    UART_Response_DMA("$HELP,  $HELP              - Show this help");
}

/**
 * @brief  处理 INFO 命令 — 列出所有参数
 */
static void Cmd_Info(void)
{
    float val;
    char str_val[16];

    UART_Response_DMA("$INFO,--- Parameter List ---");
    for (int i = 0; i < PARAM_TABLE_SIZE; i++)
    {
        if (Param_ReadFloat(&g_param_table[i], &val) == 0)
        {
            FloatToStr(val, str_val, 4);
            UART_Response_DMA("$INFO,%s = %s: %s %s",
                g_param_table[i].name,
                g_param_table[i].desc,
                str_val,
                g_param_table[i].unit);
        }
    }
    UART_Response_DMA("$INFO,--- End ---");
}

/**
 * @brief  处理 GET 命令 — 读取参数
 * @param  param_name: 参数名
 */
static void Cmd_Get(const char *param_name)
{
    float val;
    char str_val[16];
    const ParamDef_t *param = Param_Find(param_name);

    if (param == NULL)
    {
        UART_Response_DMA("$ERR,unknown parameter '%s'", param_name);
        return;
    }

    if (Param_ReadFloat(param, &val) == 0)
    {
        FloatToStr(val, str_val, 4);
        UART_Response_DMA("$%s=%s", param_name, str_val);
    }
    else
    {
        UART_Response_DMA("$ERR,read failed");
    }
}

/**
 * @brief  处理 SET 命令 — 设置参数
 * @param  param_name: 参数名
 * @param  value_str:  数值字符串
 */
static void Cmd_Set(const char *param_name, const char *value_str)
{
    float val;
    char str_val[16];
    const ParamDef_t *param = Param_Find(param_name);

    if (param == NULL)
    {
        UART_Response_DMA("$ERR,unknown parameter '%s'", param_name);
        return;
    }

    /* 字符串转float */
    val = atof(value_str);

    if (Param_WriteFloat(param, val) == 0)
    {
        /* 特殊处理 motor_enable */
        if (strcmp(param_name, "motor_enable") == 0)
        {
            if ((int)val)
            {
                enable_foc_pwm_driver();
            }else
            {
                disable_foc_pwm_driver();
            }
        }

        FloatToStr(val, str_val, 4);
        UART_Response_DMA("$OK,%s=%s", param_name, str_val);
    }
    else
    {
        UART_Response_DMA("$ERR,write failed");
    }
}

/**
 * @brief  协议命令回调（由 usart_printf 解析后调用）
 * @param  cmd: 解析后的命令结构体
 */
static void Protocol_CommandHandler(const ProtocolCmd_t *cmd)
{
    if (cmd == NULL) return;

    /* 根据命令类型分发（不区分大小写） */
    if (str_icmp(cmd->cmd, "HELP") == 0)
    {
        Cmd_Help();
    }
    else if (str_icmp(cmd->cmd, "INFO") == 0)
    {
        Cmd_Info();
    }
    else if (str_icmp(cmd->cmd, "GET") == 0)
    {
        if (cmd->argc >= 1)
            Cmd_Get(cmd->args[0]);
        else
            UART_Response_DMA("$ERR,usage: $GET,<param>");
    }
    else if (str_icmp(cmd->cmd, "SET") == 0)
    {
        if (cmd->argc >= 2)
            Cmd_Set(cmd->args[0], cmd->args[1]);
        else
            UART_Response_DMA("$ERR,usage: $SET,<param>,<value>");
    }
    else
    {
        UART_Response_DMA("$ERR,unknown command '%s'", cmd->cmd);
    }
}

/*=================== 串口线程 ===================*/

void serial_thread(void *param)
{
    /* 启动DMA循环接收（CIRCULAR + IDLE中断） */
    UART_StartRX_DMA();

    /* 注册协议命令回调 */
    UART_RegisterProtocolCallback(Protocol_CommandHandler);

    /* 输出启动信息 */
    UART_Printf_DMA("\r\n=======================================\r\n");
    UART_Printf_DMA("  MY-FOC Serial Protocol Ready\r\n");
    UART_Printf_DMA("  Type $HELP for available commands\r\n");
    UART_Printf_DMA("=======================================\r\n");

    while(1)
    {
        /* 处理串口协议（检查是否有完整行到达） */
        UART_ProtocolProcess();

        osDelay(5);
    }
}