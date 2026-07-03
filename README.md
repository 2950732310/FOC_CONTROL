# FOC_CONTROL
无刷电机FOC控制器

日期：2026-07-01
变更人：ccq
变更内容：
    1.优化MT6816编码器驱动代码
    2.变更FOC控制框架，由DMA ADC读取数据->FOC控制->PWM输出 变更为 ADC中断读取数据->FOC控制->PWM输出