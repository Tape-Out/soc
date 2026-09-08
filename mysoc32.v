/**
 * mysoc32.v - 基于PicoRV32的SoC系统集成
 *
 * 本模块将PicoRV32 RISC-V处理器核心与多个外设模块集成在一起，包括：
 * - CLINT（核本地中断控制器）
 * - PLIC（平台级中断控制器）
 * - UART（串口通信）
 * - PWM（脉冲宽度调制）
 * - GPIO（通用输入输出）
 * - MROM（启动ROM）
 * - WDT（看门狗定时器）
 * - Timer（通用定时器）
 * - SRAM（静态存储器控制器）
 * - RTC（实时时钟）
 * - 1-Wire（单总线接口）
 */
module mysoc32 (
    input  wire         clk,        // 系统时钟
    input  wire         resetn,     // 系统复位（低有效）

    // GPIO
    input  wire [31:0]  gpio_in,    // 32位GPIO输入
    output wire [31:0]  gpio_out,   // 32位GPIO输出
    output wire [31:0]  gpio_dir,   // GPIO方向控制（1=output，0=input）

    // UART串口信号
    input  wire         uart_rx,    // UART接收数据线
    output wire         uart_tx,    // UART发送数据线
    input  wire         uart_cts,   // UART清除发送（硬件流控）
    output wire         uart_rts,   // UART请求发送（硬件流控）

    // 1-Wire单总线信号
    inout  wire         wire_io,    // 1-Wire双向数据线

    // RTC实时时钟信号
    input  wire         ext_clk,    // RTC外部时钟（通常为32.768kHz）
    input  wire         use_ext_clk,// 选择使用外部时钟（1=外部，0=系统）

    // PWM输出信号
    output wire [7:0]   pwm_out,    // 8通道PWM输出

    // 看门狗复位输出
    output wire         wdt_reset,  // 看门狗复位信号（高有效）

    // 处理器陷阱指示
    output wire         trap        // CPU异常/陷阱指示
);

    // =========================================================================
    // 内部信号定义区
    // =========================================================================

    // CPU内存总线接口信号
    wire        mem_valid;      // 内存访问有效
    wire        mem_instr;      // 指令访问标识（1=取指，0=数据）
    wire        mem_ready;      // 内存访问完成
    wire [31:0] mem_addr;       // 内存地址
    wire [31:0] mem_wdata;      // 写入数据
    wire [3:0]  mem_wstrb;      // 字节写使能
    wire [31:0] mem_rdata;      // 读取数据

    // 中断相关信号
    wire [31:0] irq;            // 32位中断输入到CPU
    wire [31:0] eoi;            // 中断结束确认（每个位对应一个中断源）

    // 各个外设的内存接口信号
    // 每个外设有独立的ready和rdata信号，用于总线复用
    wire        clint_mem_ready;
    wire [31:0] clint_mem_rdata;

    wire        plic_mem_ready;
    wire [31:0] plic_mem_rdata;

    wire        gpio_mem_ready;
    wire [31:0] gpio_mem_rdata;

    wire        uart_mem_ready;
    wire [31:0] uart_mem_rdata;

    wire        pwm_mem_ready;
    wire [31:0] pwm_mem_rdata;

    wire        timer_mem_ready;
    wire [31:0] timer_mem_rdata;

    wire        wdt_mem_ready;
    wire [31:0] wdt_mem_rdata;

    wire        rtc_mem_ready;
    wire [31:0] rtc_mem_rdata;

    wire        sram_mem_ready;
    wire [31:0] sram_mem_rdata;

    wire        mrom_mem_ready;
    wire [31:0] mrom_mem_rdata;

    wire        wire_mem_ready;
    wire [31:0] wire_mem_rdata;

    // 各个外设的中断信号
    wire        clint_timer_irq;    // CLINT定时器中断
    wire        clint_software_irq; // CLINT软件中断

    wire        gpio_irq;           // GPIO中断
    wire        uart_irq;           // UART中断
    wire        pwm_irq;            // PWM中断
    wire        timer_irq;          // 定时器中断
    wire        wdt_irq;            // 看门狗中断
    wire        rtc_irq;            // RTC中断
    wire        sram_irq;           // SRAM中断
    wire        mrom_irq;           // ROM中断
    wire        wire_irq;           // 1-Wire中断

    // PLIC专用信号
    wire [`PLIC_MAX_CONTEXTS-1:0] plic_irq_pending;   // 中断等待处理
    wire [`PLIC_MAX_CONTEXTS-1:0] plic_irq_claim;     // 中断认领
    wire [`PLIC_MAX_CONTEXTS-1:0] plic_irq_complete;  // 中断完成

    // CLINT机器时间计数器
    wire [63:0] mtime;              // 64位机器时间计数器

    // =========================================================================
    // 地址解码逻辑
    // 根据内存地址确定访问哪个外设
    // =========================================================================

    // 每个外设的地址范围定义：
    wire is_clint   = (mem_addr >= 32'h0200_0000) && (mem_addr < 32'h0200_1000);  // CLINT: 0x0200_0000-0x0200_0FFF
    wire is_plic    = (mem_addr >= 32'h0C00_0000) && (mem_addr < 32'h0C00_1000);  // PLIC:  0x0C00_0000-0x0C00_0FFF
    wire is_uart    = (mem_addr >= 32'h8100_1000) && (mem_addr < 32'h8100_2000);  // UART:  0x8100_1000-0x8100_1FFF
    wire is_pwm     = (mem_addr >= 32'h8100_2000) && (mem_addr < 32'h8100_3000);  // PWM:   0x8100_2000-0x8100_2FFF
    wire is_gpio    = (mem_addr >= 32'h8100_4000) && (mem_addr < 32'h8100_5000);  // GPIO:  0x8100_4000-0x8100_4FFF
    wire is_mrom    = (mem_addr >= 32'h8100_5000) && (mem_addr < 32'h8100_6000);  // MROM:  0x8100_5000-0x8100_5FFF
    wire is_wdt     = (mem_addr >= 32'h8100_6000) && (mem_addr < 32'h8100_7000);  // WDT:   0x8100_6000-0x8100_6FFF
    wire is_timer   = (mem_addr >= 32'h8100_7000) && (mem_addr < 32'h8100_8000);  // Timer: 0x8100_7000-0x8100_7FFF
    wire is_sram    = (mem_addr >= 32'h8100_8000) && (mem_addr < 32'h8100_9000);  // SRAM:  0x8100_8000-0x8100_8FFF
    wire is_rtc     = (mem_addr >= 32'h8100_9000) && (mem_addr < 32'h8100_A000);  // RTC:   0x8100_9000-0x8100_9FFF
    wire is_wire    = (mem_addr >= 32'h8200_0000) && (mem_addr < 32'h8200_1000);  // 1-Wire:0x8200_0000-0x8200_0FFF

    // =========================================================================
    // 内存总线多路复用器
    // 将各个外设的读取数据和ready信号复用到CPU总线上
    // =========================================================================

    // 读取数据多路选择：根据地址选择对应外设的读取数据
    always @* begin
        case (1'b1)
            is_clint:  mem_rdata = clint_mem_rdata;   // CLINT读取数据
            is_plic:   mem_rdata = plic_mem_rdata;    // PLIC读取数据
            is_uart:   mem_rdata = uart_mem_rdata;    // UART读取数据
            is_pwm:    mem_rdata = pwm_mem_rdata;     // PWM读取数据
            is_gpio:   mem_rdata = gpio_mem_rdata;    // GPIO读取数据
            is_mrom:   mem_rdata = mrom_mem_rdata;    // MROM读取数据
            is_wdt:    mem_rdata = wdt_mem_rdata;     // WDT读取数据
            is_timer:  mem_rdata = timer_mem_rdata;   // Timer读取数据
            is_sram:   mem_rdata = sram_mem_rdata;    // SRAM读取数据
            is_rtc:    mem_rdata = rtc_mem_rdata;     // RTC读取数据
            is_wire:   mem_rdata = wire_mem_rdata;    // 1-Wire读取数据
            default:   mem_rdata = 32'h0000_0000;     // 默认返回0
        endcase
    end

    // Ready信号组合逻辑：只有被选中的外设的ready信号有效
    assign mem_ready =
        (is_clint  ? clint_mem_ready  : 1'b0) |  // CLINT就绪
        (is_plic   ? plic_mem_ready   : 1'b0) |  // PLIC就绪
        (is_uart   ? uart_mem_ready   : 1'b0) |  // UART就绪
        (is_pwm    ? pwm_mem_ready    : 1'b0) |  // PWM就绪
        (is_gpio   ? gpio_mem_ready   : 1'b0) |  // GPIO就绪
        (is_mrom   ? mrom_mem_ready   : 1'b0) |  // MROM就绪
        (is_wdt    ? wdt_mem_ready    : 1'b0) |  // WDT就绪
        (is_timer  ? timer_mem_ready  : 1'b0) |  // Timer就绪
        (is_sram   ? sram_mem_ready   : 1'b0) |  // SRAM就绪
        (is_rtc    ? rtc_mem_ready    : 1'b0) |  // RTC就绪
        (is_wire   ? wire_mem_ready   : 1'b0);   // 1-Wire就绪

    // =========================================================================
    // 中断源连接
    // 将各个外设的中断信号连接到PLIC中断控制器
    // =========================================================================

    // PLIC中断源拼接：将各个外设中断分配到PLIC的不同中断源
    wire [`PLIC_MAX_SOURCES-1:0] irq_sources;
    assign irq_sources = {
        {`PLIC_MAX_SOURCES-10{1'b0}},  // 高位填充0
        wire_irq,      // 源9: 1-Wire中断
        rtc_irq,       // 源8: RTC中断
        sram_irq,      // 源7: SRAM中断
        timer_irq,     // 源6: 定时器中断
        wdt_irq,       // 源5: 看门狗中断
        mrom_irq,      // 源4: ROM中断
        gpio_irq,      // 源3: GPIO中断
        pwm_irq,       // 源2: PWM中断
        uart_irq,      // 源1: UART中断
        clint_timer_irq // 源0: CLINT定时器中断
    };

    // =========================================================================
    // PicoRV32 CPU核心实例化
    // =========================================================================

    picorv32 #(
        // CPU配置参数
        .ENABLE_COUNTERS(1),        // 启用性能计数器
        .ENABLE_COUNTERS64(1),      // 启用64位计数器
        .ENABLE_REGS_16_31(1),      // 启用寄存器16-31
        .ENABLE_REGS_DUALPORT(1),   // 启用双端口寄存器文件
        .LATCHED_MEM_RDATA(0),      // 不使用锁存的读取数据
        .TWO_STAGE_SHIFT(1),        // 使用两级移位器
        .BARREL_SHIFTER(0),         // 禁用桶形移位器
        .TWO_CYCLE_COMPARE(0),      // 禁用两周期比较
        .TWO_CYCLE_ALU(0),          // 禁用两周期ALU
        .COMPRESSED_ISA(0),         // 禁用压缩指令集
        .CATCH_MISALIGN(1),         // 捕获不对齐访问
        .CATCH_ILLINSN(1),          // 捕获非法指令
        .ENABLE_PCPI(0),            // 禁用协处理器接口
        .ENABLE_MUL(0),             // 禁用乘法器
        .ENABLE_FAST_MUL(0),        // 禁用快速乘法
        .ENABLE_DIV(0),             // 禁用除法器
        .ENABLE_IRQ(1),             // 启用中断支持
        .ENABLE_IRQ_QREGS(1),       // 启用快速中断寄存器
        .ENABLE_IRQ_TIMER(1),       // 启用定时器中断
        .ENABLE_TRACE(0),           // 禁用跟踪
        .REGS_INIT_ZERO(0),         // 寄存器不初始化为0
        .MASKED_IRQ(32'h0000_0000), // 不屏蔽任何中断
        .LATCHED_IRQ(32'hffff_ffff),// 锁存中断信号
        .PROGADDR_RESET(32'h8000_0000), // 程序起始地址
        .PROGADDR_IRQ(32'h0000_0010),   // 中断处理程序地址
        .STACKADDR(32'hffff_ffff)       // 堆栈地址
    ) cpu (
        .clk(clk),
        .resetn(resetn & ~wdt_reset),  // 看门狗可以复位CPU（低有效与看门狗高有效结合）
        .trap(trap),                // 陷阱输出

        // 内存总线接口
        .mem_valid(mem_valid),
        .mem_instr(mem_instr),
        .mem_ready(mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(mem_rdata),

        // 前瞻内存接口（未使用，悬空）
        .mem_la_read(),
        .mem_la_write(),
        .mem_la_addr(),
        .mem_la_wdata(),
        .mem_la_wstrb(),

        // 协处理器接口（未使用，固定值）
        .pcpi_valid(),
        .pcpi_insn(),
        .pcpi_rs1(),
        .pcpi_rs2(),
        .pcpi_wr(1'b0),
        .pcpi_rd(32'b0),
        .pcpi_wait(1'b0),
        .pcpi_ready(1'b0),

        // 中断接口
        .irq(irq),      // 中断输入
        .eoi(eoi)       // 中断结束输出
    );

    // =========================================================================
    // 外设模块实例化
    // 每个外设都连接到系统总线，并在特定地址范围内响应
    // =========================================================================

    // CLINT - 核本地中断控制器（处理定时器和软件中断）
    clint_mmio #(
        .BASE_ADDR(32'h0200_0000),
        .CLK_FREQ(32'd100_000_000)  // 100MHz时钟
    ) clint_inst (
        .clk(clk),
        .resetn(resetn),

        // 内存总线接口（只在CLINT地址范围内有效）
        .mem_valid(mem_valid & is_clint),
        .mem_instr(mem_instr),
        .mem_ready(clint_mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(clint_mem_rdata),

        // CLINT特定信号
        .mtime(mtime),                  // 机器时间计数器
        .timer_irq(clint_timer_irq),    // 定时器中断
        .software_irq(clint_software_irq), // 软件中断
        .eoi(eoi[0])                    // 中断结束确认
    );

    // PLIC - 平台级中断控制器（管理所有外设中断）
    plic_mmio #(
        .BASE_ADDR(32'h0C00_0000),
        .MAX_SOURCES(`PLIC_MAX_SOURCES),    // 最大中断源数
        .MAX_CONTEXTS(`PLIC_MAX_CONTEXTS),  // 最大上下文数
        .PRIO_BITS(3)                       // 优先级位数（0-7级）
    ) plic_inst (
        .clk(clk),
        .resetn(resetn),

        // 内存总线接口
        .mem_valid(mem_valid & is_plic),
        .mem_instr(mem_instr),
        .mem_ready(plic_mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(plic_mem_rdata),

        // PLIC中断管理
        .irq_sources(irq_sources),      // 所有中断源输入
        .irq_pending(plic_irq_pending), // 等待处理的中断
        .irq_claim(plic_irq_claim),     // 中断认领
        .irq_complete(plic_irq_complete) // 中断完成
    );

    // UART - 串口通信控制器
    uart_mmio #(
        .BASE_ADDR(32'h8100_1000),
        .CLK_FREQ(32'd100_000_000),     // 100MHz
        .DEFAULT_BAUD(`UART_DEFAULT_BAUD), // 默认波特率
        .FIFO_DEPTH(`UART_FIFO_DEPTH),  // FIFO深度
        .MIN_BAUD(1500)                 // 最小波特率
    ) uart_inst (
        .clk(clk),
        .resetn(resetn),

        // 内存总线接口
        .mem_valid(mem_valid & is_uart),
        .mem_instr(mem_instr),
        .mem_ready(uart_mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(uart_mem_rdata),

        // UART物理接口
        .uart_rx(uart_rx),      // 接收数据
        .uart_tx(uart_tx),      // 发送数据
        .uart_cts(uart_cts),    // 清除发送
        .uart_rts(uart_rts),    // 请求发送

        .irq(uart_irq),         // UART中断
        .eoi(eoi[1])            // 中断结束
    );

    // PWM - 脉冲宽度调制控制器（用于电机控制、LED调光等）
    pwm_mmio #(
        .BASE_ADDR(32'h8100_2000),
        .CLK_FREQ(32'd100_000_000),     // 100MHz
        .DEFAULT_FREQ(`PWM_DEFAULT_FREQ), // 默认PWM频率
        .MAX_CHANNELS(`PWM_MAX_CHANNELS)  // PWM通道数
    ) pwm_inst (
        .clk(clk),
        .resetn(resetn),

        // 内存总线接口
        .mem_valid(mem_valid & is_pwm),
        .mem_instr(mem_instr),
        .mem_ready(pwm_mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(pwm_mem_rdata),

        .pwm_out(pwm_out),      // PWM输出信号

        .irq(pwm_irq),          // PWM中断
        .eoi(eoi[2])            // 中断结束
    );

    // GPIO - 通用输入输出控制器
    gpio_mmio #(
        .GPIO_WIDTH(32),            // 32位GPIO
        .BASE_ADDR(32'h8100_4000)
    ) gpio_inst (
        .clk(clk),
        .resetn(resetn),

        // 内存总线接口
        .mem_valid(mem_valid & is_gpio),
        .mem_instr(mem_instr),
        .mem_ready(gpio_mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(gpio_mem_rdata),

        // GPIO物理接口
        .gpio_in(gpio_in),      // GPIO输入
        .gpio_out(gpio_out),    // GPIO输出
        .gpio_dir(gpio_dir),    // GPIO方向控制

        .irq(gpio_irq),         // GPIO中断（边沿检测等）
        .eoi(eoi[3])            // 中断结束
    );

    // MROM - 启动ROM（存储启动代码）
    mrom_mmio #(
        .BROM_ADDR(32'h2000_0000),       // ROM映射地址
        .BASE_ADDR(32'h8100_5000),       // 控制寄存器地址
        .ROM_SIZE(`BROM_DEFAULT_SIZE),   // ROM大小
        .INIT_FILE(`BROM_INIT_FILE),     // 初始化文件
        .READ_ONLY(1'b1)                 // 只读存储器
    ) mrom_inst (
        .clk(clk),
        .resetn(resetn),

        // 内存总线接口
        .mem_valid(mem_valid & is_mrom),
        .mem_instr(mem_instr),
        .mem_ready(mrom_mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(mrom_mem_rdata),

        .irq(mrom_irq),         // ROM访问中断（如访问错误）
        .eoi(eoi[4])            // 中断结束
    );

    // WDT - 看门狗定时器（系统监控和恢复）
    wdt_mmio #(
        .BASE_ADDR(32'h8100_6000),
        .CLK_FREQ(`CLK_FREQ_HZ),            // 系统时钟频率
        .DEFAULT_TIMEOUT(`WDT_DEFAULT_TIMEOUT), // 默认超时时间
        .MAX_TIMEOUT(`WDT_MAX_TIMEOUT)      // 最大超时时间
    ) wdt_inst (
        .clk(clk),
        .resetn(resetn),

        // 内存总线接口
        .mem_valid(mem_valid & is_wdt),
        .mem_instr(mem_instr),
        .mem_ready(wdt_mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(wdt_mem_rdata),

        .wdt_reset(wdt_reset),  // 看门狗复位输出
        .wdt_irq(wdt_irq),      // 看门狗中断（预警）
        .eoi(eoi[5])            // 中断结束
    );

    // Timer - 通用定时器
    timer_mmio #(
        .BASE_ADDR(32'h8100_7000),
        .CLK_FREQ(32'd100_000_000),         // 100MHz
        .DEFAULT_PRESCALER(`TIMER_DEFAULT_PRESCALER), // 默认预分频
        .MAX_COMPARE(`TIMER_MAX_COMPARE)    // 最大比较值
    ) timer_inst (
        .clk(clk),
        .resetn(resetn),

        // 内存总线接口
        .mem_valid(mem_valid & is_timer),
        .mem_instr(mem_instr),
        .mem_ready(timer_mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(timer_mem_rdata),

        .timer_irq(timer_irq),  // 定时器中断
        .eoi(eoi[6])            // 中断结束
    );

    // SRAM - 静态存储器控制器
    sram_mmio #(
        .BASE_ADDR(32'h8100_8000),
        .MEM_SIZE(`SRAM_DEFAULT_SIZE)   // SRAM大小
    ) sram_inst (
        .clk(clk),
        .resetn(resetn),

        // 内存总线接口
        .mem_valid(mem_valid & is_sram),
        .mem_instr(mem_instr),
        .mem_ready(sram_mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(sram_mem_rdata),

        .irq(sram_irq),         // SRAM访问中断
        .eoi(eoi[7])            // 中断结束
    );

    // RTC - 实时时钟（带日历功能）
    rtc_mmio #(
        .BASE_ADDR(32'h8100_9000),
        .CLK_FREQ(32'd100_000_000),     // 系统时钟
        .DEFAULT_YEAR(`RTC_DEFAULT_YEAR), // 默认年份
        .DEFAULT_TIME(`RTC_DEFAULT_TIME), // 默认时间
        .EXT_CLK_FREQ(32'd32768)        // 外部时钟频率（32.768kHz）
    ) rtc_inst (
        .clk(clk),
        .ext_clk(ext_clk),          // 外部时钟输入
        .use_ext_clk(use_ext_clk),  // 时钟源选择
        .resetn(resetn),

        // 内存总线接口
        .mem_valid(mem_valid & is_rtc),
        .mem_instr(mem_instr),
        .mem_ready(rtc_mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(rtc_mem_rdata),

        .irq(rtc_irq),          // RTC中断（闹钟等）
        .eoi(eoi[8])            // 中断结束
    );

    // 1-Wire - 单总线接口（用于温度传感器、EEPROM等）
    wire_mmio #(
        .BASE_ADDR(32'h8200_0000),
        .CLK_FREQ(32'd100_000_000),     // 100MHz
        .DEFAULT_SPEED(`WIRE_DEFAULT_SPEED), // 默认速度模式
        .FIFO_DEPTH(`WIRE_FIFO_DEPTH),  // FIFO深度
        .RESET_TIMEOUT(1000)            // 复位超时周期
    ) wire_inst (
        .clk(clk),
        .resetn(resetn),

        // 内存总线接口
        .mem_valid(mem_valid & is_wire),
        .mem_instr(mem_instr),
        .mem_ready(wire_mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(wire_mem_rdata),

        .wire_io(wire_io),      // 双向1-Wire数据线

        .irq(wire_irq),         // 1-Wire中断
        .eoi(eoi[9])            // 中断结束
    );

    // =========================================================================
    // 中断分配逻辑
    // 将各个外设的中断信号分配到CPU的32位中断输入总线上
    // =========================================================================

    assign irq = {
        20'b0,                    // 位31-12: 保留
        wire_irq,                 // 位11: 1-Wire中断
        rtc_irq,                  // 位10: RTC中断
        sram_irq,                 // 位9: SRAM中断
        timer_irq,                // 位8: 定时器中断
        wdt_irq,                  // 位7: 看门狗中断
        mrom_irq,                 // 位6: ROM中断
        gpio_irq,                 // 位5: GPIO中断
        pwm_irq,                  // 位4: PWM中断
        uart_irq,                 // 位3: UART中断
        clint_software_irq,       // 位2: CLINT软件中断
        clint_timer_irq           // 位1: CLINT定时器中断
        // 位0: 保留
    };

endmodule
