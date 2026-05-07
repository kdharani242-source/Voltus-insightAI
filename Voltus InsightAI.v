// Code your design here
//=========================================================
//=========================================================
// AI FSM POWER INTEGRITY DESIGN
module ai_fsm_power_integrity (

    input              clk,
    input              rst,
    input              enable,
    input              sensor,

    output reg [1:0]   state,
    output reg         alarm,
    output reg         process_done

);

//=========================================================
// STATE ENCODING
//=========================================================

parameter IDLE    = 2'b00;
parameter MONITOR = 2'b01;
parameter PROCESS = 2'b10;
parameter ALERT   = 2'b11;

//=========================================================
// OPERAND ISOLATION
//=========================================================

wire sensor_iso;

assign sensor_iso = (enable) ? sensor : 1'b0;

//=========================================================
// FSM SEQUENTIAL LOGIC
//=========================================================

always @(posedge clk or posedge rst) begin

    if(rst)
        state <= IDLE;

    else begin

        case(state)

            IDLE: begin

                if(sensor_iso)
                    state <= MONITOR;
                else
                    state <= IDLE;

            end

            MONITOR: begin

                if(sensor_iso)
                    state <= PROCESS;
                else
                    state <= IDLE;

            end

            PROCESS: begin

                if(sensor_iso)
                    state <= ALERT;
                else
                    state <= IDLE;

            end

            ALERT: begin

                state <= IDLE;

            end

            default: begin

                state <= IDLE;

            end

        endcase

    end

end

//=========================================================
// OUTPUT LOGIC
//=========================================================

always @(*) begin

    // Default outputs
    alarm        = 1'b0;
    process_done = 1'b0;

    case(state)

        IDLE: begin

            alarm        = 1'b0;
            process_done = 1'b0;

        end

        MONITOR: begin

            alarm        = 1'b0;
            process_done = 1'b0;

        end

        PROCESS: begin

            alarm        = 1'b0;
            process_done = 1'b1;

        end

        ALERT: begin

            alarm        = 1'b1;
            process_done = 1'b1;

        end

        default: begin

            alarm        = 1'b0;
            process_done = 1'b0;

        end

    endcase

end

endmodule

// Code your testbench here
// or browse Examples
`timescale 1ns/1ps

module tb_ai_fsm_power_integrity;

//=========================================================
// INPUTS
//=========================================================

reg clk;
reg rst;
reg enable;
reg sensor;

//=========================================================
// OUTPUTS
//=========================================================

wire [1:0] state;
wire alarm;
wire process_done;

//=========================================================
// ANALYSIS VARIABLES
//=========================================================

integer toggle_count;
integer state_transition_count;

reg [1:0] prev_state;
reg prev_alarm;
reg prev_process_done;

real current_estimation;
real resistance;
real ir_drop;
real final_voltage;
real dynamic_power;
real em_current_density;

real efficiency;
real performance_index;
real area_estimation;
real thermal_index;
real delay_estimation;
real leakage_power;

//=========================================================
// DUT
//=========================================================

ai_fsm_power_integrity uut (

    .clk(clk),
    .rst(rst),
    .enable(enable),
    .sensor(sensor),
    .state(state),
    .alarm(alarm),
    .process_done(process_done)

);

//=========================================================
// CLOCK GENERATION
//=========================================================

always #5 clk = ~clk;

//=========================================================
// TOGGLE COUNTER
//=========================================================

always @(posedge clk) begin

    //-----------------------------------------------------
    // Ignore reset condition
    //-----------------------------------------------------

    if(!rst) begin

        //-------------------------------------------------
        // Count alarm toggle
        //-------------------------------------------------

        if(alarm != prev_alarm)
            toggle_count = toggle_count + 1;

        //-------------------------------------------------
        // Count process_done toggle
        //-------------------------------------------------

        if(process_done != prev_process_done)
            toggle_count = toggle_count + 1;

        //-------------------------------------------------
        // Count state transition
        //-------------------------------------------------

        if(state != prev_state)
            state_transition_count =
            state_transition_count + 1;

    end

    //-----------------------------------------------------
    // Store previous values
    //-----------------------------------------------------

    prev_state        <= state;
    prev_alarm        <= alarm;
    prev_process_done <= process_done;

end

//=========================================================
// SIMULATION
//=========================================================

initial begin

    //-----------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------

    clk = 0;
    rst = 1;
    enable = 0;
    sensor = 0;

    toggle_count = 0;
    state_transition_count = 0;

    prev_state = 2'b00;
    prev_alarm = 1'b0;
    prev_process_done = 1'b0;

    resistance = 0.5;

    //-----------------------------------------------------
    // WAVEFORM
    //-----------------------------------------------------

    $dumpfile("fsm_wave.vcd");
    $dumpvars(0, tb_ai_fsm_power_integrity);

    //-----------------------------------------------------
    // HEADER
    //-----------------------------------------------------

    $display("====================================================");
    $display(" VOLTUS INSIGHTAI ADVANCED ANALYSIS ");
    $display("====================================================");

    //-----------------------------------------------------
    // RESET
    //-----------------------------------------------------

    #10;
    rst = 0;

    //-----------------------------------------------------
    // LOW ACTIVITY
    //-----------------------------------------------------

    enable = 1;
    sensor = 0;

    #20;

    //-----------------------------------------------------
    // MEDIUM ACTIVITY
    //-----------------------------------------------------

    sensor = 1;

    #20;

    //-----------------------------------------------------
    // HIGH ACTIVITY
    //-----------------------------------------------------

    sensor = 1;

    #20;

    //-----------------------------------------------------
    // VERY HIGH ACTIVITY
    //-----------------------------------------------------

    sensor = 1;

    #20;

    //-----------------------------------------------------
    // STRESS CONDITION
    //-----------------------------------------------------

    sensor = 1;

    #20;

    //-----------------------------------------------------
    // POWER ANALYSIS
    //-----------------------------------------------------

    current_estimation =
        (toggle_count + state_transition_count) * 0.01;

    dynamic_power =
        (toggle_count * 1.0) +
        (state_transition_count * 1.5);

    leakage_power =
        0.05 * dynamic_power;

    ir_drop =
        current_estimation * resistance;

    final_voltage =
        1.0 - ir_drop;

    em_current_density =
        current_estimation / 0.1;

    //-----------------------------------------------------
    // PERFORMANCE ANALYSIS
    //-----------------------------------------------------

    delay_estimation =
        1.0 + (state_transition_count * 0.05);

    performance_index =
        100 / delay_estimation;

    //-----------------------------------------------------
    // AREA ESTIMATION
    //-----------------------------------------------------

    area_estimation = 45.0;

    //-----------------------------------------------------
    // THERMAL INDEX
    //-----------------------------------------------------

    thermal_index =
        dynamic_power * 0.8;

    //-----------------------------------------------------
    // EFFICIENCY
    //-----------------------------------------------------

    if(dynamic_power != 0)
        efficiency =
        performance_index / dynamic_power;
    else
        efficiency = 0;

    //-----------------------------------------------------
    // REPORT
    //-----------------------------------------------------

    $display("----------------------------------------------------");

    $display("Total Toggle Count          = %0d",
              toggle_count);

    $display("Estimated Current           = %0f A",
              current_estimation);

    $display("Estimated Dynamic Power     = %0f uW",
              dynamic_power);

    $display("Estimated Leakage Power     = %0f uW",
              leakage_power);

    $display("Estimated IR Drop           = %0f V",
              ir_drop);

    $display("Final Cell Voltage          = %0f V",
              final_voltage);

    $display("EM Current Density          = %0f A/um",
              em_current_density);

    $display("Estimated Delay             = %0f ns",
              delay_estimation);

    $display("Performance Index           = %0f",
              performance_index);

    $display("Estimated Area              = %0f um^2",
              area_estimation);

    $display("Thermal Index               = %0f",
              thermal_index);

    $display("Efficiency                  = %0f",
              efficiency);

    $display("----------------------------------------------------");

    //-----------------------------------------------------
    // AI HOTSPOT DETECTION
    //-----------------------------------------------------

    if(dynamic_power > 12)
        $display("AI Hotspot Detection : CRITICAL HOTSPOT");

    else if(dynamic_power > 8)
        $display("AI Hotspot Detection : MODERATE HOTSPOT");

    else
        $display("AI Hotspot Detection : SAFE REGION");

    //-----------------------------------------------------
    // IR ANALYSIS
    //-----------------------------------------------------

    if(ir_drop > 0.1)
        $display("IR Analysis : CRITICAL IR DROP");

    else if(ir_drop > 0.05)
        $display("IR Analysis : MODERATE IR DROP");

    else
        $display("IR Analysis : SAFE IR DROP");

    //-----------------------------------------------------
    // EM ANALYSIS
    //-----------------------------------------------------

    if(em_current_density > 1.0)
        $display("EM Analysis : ELECTROMIGRATION RISK");

    else
        $display("EM Analysis : EM SAFE");

    //-----------------------------------------------------
    // THERMAL ANALYSIS
    //-----------------------------------------------------

    if(thermal_index > 10)
        $display("Thermal Analysis : HIGH TEMPERATURE REGION");

    else
        $display("Thermal Analysis : THERMALLY SAFE");

    //-----------------------------------------------------
    // TECHNOLOGY RECOMMENDATIONS
    //-----------------------------------------------------

    $display("----------------------------------------------------");

    $display("Best Suitable Technology Recommendations:");

    $display("1. 5nm FinFET  -> High Performance AI Chips");
    $display("2. 7nm FinFET  -> Balanced PPA Optimization");
    $display("3. 12nm FinFET -> Moderate Power Designs");
    $display("4. 28nm CMOS   -> Low Cost FPGA/ASIC Designs");

    $display("----------------------------------------------------");

    //-----------------------------------------------------
    // AI OPTIMIZATION SUGGESTIONS
    //-----------------------------------------------------

    $display("AI Optimization Suggestions:");

    $display("1. Operand Isolation");
    $display("2. Clock Gating");
    $display("3. Reduce Switching Activity");
    $display("4. Widen Power Rails");
    $display("5. Insert Decoupling Capacitors");
    $display("6. Spread High Activity Cells");
    $display("7. Multi-Vt Cell Optimization");

    $display("----------------------------------------------------");

    $finish;

end

endmodule



//OUTPUT
# KERNEL: ====================================================
# KERNEL:  VOLTUS INSIGHTAI ADVANCED ANALYSIS 
# KERNEL: ====================================================
# KERNEL: ----------------------------------------------------
# KERNEL: Total Toggle Count          = 6
# KERNEL: Estimated Current           = 0.130000 A
# KERNEL: Estimated Dynamic Power     = 16.500000 uW
# KERNEL: Estimated Leakage Power     = 0.825000 uW
# KERNEL: Estimated IR Drop           = 0.065000 V
# KERNEL: Final Cell Voltage          = 0.935000 V
# KERNEL: EM Current Density          = 1.300000 A/um
# KERNEL: Estimated Delay             = 1.350000 ns
# KERNEL: Performance Index           = 74.074074
# KERNEL: Estimated Area              = 45.000000 um^2
# KERNEL: Thermal Index               = 13.200000
# KERNEL: Efficiency                  = 4.489338
# KERNEL: ----------------------------------------------------
# KERNEL: AI Hotspot Detection : CRITICAL HOTSPOT
# KERNEL: IR Analysis : MODERATE IR DROP
# KERNEL: EM Analysis : ELECTROMIGRATION RISK
# KERNEL: Thermal Analysis : HIGH TEMPERATURE REGION
# KERNEL: ----------------------------------------------------
# KERNEL: Best Suitable Technology Recommendations:
# KERNEL: 1. 5nm FinFET  -> High Performance AI Chips
# KERNEL: 2. 7nm FinFET  -> Balanced PPA Optimization
# KERNEL: 3. 12nm FinFET -> Moderate Power Designs
# KERNEL: 4. 28nm CMOS   -> Low Cost FPGA/ASIC Designs
# KERNEL: ----------------------------------------------------
# KERNEL: AI Optimization Suggestions:
# KERNEL: 1. Operand Isolation
# KERNEL: 2. Clock Gating
# KERNEL: 3. Reduce Switching Activity
# KERNEL: 4. Widen Power Rails
# KERNEL: 5. Insert Decoupling Capacitors
# KERNEL: 6. Spread High Activity Cells
# KERNEL: 7. Multi-Vt Cell Optimization
# KERNEL: ----------------------------------------------------
