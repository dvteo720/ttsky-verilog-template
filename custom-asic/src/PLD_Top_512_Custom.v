`timescale 1ns / 1ps
//
// PLD_Top_512_Custom.v
// Verilog-2001
//
// Adaptare pentru ASIC custom (Cadence/SkyWater MPW, sau orice alt shuttle
// sky130) - NU pentru Tiny Tapeout. Fabric-ul intern (512 LE, 64 IOB,
// BRAM 256x16, 18176 biti de config) e IDENTIC cu PLD_Top_512_FINAL.v.
// Doar interfata de varf s-a schimbat, din 2 motive tehnice reale:
//
//   1) cfg_clk nu mai e ceas fizic separat de `clk`. Doua domenii de ceas
//      independente ar cere un SDC custom cu constrangeri multi-clock ca
//      sa fie analizate corect de STA - fezabil, dar nu-l pot verifica
//      fara sa rulez tot flow-ul. Solutia de mai jos e mai robusta: cfg_clk
//      devine semnal normal, sincronizat pe clk (2 flip-flop-uri +
//      detectie de front). Protocolul extern catre Arduino (D2/D3/D4/D5)
//      ramane IDENTIC - Arduino tot bit-banger-eaza exact ca inainte.
//
//   2) IO_Block nu mai foloseste `1'bz` intern (Yosys: "limited support
//      for tri-state logic", confirmat la sinteza reala). Fiecare din cele
//      64 paduri expune acum 3 semnale separate - out/oe/in - identic cu
//      cum trateaza orice flow ASIC un pin bidirecțional real. La nivelul
//      chip-ului fizic, celula de pad I/O din libraria sky130 e cea care
//      face tri-state-ul electric adevarat, pe baza semnalului oe.
//
// Restul (localparams, bitstream, mapare BRAM, bus global) - neschimbate.
// Vezi comentariile originale mai jos.
//
// Tranzistori (neschimbat):
//   Config SRAM  18176 biti x 6T  = 109,056T
//   LUT4           512      x 80T =  40,960T
//   DFF            512      x 30T =  15,360T
//   Bypass mux     512      x  6T =   3,072T
//   RoutingMux32  1536      x 40T =  61,440T
//   RoutingMux8    512      x 20T =  10,240T
//   IO_Block        64      x 50T =   3,200T
//   BRAM         4096 biti  x  6T =  24,576T
//   BRAM logica                   =     200T
//                         TOTAL  ~= 268,104T
//
// Bitstream (18176 biti, LSB-first):
//   [9215:0]      Routing config (512 LE x 18 biti)
//   [17919:9216]  LE config      (512 LE x 17 biti)
//   [18175:17920] IOB config     ( 64 IOB x  4 biti)
//
// BRAM mapping (intern, fara pini externi):
//   addr[7:0]  = le_outputs[7:0]
//   din[15:0]  = le_outputs[23:8]
//   we         = le_outputs[24]
//   dout[15:0] -> global_bus[15:0]  <- TOTI 16 biti!
//
// Global bus (32 linii):
//   [15:0]  = BRAM dout complet
//   [31:16] = le_outputs[0,32,64..480] (stride 32)
//
// WARNING: daca un LE e configurat combinational (bypass=1) si
// rutarea sa formeaza o bucla catre propria intrare, apare o
// bucla combinationala. Inregistreaza caile de feedback (bypass=0).
//
// ====================================================================

// --------------------------------------------------------------------
// LUT4
// --------------------------------------------------------------------
module LUT4 (
    input  wire [3:0]  in,
    input  wire [15:0] sram,
    output wire        out
);
    assign out = sram[in];
endmodule

// --------------------------------------------------------------------
// DFF
// --------------------------------------------------------------------
module DFF (
    input  wire clk,
    input  wire rst,
    input  wire en,
    input  wire d,
    output reg  q
);
    always @(posedge clk or posedge rst) begin
        if (rst)     q <= 1'b0;
        else if (en) q <= d;
    end
endmodule

// --------------------------------------------------------------------
// LogicElement: LUT4 + DFF + bypass mux
//   config_bits[15:0] = LUT truth table
//   config_bits[16]   = 1 combinational, 0 registered
// --------------------------------------------------------------------
module LogicElement (
    input  wire [3:0]  logic_in,
    input  wire        clk,
    input  wire        rst,
    input  wire [16:0] config_bits,
    output wire        le_out
);
    wire lut_out, dff_out;

    LUT4 lut (
        .in   (logic_in),
        .sram (config_bits[15:0]),
        .out  (lut_out)
    );

    DFF dff (
        .clk (clk),
        .rst (rst),
        .en  (1'b1),
        .d   (lut_out),
        .q   (dff_out)
    );

    // MODIFICAT pentru viteza maxima: le_out e ÎNTOTDEAUNA inregistrat.
    // config_bits[16] (fostul select de bypass) ramane in bitstream
    // (aceeasi lungime totala, 18176 biti, acelasi shift register) dar
    // e ignorat functional. Asta elimina ORICE cale combinationala
    // LE-la-LE prin crossbar: fiecare hop e acum garantat inregistrat,
    // deci path-ul cel mai lung intre doua DFF-uri e mereu exact
    // DFF -> RoutingMux -> LUT4 -> DFF (UN SINGUR hop), nu un lant de
    // pana la 512 LE-uri inlantuite combinational.
    assign le_out = dff_out;
endmodule

// --------------------------------------------------------------------
// RoutingMux32: 32:1
// --------------------------------------------------------------------
module RoutingMux32 (
    input  wire [31:0] routing_bus,
    input  wire [4:0]  config_sel,
    output wire        out
);
    assign out = routing_bus[config_sel];
endmodule

// --------------------------------------------------------------------
// RoutingMux8: 8:1
// --------------------------------------------------------------------
module RoutingMux8 (
    input  wire [7:0] routing_bus,
    input  wire [2:0] config_sel,
    output wire       out
);
    assign out = routing_bus[config_sel];
endmodule

// --------------------------------------------------------------------
// IO_Block: pad bidirectional - out/oe/in separate (fara 1'bz intern)
// --------------------------------------------------------------------
module IO_Block (
    input  wire        pad_in,
    output wire        pad_out,
    output wire        pad_oe,
    output wire        to_pld_core,
    input  wire [3:0]  out_src_bus,
    input  wire [1:0]  out_src_sel,
    input  wire        config_is_output,
    input  wire        config_oe
);
    wire from_core = out_src_bus[out_src_sel];

    assign pad_out     = from_core;
    assign pad_oe      = config_is_output && config_oe;
    assign to_pld_core = pad_in;
endmodule

// --------------------------------------------------------------------
// BRAM_256x16: memorie interna 256x16 biti
// --------------------------------------------------------------------
module BRAM_256x16 (
    input  wire        clk,
    input  wire [7:0]  addr,
    input  wire [15:0] din,
    input  wire        we,
    output reg  [15:0] dout
);
    reg [15:0] mem [0:255];

    integer k;
    initial begin
        for (k = 0; k < 256; k = k + 1)
            mem[k] = 16'd0;
    end

    always @(posedge clk) begin
        if (we)
            mem[addr] <= din;
        dout <= mem[addr];
    end
endmodule

// ====================================================================
// pld_jupiter_512_top - varianta ASIC custom (nu Tiny Tapeout)
// ====================================================================
module pld_jupiter_512_top (
    input  wire        clk,
    input  wire        rst,

    // Configurare seriala LSB-first (protocol extern neschimbat)
    // Arduino: cfg_clk=D2, cfg_data=D3, cfg_en=D4, cfg_done=D5
    input  wire        cfg_clk,
    input  wire        cfg_data,
    input  wire        cfg_en,
    output wire        cfg_done,

    // 64 paduri I/O, descompuse in out/oe/in
    input  wire [63:0] io_pad_in,
    output wire [63:0] io_pad_out,
    output wire [63:0] io_pad_oe
);

    // ------------------------------------------------------------------
    // Constante (identice cu originalul)
    // ------------------------------------------------------------------
    localparam integer NUM_LE     = 512;
    localparam integer NUM_IOB    = 64;
    localparam integer NUM_GLOBAL = 32; // 16 BRAM + 16 LE

    // 5+5+3+5 = 18 biti routing per LE
    localparam integer ROUTE_BITS_PER_LE = 18;
    localparam integer LE_CFG_BITS       = 17;
    localparam integer IOB_CFG_BITS      = 4;

    localparam integer ROUTE_BITS        = NUM_LE * ROUTE_BITS_PER_LE; // 9216
    localparam integer LE_BITS           = NUM_LE * LE_CFG_BITS;       // 8704
    localparam integer IOB_BITS          = NUM_IOB * IOB_CFG_BITS;     //  256
    localparam integer TOTAL_CONFIG_BITS = ROUTE_BITS + LE_BITS
                                         + IOB_BITS;                   // 18176

    // ------------------------------------------------------------------
    // Sincronizare + detectie front cfg_clk (vezi explicatia din header)
    // ------------------------------------------------------------------
    reg [2:0] cfg_clk_sync;
    reg [1:0] cfg_data_sync;
    reg [1:0] cfg_en_sync;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cfg_clk_sync  <= 3'b000;
            cfg_data_sync <= 2'b00;
            cfg_en_sync   <= 2'b00;
        end else begin
            cfg_clk_sync  <= {cfg_clk_sync[1:0],  cfg_clk};
            cfg_data_sync <= {cfg_data_sync[0],   cfg_data};
            cfg_en_sync   <= {cfg_en_sync[0],     cfg_en};
        end
    end

    wire cfg_clk_rise = cfg_clk_sync[1] && !cfg_clk_sync[2];
    wire cfg_data_s   = cfg_data_sync[1];
    wire cfg_en_s     = cfg_en_sync[1];

    // ------------------------------------------------------------------
    // Shift register configurare (LSB-first) - acum pe `clk`
    // 15 biti counter: max 32767 > 18176
    // ------------------------------------------------------------------
    reg [TOTAL_CONFIG_BITS-1:0] sram_cells;
    reg [14:0]                  cfg_bit_count;

    wire cfg_done_int = (cfg_bit_count >= 15'd18176);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sram_cells    <= {TOTAL_CONFIG_BITS{1'b0}};
            cfg_bit_count <= 15'd0;
        end else if (cfg_clk_rise && cfg_en_s && !cfg_done_int) begin
            sram_cells    <= {cfg_data_s, sram_cells[TOTAL_CONFIG_BITS-1:1]};
            cfg_bit_count <= cfg_bit_count + 15'd1;
        end
    end

    assign cfg_done = cfg_done_int;

    // ------------------------------------------------------------------
    // Busuri interne (identice cu originalul)
    // ------------------------------------------------------------------
    wire [NUM_LE-1:0]     le_outputs;
    wire [NUM_IOB-1:0]    iob_to_core;
    wire [NUM_GLOBAL-1:0] global_bus; // 32 linii

    // ------------------------------------------------------------------
    // BRAM complet intern
    //   LE[7:0]  -> addr
    //   LE[23:8] -> din
    //   LE[24]   -> we
    // ------------------------------------------------------------------
    wire [7:0]  internal_bram_addr = le_outputs[7:0];
    wire [15:0] internal_bram_din  = le_outputs[23:8];
    wire        internal_bram_we   = le_outputs[24];
    wire [15:0] internal_bram_dout;

    BRAM_256x16 bram_inst (
        .clk  (clk),
        .addr (internal_bram_addr),
        .din  (internal_bram_din),
        .we   (internal_bram_we),
        .dout (internal_bram_dout)
    );

    // ------------------------------------------------------------------
    // Global bus 32 linii:
    //   [15:0]  = BRAM dout COMPLET (toti 16 biti!)
    //   [31:16] = 16 LE-uri la stride 32
    // ------------------------------------------------------------------
    genvar g;
    generate
        // [15:0] -> toti 16 biti din BRAM
        for (g = 0; g < 16; g = g + 1) begin : GLOBAL_RAM
            assign global_bus[g] = internal_bram_dout[g];
        end
        // [31:16] -> LE-uri distribuite uniform (stride 32)
        for (g = 0; g < 16; g = g + 1) begin : GLOBAL_LE
            assign global_bus[g + 16] = le_outputs[g * 32];
        end
    endgenerate

    // Busuri dublate pentru ferestre wrap-around
    wire [2*NUM_LE-1:0]  le_bus_dbl  = {le_outputs,  le_outputs};
    wire [2*NUM_IOB-1:0] iob_bus_dbl = {iob_to_core, iob_to_core};

    genvar i, j;

    // ==================================================================
    // FABRIC LOGIC (512 Logic Elements) - identic cu originalul
    // ==================================================================
    generate
        for (i = 0; i < NUM_LE; i = i + 1) begin : LE_ARRAY

            localparam integer LE_WIN_A = (i * 7)            % NUM_LE;
            localparam integer LE_WIN_B = (i * 7 + NUM_LE/2) % NUM_LE;
            localparam integer IOB_WIN  = (i * 3)            % NUM_IOB;

            localparam integer RC = i * ROUTE_BITS_PER_LE;
            localparam integer LC = ROUTE_BITS + i * LE_CFG_BITS;

            wire [3:0] le_in;

            // in[0]: 32:1 din LE bus window A
            RoutingMux32 mux0 (
                .routing_bus (le_bus_dbl[LE_WIN_A +: 32]),
                .config_sel  (sram_cells[RC +: 5]),
                .out         (le_in[0])
            );

            // in[1]: 32:1 din LE bus window B
            RoutingMux32 mux1 (
                .routing_bus (le_bus_dbl[LE_WIN_B +: 32]),
                .config_sel  (sram_cells[RC+5 +: 5]),
                .out         (le_in[1])
            );

            // in[2]: 8:1 din IOB bus
            RoutingMux8 mux2 (
                .routing_bus (iob_bus_dbl[IOB_WIN +: 8]),
                .config_sel  (sram_cells[RC+10 +: 3]),
                .out         (le_in[2])
            );

            // in[3]: 32:1 din global bus (BRAM[15:0] + 16 LE signals)
            RoutingMux32 mux3 (
                .routing_bus (global_bus),
                .config_sel  (sram_cells[RC+13 +: 5]),
                .out         (le_in[3])
            );

            LogicElement le (
                .logic_in   (le_in),
                .clk        (clk),
                .rst        (rst),
                .config_bits(sram_cells[LC +: 17]),
                .le_out     (le_outputs[i])
            );

        end
    endgenerate

    // ==================================================================
    // I/O RING (64 paduri bidirecționale - out/oe/in separate)
    // ==================================================================
    generate
        for (j = 0; j < NUM_IOB; j = j + 1) begin : IOB_ARRAY

            localparam integer IC      = ROUTE_BITS + LE_BITS
                                        + j * IOB_CFG_BITS;
            localparam integer IOB_SRC = (j * 8) % NUM_LE;

            IO_Block iob (
                .pad_in          (io_pad_in[j]),
                .pad_out         (io_pad_out[j]),
                .pad_oe          (io_pad_oe[j]),
                .to_pld_core     (iob_to_core[j]),
                .out_src_bus     (le_bus_dbl[IOB_SRC +: 4]),
                .out_src_sel     (sram_cells[IC+2 +: 2]),
                .config_is_output(sram_cells[IC]),
                .config_oe       (sram_cells[IC+1])
            );

        end
    endgenerate

endmodule
