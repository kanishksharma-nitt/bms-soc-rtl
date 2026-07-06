// Self-checking testbench for the BMS SoC estimation engine.
//
// Replays five drive cycles recorded by python/bms_model.py (constant
// discharge, pulse discharge, charge with rests, current-sensor offset,
// fault injection) and compares SoC and all plausibility flags bit-exactly
// on every 100 Hz sample. The DUT is reset between scenarios.
`timescale 1ns / 1ps
`default_nettype none

module tb_soc;

  localparam CLK_PERIOD = 10;
  localparam CLKS_PER_SAMPLE = 64;   // > inverse-lookup latency (~31)

  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg sample_stb = 1'b0;
  reg signed [15:0] iq = 16'sd0, tq = 16'sd0;
  reg [15:0] vq = 16'd0;

  wire [15:0] soc, ocv_dbg;
  wire [3:0] flags;

  integer errors = 0;
  integer nscen, sc, nsamp, i, base, eidx, w;
  reg [31:0] vec [0:131071];
  reg [31:0] expw;

  soc_top dut (
      .clk(clk), .rst_n(rst_n), .sample_stb(sample_stb),
      .iq(iq), .vq(vq), .tq(tq),
      .soc(soc), .flags(flags), .ocv_dbg(ocv_dbg));

  always #(CLK_PERIOD / 2) clk = ~clk;

  initial begin
    if ($test$plusargs("vcd")) begin
      $dumpfile("tb_soc.vcd");
      $dumpvars(0, tb_soc);
    end

    $readmemh("../test/bms_vectors.mem", vec);
    nscen = vec[0];
    base = 1;

    for (sc = 0; sc < nscen; sc = sc + 1) begin
      nsamp = vec[base];

      rst_n = 1'b0;
      sample_stb = 1'b0;
      repeat (3) @(negedge clk);
      rst_n = 1'b1;
      @(negedge clk);

      for (i = 0; i < nsamp; i = i + 1) begin
        iq = vec[base + 1 + 3*i][15:0];
        vq = vec[base + 2 + 3*i][15:0];
        tq = vec[base + 3 + 3*i][15:0];
        sample_stb = 1'b1;
        @(negedge clk);
        sample_stb = 1'b0;

        eidx = base + 1 + 3*nsamp + i;
        expw = vec[eidx];
        if ({flags, soc} !== expw[19:0]) begin
          errors = errors + 1;
          if (errors < 20)
            $display("FAIL scen %0d sample %0d: flags/soc %b/%0d expected %b/%0d",
                     sc, i, flags, soc, expw[19:16], expw[15:0]);
        end

        for (w = 0; w < CLKS_PER_SAMPLE - 1; w = w + 1) @(negedge clk);
      end

      $display("scenario %0d: %0d samples, %0d total errors",
               sc, nsamp, errors);
      base = base + 1 + 4 * nsamp;
    end

    if (errors == 0) $display("TEST PASSED");
    else             $display("TEST FAILED: %0d errors", errors);
    $finish;
  end

  initial begin
    #40_000_000;
    $display("TEST FAILED: global timeout");
    $finish;
  end

endmodule

`default_nettype wire
