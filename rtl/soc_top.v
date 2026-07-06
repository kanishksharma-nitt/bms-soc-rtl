// BMS state-of-charge top: Coulomb counting + rest-time OCV correction with
// plausibility gating. Per sample strobe, one combinational chain:
//   sanity flags -> rest detect -> Coulomb update -> OCV blend (when rested,
//   valid and not frozen). The inverse OCV lookup for this sample's voltage
//   runs as a multi-cycle FSM (~31 cycles), and its result feeds the next
//   sample's blend.
`default_nettype none

module soc_top #(
    parameter LUT_FILE = "../rtl/ocv_lut.mem"
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_stb,
    input  wire signed [15:0] iq,        // pack current, Q7.9, +ve discharge
    input  wire [15:0]        vq,        // cell voltage, Q3.13
    input  wire signed [15:0] tq,        // temperature, Q7.1
    output wire [15:0]        soc,       // Q1.15 (1.0 = 32768)
    output reg  [3:0]         flags,     // {rested, i_stuck, t_oor, v_oor}
    output wire [15:0]        ocv_dbg    // forward OCV(soc), debug
);

  wire v_oor, t_oor, i_stuck, freeze_corr, rested;

  sanity u_sanity (
      .clk(clk), .rst_n(rst_n), .en(sample_stb),
      .iq(iq), .vq(vq), .tq(tq),
      .v_oor(v_oor), .t_oor(t_oor), .i_stuck(i_stuck),
      .freeze_corr(freeze_corr));

  rest_det u_rest (
      .clk(clk), .rst_n(rst_n), .en(sample_stb),
      .iq(iq), .rested(rested));

  // inverse-lookup result from the previous sample
  reg  [31:0] soc_ocv_q;
  reg         ocv_valid;
  wire        inv_done;
  wire [31:0] inv_soc;

  wire [31:0] soc_q31;
  coulomb u_coulomb (
      .clk(clk), .rst_n(rst_n), .en(sample_stb),
      .iq(iq),
      .corr_en(rested && ocv_valid && !freeze_corr),
      .soc_ocv(soc_ocv_q),
      .soc(soc_q31));

  wire ocv_busy;
  ocv_lut #(.LUT_FILE(LUT_FILE)) u_ocv (
      .clk(clk), .rst_n(rst_n),
      .soc(soc_q31), .ocv(ocv_dbg),
      .start(sample_stb), .v_in(vq),
      .done(inv_done), .soc_ocv(inv_soc), .busy(ocv_busy));

  wire _unused = &{1'b0, ocv_busy};

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      soc_ocv_q <= 32'd0;
      ocv_valid <= 1'b0;
      flags     <= 4'd0;
    end else begin
      if (inv_done) begin
        soc_ocv_q <= inv_soc;
        ocv_valid <= 1'b1;
      end
      if (sample_stb)
        flags <= {rested, i_stuck, t_oor, v_oor};
    end
  end

  assign soc = soc_q31[31:16];

endmodule

`default_nettype wire
