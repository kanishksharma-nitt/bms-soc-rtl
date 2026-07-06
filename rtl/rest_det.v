// Rest detector: rested when |I| stays below REST_TH for REST_N samples.
// The `rested` output is the post-update value for the current sample
// (combinational off the counter's next state), matching the golden model.
`default_nettype none

module rest_det #(
    parameter signed [15:0] REST_TH = 16'sd154,   // 0.3 A in Q7.9
    parameter [8:0]         REST_N  = 9'd300      // 3 s at 100 Hz
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               en,
    input  wire signed [15:0] iq,
    output wire               rested
);

  reg [8:0] cnt;

  wire signed [16:0] iq17   = {iq[15], iq};
  wire signed [16:0] a_iq   = iq17[16] ? -iq17 : iq17;
  wire               below  = (a_iq < {REST_TH[15], REST_TH});
  wire [8:0]         cnt_nx = !below           ? 9'd0 :
                              (cnt == REST_N)  ? REST_N : cnt + 9'd1;

  assign rested = (cnt_nx >= REST_N);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)  cnt <= 9'd0;
    else if (en) cnt <= cnt_nx;
  end

endmodule

`default_nettype wire
