/*  This file is part of JT89.

    JT89 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    JT89 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with JT89.  If not, see <http://www.gnu.org/licenses/>.

    Author: Jose Tejada Gomez. Twitter: @topapate
    Version: 1.0
    Date: December, 1st 2018
   
    */

module jt89_mixer #(parameter bw=9, interpol16=0)(
    input            rst,
    input            clk,
    input            clk_en,
    input            cen_16,
    input            cen_4,
    input     [bw-1:0] ch0,
    input     [bw-1:0] ch1,
    input     [bw-1:0] ch2,
    input     [bw-1:0] noise,
    output signed [bw+1:0] sound
);

reg signed [bw+1:0] fresh;

always @(posedge clk)
    fresh <= 
        { {2{ch0[bw-1]}}, ch0   }+
        { {2{ch1[bw-1]}}, ch1   }+
        { {2{ch2[bw-1]}}, ch2   }+
        { {2{noise[bw-1]}}, noise };

generate
    if( interpol16==1 ) begin
        wire signed [bw+1:0] snd4;
        localparam calcw=bw+8;
        jt12_interpol #(.calcw(calcw),.inw(bw+2),.rate(4),.m(4),.n(2)) u_uprate1 (
            .rst    ( rst    ),
            .clk    ( clk    ),
            .cen_in ( cen_16 ),
            .cen_out( cen_4  ),
            .snd_in ( fresh  ),
            .snd_out( snd4   )
        );
        jt12_interpol #(.calcw(calcw),.inw(bw+2),.rate(4),.m(4),.n(2)) u_uprate2 (
            .rst    ( rst    ),
            .clk    ( clk    ),
            .cen_in ( cen_4  ),
            .cen_out( clk_en ),
            .snd_in ( snd4   ),
            .snd_out( sound  )
        );        
    end else
        assign sound = fresh;
endgenerate

endmodule