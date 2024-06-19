`timescale 1ps/1ps
//*********************************************//
// CREATED: 2024.06.17                         //
// TITLE : DIGITAL Hardware                    //
//*********************************************//

module main (
            input clk,
            input [0:0] sw,
            input PS2Clk,
            input PS2Data,

            output [3:0] an,
            output [6:0] seg,
            output [7:0] led,

            // VGA
            output [11:0] display_rgb,
            output HSYNC,
            output VSYNC
);

    wire end_game_flag;
    wire end_game_flag_endToship;
    wire end_game_flag_endToRock;
    wire end_game_flag_endToDisplay;

    wire game_succeed_flag;

    wire reset_game_flag_shipToComp;
    wire reset_game_flag_rockToComp;

    wire [31:0] ship_position_display;
    wire [31:0] ship_position_comp;

    wire [31:0] rock_position_display;
    wire [31:0] rock_position_comp;

    wire clk_25Mhz;

    wire rock_position_changed_flag;
    wire ship_position_changed_flag;
    
    wire [1:0] btn_keyboard;

    //**********************************************//
    //                  GAME                        //
    //**********************************************//

    ship_position genergate_ship_position (
                                        .clk(clk),
                                        .btnD(btn_keyboard[1]),
                                        .btnU(btn_keyboard[0]),
                                        .sw(sw),
                                        .end_game_in(end_game_flag_endToship),

                                        .reset_compute_flag_out(reset_game_flag_shipToComp),
                                        .ship_position_out_display(ship_position_display),
                                        .ship_position_out_comp(ship_position_comp),

                                        .ship_position_changed_flag_out(ship_position_changed_flag)
    );

    rock_position generate_rock_position (
                                        .clk(clk),
                                        .sw(sw),
                                        .end_game_in(end_game_flag_endToRock),

                                        .reset_compute_flag_out(reset_game_flag_rockToComp),
                                        .rock_position_out_comp(rock_position_comp),
                                        .rock_position_out_display(rock_position_display),

                                        .rock_position_changed_flag_out(rock_position_changed_flag)
    );

    compute generate_compute (
                            .clk(clk),
                            .sw(sw),
                            .ship_position_in(ship_position_comp),
                            .rock_position_in(rock_position_comp),
                            .reset_compute_in_ship(reset_game_flag_shipToComp),
                            .reset_compute_in_rock(reset_game_flag_rockToComp),

                            .game_succeed_flag_out(game_succeed_flag),
                            .end_game_flag_out(end_game_flag),
                            .end_game_flag_out_ship(end_game_flag_endToship),
                            .end_game_flag_out_rock(end_game_flag_endToRock)
    );

    end_game generate_end_game(
                            .clk(clk),
                            .sw(sw),
                            .end_game_flag_in(end_game_flag),

                            .reset_flag_out_ship(end_game_flag_endToship),
                            .reset_flag_out_rock(end_game_flag_endToRock),
                            .reset_flag_out_display(end_game_flag_endToDisplay)
    );

    
    //*********************************************//
    //              Peripheral                     //
    //*********************************************//
 

    Keyboard get_keyboard_data(
                               .clk(clk),
                               .sw(sw),
                               .PS2Clk(PS2Clk),
                               .PS2Data(PS2Data),

                               .led(led),
                               .btn_out(btn_keyboard)
    );

    wire [10:0] display_x;
    wire [10:0] display_y;

    wire video_on_flag;

    wire [11:0] graphic_rgb;

    vga_controller generate_vga(
                                .mclk(clk),
                                .sw(sw),
                                
                                .HSYNC_out(HSYNC),
                                .VSYNC_out(VSYNC),
                                .Red_out(),
                                .Green_out(),
                                .Blue_out(),
                                .clk_25Mhz(clk_25Mhz),

                                .current_display_x_coordinate_out(display_x),
                                .current_display_y_coordinate_out(display_y),

                                .video_on_flag_out(video_on_flag)
    );

    vga_graphic_interface generate_graphic_interface (
                                                      .clk(clk),
                                                      .btnU(btn_keyboard[1]),
                                                      .btnD(btn_keyboard[0]),
                                                      .sw(sw),

                                                      .current_display_x_coordinate_in(display_x),
                                                      .current_display_y_coordinate_in(display_y),
                                                      .video_on_flag_in(video_on_flag),

                                                      .ship_position_in(ship_position_display),
                                                      .rock_position_in(rock_position_display),
                                                      .ship_position_changed_flag_in(ship_position_changed_flag),
                                                      .rock_position_changed_flag_in(rock_position_changed_flag),

                                                      .graphic_rgb_out(graphic_rgb)
    ); 

    //*******************************************//
    //          GENERATE VIDEO                   //
    //*******************************************//

    generate_video DO (
                       .clk(clk),
                       .sw(sw),
                       .video_data(graphic_rgb),
                       .clk_25Mhz(clk_25Mhz),

                       .rgb_out(display_rgb)
    );


endmodule


module end_game (
                input clk,
                input [0:0] sw,
                input wire end_game_flag_in,

                output reg reset_flag_out_ship,
                output reg reset_flag_out_rock,
                output reg reset_flag_out_display 
);
    always @(clk) begin
        if(~sw) begin
                reset_flag_out_ship <= 1'b0;
                reset_flag_out_rock <= 1'b0;
                reset_flag_out_display <= 1'b0;
        end else begin
            if(end_game_flag_in) begin
                reset_flag_out_ship <= 1'b1;
                reset_flag_out_rock <= 1'b1;
                reset_flag_out_display <= 1'b1;
            end else begin
                reset_flag_out_ship <= 1'b0;
                reset_flag_out_rock <= 1'b0;
                reset_flag_out_display <= 1'b1;
            end
        end
    end
endmodule

module ship_position (
                    input wire clk,
                    input wire btnD,
                    input wire btnU,
                    input wire [0:0] sw,
                    input wire [31:0] end_game_in,

                    output reg reset_compute_flag_out,
                    output reg [31:0] ship_position_out_display,
                    output reg [31:0] ship_position_out_comp,

                    output reg ship_position_changed_flag_out
);

    reg [31:0] ship_position;
    
    //******************************************//
    //           CREATE 0.5ms pulse             //
    //******************************************//

    reg [31:0] clk_counter;
    reg clk_2ms;

    always @(posedge clk) begin
        if (~sw) begin
            clk_counter <= 1'b0;
            clk_2ms <= 1'b0;
        end else begin
            if (clk_counter == 200 - 1) begin
                clk_counter <= 1'b0;
                clk_2ms <= ~clk_2ms;
            end else begin
                clk_counter <= clk_counter + 1'b1;
            end
        end
    end


    //******************************************//
    //          CREATE ship position            //
    //******************************************//


    always@ (posedge clk_2ms) begin
        if(~sw ) begin
            ship_position <= 1'b0;
            reset_compute_flag_out <= 1'b1;
            ship_position_changed_flag_out <= 1'b0;
        end else begin
            reset_compute_flag_out <= 1'b0;
            ship_position_changed_flag_out <= 1'b0;

            if(btnD) begin
                if(ship_position < 5) begin
                    ship_position <= ship_position + 32'd4;
                    ship_position_changed_flag_out <= 1'b1;
                end else begin
                    ship_position <= ship_position;
                    ship_position_changed_flag_out <= 1'b0;
                end
            end else if(btnU) begin
                if(ship_position > 3) begin
                    ship_position <= ship_position - 32'd4;
                    ship_position_changed_flag_out <= 1'b1;
                end else begin
                    ship_position <= ship_position;
                    ship_position_changed_flag_out <= 1'b0;
                end
            end
        end
    end

    always @(ship_position) begin
        ship_position_out_comp <= ship_position;
        ship_position_out_display <= ship_position;
    end
endmodule

module compute (
                input wire clk,
                input wire [0:0] sw,
                input wire [31:0] ship_position_in,
                input wire [31:0] rock_position_in,
                input wire reset_compute_in_ship,
                input wire reset_compute_in_rock,
                
                output reg game_succeed_flag_out,
                output reg end_game_flag_out,
                output reg end_game_flag_out_ship,
                output reg end_game_flag_out_rock
);

    always @(posedge clk) begin
        if(~sw | reset_compute_in_ship | reset_compute_in_rock) begin
            end_game_flag_out <= 1'b0;
            end_game_flag_out_ship <= 1'b0;
            end_game_flag_out_rock <= 1'b0;
            game_succeed_flag_out <= 1'b0;
        end else begin
            if(ship_position_in == rock_position_in) begin
                // game failed
                end_game_flag_out <= 1'b1;
                end_game_flag_out_ship <= 1'b1;
                end_game_flag_out_rock <=1'b1;
                game_succeed_flag_out <= 1'b0;
            end else if (rock_position_in == 32'd0 | rock_position_in == 32'd4 | rock_position_in | 32'd8) begin
                // game succeed
                game_succeed_flag_out <= 1'b1;
                end_game_flag_out <= 1'b0;
                end_game_flag_out_ship <= 1'b0;
                end_game_flag_out_rock <=1'b0;
            end else begin
                // game in progress
                game_succeed_flag_out <= 1'b0;
                end_game_flag_out <= 1'b0;
                end_game_flag_out_ship <= 1'b0;
                end_game_flag_out_rock <=1'b0;
            end
        end        
    end
endmodule

module rock_position (
                    input wire clk,
                    input wire [0:0] sw,
                    input wire [31:0] end_game_in,
                    
                    output reg reset_compute_flag_out,
                    output reg rock_position_changed_flag_out,
                    output reg [31:0] rock_position_out_display,
                    output reg [31:0] rock_position_out_comp
);

    reg clk_1s;
    reg [31:0] clk_counter;

    reg [31:0] rock_position;
    reg [31:0] rock_n_position;

    reg rock_chage_flag;

    //**********************************************//
    //          CREATE 1S CLK                       //
    //**********************************************//

    always@ (posedge clk) begin
        if(~sw | end_game_in) begin
            clk_1s <= 1'b0;
        end else begin  
            if(clk_counter < 32'd100000000) begin
                clk_counter <= clk_counter  + 1'b1;
            end
            else begin
                clk_1s <= ~clk_1s;
                clk_counter <= 1'b0;
            end
        end
    end

    //************************************************//
    //              CREAT ROCK POSITION               //
    //************************************************//
    reg change;
    reg [31:0] clk_count;

    always @(posedge clk) begin
        if(~sw | end_game_in) begin
            rock_position <= 1'b0;
            rock_n_position <= 32'd3;

            // reset_compute_flag_out <= 1'b1;
            // rock_position_changed_flag_out <= 1'b0;
        end else begin
        // reset_compute_flag_out <= 1'b0;
        // rock_position_changed_flag_out <= 1'b0;

            rock_position <= rock_n_position;
        end
    end

    always @(clk) begin
        if(change == 1 ) begin
            case(rock_position)
                32'd0: rock_n_position <= 32'd7;
                32'd1: rock_n_position <= 32'd0;
                32'd2: rock_n_position <= 32'd1;
                32'd3: rock_n_position <= 32'd2;
                32'd4: rock_n_position <= 32'd11;
                32'd5: rock_n_position <= 32'd4;
                32'd6: rock_n_position <= 32'd5;
                32'd7: rock_n_position <= 32'd6;
                32'd8: rock_n_position <= 32'd3;
                32'd9: rock_n_position <= 32'd8;
                32'd10: rock_n_position <= 32'd9;
                32'd11: rock_n_position <= 32'd10;
                default: rock_n_position <= 32'd3;
            endcase
    end else begin
            rock_n_position <= rock_position;   //state isn't changed
        end
    end

    always@ (posedge clk) begin
        if(~sw) begin
            clk_count <= 32'd0;
            change <= 1'b0;
        end else
            if (clock_count != 32'd4) begin
                clk_count <= clk_count + 1'b1;
                change <= 1'b1;
            end else begin
                clk_count <= 32'b0;
                change <= 1'b1;
            end
    end

    always @(rock_position) begin
        rock_position_out_comp <= rock_position;
        rock_position_out_display <= rock_position; 
    end

endmodule


//****************************************//
//              Peripheral                //
//****************************************//


//**************************************//
//              VGA Interface           //
//**************************************//

module vga_controller(
    input wire mclk,  
    input wire [0:0] sw,        
    
    output reg HSYNC_out,       
    output reg VSYNC_out,

    output wire [2:0] Red_out,
    output wire [2:0] Green_out,
    output wire [2:1] Blue_out,
    output wire clk_25Mhz,

    output wire [10:0] current_display_x_coordinate_out,
    output wire [10:0] current_display_y_coordinate_out,

    output wire video_on_flag_out
);
        
    parameter H_SYNC_CYC=96;
    parameter H_SYNC_BACK=48;   //46 -> 48
    parameter H_SYNC_ACT=640;
    parameter H_SYNC_FRONT=16;
    parameter H_SYNC_TOTAL=800;

    //Virtical Parameter( Line )
    parameter V_SYNC_CYC=2;
    parameter V_SYNC_BACK=33;
    parameter V_SYNC_ACT=480;
    parameter V_SYNC_FRONT=10;
    parameter V_SYNC_TOTAL=525;

    //*****************************************//
    //              CREATE 25MHZ               //
    //*****************************************//

    reg [1:0] clk_counter;

    always @(posedge mclk) begin
        if(~sw) begin
            clk_counter = 2'd0;
        end else if (clk_counter == 2'd3) begin
            clk_counter = 2'd0;
        end else begin
            clk_counter = clk_counter + 1'b1;
        end
    end

    assign clk_25Mhz = (clk_counter > 1) ? 1'b0 : 1'b1;
    
    //*****************************************//
    //              Create FSM counter         //
    //*****************************************//
    
    reg [10:0] H_count;
    reg [10:0] V_count;

    reg [10:0] new_H_count;
    reg [10:0] new_V_count;

    reg [10:0] H_count_next;
    reg [10:0] V_count_next;


    always @(posedge mclk) begin
        if(~sw) begin
            H_count <= 1'b0;
            V_count <= 1'b0;

            new_H_count <= 1'b0;
            new_V_count <= 1'b0;
        end else begin
            H_count <= H_count_next;
            V_count <= V_count_next;

            new_H_count <= H_count_next;
            new_V_count <= V_count_next;

        end
         
    end

    //****************************************//
    //          CREATE H_SYNC CLK             //
    //****************************************//

 
    always @(posedge clk_25Mhz) begin
        if(~sw) begin
            H_count_next <= 11'b0;
            HSYNC_out <= 1'b1;
        end else begin
            if (H_count < H_SYNC_TOTAL) begin
                H_count_next <= H_count + 1'b1;
            end else begin
                H_count_next <= 1'b0;
            end
        end

        if(H_count < H_SYNC_CYC) begin
            HSYNC_out <= 1'b0;
        end else begin
            HSYNC_out <= 1'b1;
        end
    end

    //****************************************//
    //          CREATE V_SYNC CLK             //
    //****************************************//

    always @(posedge clk_25Mhz) begin
        if(~sw) begin
            V_count_next <= 11'b0;
            VSYNC_out <= 1'b1;
        end else begin
            if(H_count == 11'b0) begin
                if(V_count < V_SYNC_TOTAL) begin
                    V_count_next <= V_count + 1'b1;
                end else begin
                    V_count_next <= 11'b0;
                end

                if(V_count < V_SYNC_CYC) begin
                    VSYNC_out <= 1'b0;
                end else begin
                    VSYNC_out <= 1'b1;
                end
            end
        end
    end

    //********************************************//
    //         CREATE Horizontal PIXEL            //
    //********************************************//
    /*
    // if end of the monitor pixel reached, send image. 

    reg [10:0] H_image_count;
    reg H_image_flag;

    // create horizontal count
    always @(posedge clk_25Mhz) begin
        if(~sw) begin
            H_image_count <= 11'b0;
        end else begin
            if (H_count < H_SYNC_CYC + H_SYNC_BACK) begin
                H_image_count <= 11'b0;
            end else if (H_count >= H_SYNC_CYC + H_SYNC_BACK + H_SYNC_ACT) begin
                H_image_count <= 11'b0;
            end else begin
                H_image_count <= H_image_count + 1'b1;
            end
        end        
    end

    // create horizontal send bit
    always @(posedge clk_25Mhz) begin
        if(~sw) begin
            H_image_flag <= 1'b0;
        end else begin
            if( H_count < H_SYNC_CYC + H_SYNC_BACK) begin
                H_image_flag <= 1'b0;
            end else if ( H_count >= H_SYNC_CYC + H_SYNC_BACK + H_SYNC_ACT ) begin
                H_image_flag <= 1'b0;
            end else begin
                H_image_flag <= 1'b1;
            end
        end
    end
    
    */
    //********************************************//
    //           CREATE Vertical PIXEL            //
    //********************************************//

    /*
    reg [10:0] V_image_count;
    reg V_image_flag;

    // create vertical count
    always @(posedge clk_25Mhz) begin
        if(~sw) begin
            V_image_count <= 11'b0;
        end else begin 
            if(H_count == 11'b0) begin
                if(V_count < V_SYNC_CYC + V_SYNC_BACK) begin
                    V_image_count <= 1'b0;
                end else if ( V_count >= V_SYNC_CYC + V_SYNC_BACK + V_SYNC_ACT) begin
                    V_image_count <= 11'b0;
                end else begin
                    V_image_count <= V_image_count + 1'b1;
                end
            end
        end
    end
    // create vertial send bit
    always@ (posedge clk_25Mhz) begin
        if(~sw) begin
            V_image_flag <= 1'b0;
        end else begin
            if(H_count == 1'b0) begin
                if(V_count < V_SYNC_CYC + V_SYNC_BACK) begin
                    V_image_flag <= 1'b0;
                end else if (V_count >= V_SYNC_CYC + V_SYNC_BACK + V_SYNC_ACT)begin
                    V_image_flag <= 1'b0;
                end else begin
                    V_image_flag <= 1'b1;
                end
            end
        end
    end
    */

    assign video_on_flag_out = (H_count < 640) && (V_count < 480); // 0-639 and 0-479 respectively

    assign current_display_x_coordinate_out = new_H_count;
    assign current_display_y_coordinate_out = new_V_count;

    // DEBUG
    /*
    wire image_out;

    assign image_out = H_image_flag && V_image_flag;

    assign Red_out = (image_out && (V_image_count < 200)) ? 
                 3'b111 : 3'b000; 
    assign Green_out = (image_out && ~(V_image_count < 200) && (V_image_count < 400)) ? 
                 3'b111 : 3'b000; 
    assign Blue_out = (image_out && (V_image_count >= 400)) ? 2'b11 : 2'b00; 
    */

endmodule


module vga_graphic_interface(
                             input clk,
                             input btnU,
                             input btnD,
                             input [0:0] sw,
                             
                             input [10:0] current_display_x_coordinate_in,
                             input [10:0] current_display_y_coordinate_in,
                             input video_on_flag_in,
                            
                             input [31:0] ship_position_in,
                             input [31:0] rock_position_in,
                             input ship_position_changed_flag_in,
                             input rock_position_changed_flag_in,

                             output reg [11:0] graphic_rgb_out
);
    wire [10:0] x = current_display_x_coordinate_in;
    wire [10:0] y = current_display_y_coordinate_in;

    reg [31:0] x_ship_position_coordinate;
    reg [31:0] y_ship_position_coordinate;

    reg [31:0] x_rock_position_coordinate;
    reg [31:0] y_rock_position_coordinate;

    wire [9:0] y_rock_top;
    wire [9:0] y_rock_bottom;
    
    wire [11:0] ship_rgb_color = 12'hFFF;       // White 
    wire [11:0] rock_rgb_color = 12'hF00;       // Grey
    wire [11:0] background_rgb_color = 12'h000; // Black
    
    
    // create ship and rock position coordinate
    always @(posedge clk) begin
        if(~sw) begin
            x_ship_position_coordinate <= 1'b0;
            y_ship_position_coordinate <= 1'b0;
        end else begin
                case(ship_position_in)
                32'd0 : x_ship_position_coordinate = 32'd200;  // (200, 100)
                32'd4 : x_ship_position_coordinate = 32'd400;  // (400, 100)
                32'd8 : x_ship_position_coordinate = 32'd600;  // (600, 100)
                endcase

                y_ship_position_coordinate <= 32'd100;      
            end
            
            if(rock_position_changed_flag_in) begin
                case(rock_position_in)
                32'd3 : begin x_rock_position_coordinate = 32'd200; y_rock_position_coordinate = 32'd400; end   // (200, 400)
                32'd2 : begin x_rock_position_coordinate = 32'd200; y_rock_position_coordinate = 32'd300; end   // (200, 300)
                32'd1 : begin x_rock_position_coordinate = 32'd200; y_rock_position_coordinate = 32'd200; end   // (200, 200)
                32'd0 : begin x_rock_position_coordinate = 32'd200; y_rock_position_coordinate = 32'd100; end   // (200, 100)
                32'd7 : begin x_rock_position_coordinate = 32'd400; y_rock_position_coordinate = 32'd400; end   // (400, 400)
                32'd6 : begin x_rock_position_coordinate = 32'd400; y_rock_position_coordinate = 32'd300; end   // (400, 300)
                32'd5 : begin x_rock_position_coordinate = 32'd400; y_rock_position_coordinate = 32'd200; end   // (400, 200)
                32'd4 : begin x_rock_position_coordinate = 32'd400; y_rock_position_coordinate = 32'd100; end   // (400, 100)
                32'd11 : begin x_rock_position_coordinate = 32'd600; y_rock_position_coordinate = 32'd400; end  // (600, 400)
                32'd10 : begin x_rock_position_coordinate = 32'd600; y_rock_position_coordinate = 32'd300; end  // (600, 300)
                32'd9 : begin x_rock_position_coordinate = 32'd600; y_rock_position_coordinate = 32'd200; end   // (600, 200)
                32'd8 : begin x_rock_position_coordinate = 32'd600; y_rock_position_coordinate = 32'd100; end   // (600, 100)
                endcase
            end
        end
    
    //**********************************************//
    //          CREATE SHIP Graphic                 //
    //**********************************************//

    wire [9:0] x_ship_left_coordinate;
    wire [9:0] x_ship_right_coordinate;
    wire [9:0] y_ship_top_coordinate;
    wire [9:0] y_ship_bottom_coordinate;

    wire current_display_in_ship_coordinate;

    parameter MAX_SHIP_SIZE = 8;
    
    // referenced position top left edge
    assign x_ship_left_coordinate = x_ship_position_coordinate[9:0];
    assign y_ship_top_coordinate = y_ship_position_coordinate[9:0];
    assign x_ship_right_coordinate = x_ship_left_coordinate + MAX_SHIP_SIZE - 1;
    assign y_ship_bottom_coordinate = y_ship_top_coordinate + MAX_SHIP_SIZE - 1;

    assign current_display_in_ship_coordinate = (x_ship_left_coordinate <= x) &&
                                                (x <= x_ship_right_coordinate) &&
                                                (y_ship_top_coordinate <= y) &&
                                                (y <= y_ship_bottom_coordinate);

    //**************************************************//
    //          Get ship grhpic data from rom           //
    //**************************************************//

    reg [7:0] rom_data_ship;
    wire [2:0] rom_addr_ship;
    wire [2:0] rom_col_ship;
    wire rom_bit_ship;

    
    always @*
        case(rom_addr_ship) 
            3'b000 :    rom_data_ship = 8'b11111111;     //|********| 
            3'b001 :    rom_data_ship = 8'b11000011;     //|**    **|
            3'b010 :    rom_data_ship = 8'b11111111;     //|********|
            3'b011 :    rom_data_ship = 8'b01111110;     //| ****** |
            3'b100 :    rom_data_ship = 8'b01111110;     //| ****** |
            3'b101 :    rom_data_ship = 8'b11111111;     //|********|
            3'b110 :    rom_data_ship = 8'b11000011;     //|**    **|
            3'b111 :    rom_data_ship = 8'b11111111;     //|********|
        endcase

    assign rom_addr_ship = y[2:0] - y_ship_top_coordinate[2:0];
    assign rom_col_ship = x[2:0] - x_ship_left_coordinate[2:0];
    assign rom_bit_ship = rom_data_ship[rom_col_ship];
    
    wire ship_position_area;

    assign ship_position_area = current_display_in_ship_coordinate & rom_bit_ship;

    //*****************************************************//
    //              CREATE Rock GRAHPIC                    //
    //*****************************************************//
    
    wire [9:0] x_rock_left_coordinate;
    wire [9:0] x_rock_right_coordinate;
    wire [9:0] y_rock_top_coordinate;
    wire [9:0] y_rock_bottom_coordinate;

    wire current_display_in_rock_coordinate;

    parameter MAX_ROCK_SIZE = 8;
    
    // referenced position top left edge
    assign x_rock_left_coordinate = x_rock_position_coordinate[9:0];
    assign y_rock_top_coordinate = y_rock_position_coordinate[9:0];
    assign x_rock_right_coordinate = x_rock_left_coordinate + MAX_ROCK_SIZE - 1;
    assign y_rock_bottom_coordinate = y_rock_top_coordinate + MAX_ROCK_SIZE - 1;

    assign current_display_in_rock_coordinate = (x_rock_left_coordinate <= x) &&
                                                (x <= x_rock_right_coordinate) &&
                                                (y_rock_top_coordinate <= y) &&
                                                (y <= y_rock_bottom_coordinate);

    //**************************************************//
    //          Get ship grhpic data from rom           //
    //**************************************************//
    wire rom_bit_rock;
    
    wire [2:0] rom_addr_rock;
    wire [2:0] rom_col_rock;
    reg [7:0] rom_data_rock;

     // rom for siave rock graphic design
    always @*
        case(rom_addr_rock) 
            3'b000 :    rom_data_rock = 8'b11111111;     //|********| 
            3'b001 :    rom_data_rock = 8'b11111111;     //|********|
            3'b010 :    rom_data_rock = 8'b11111111;     //|********|
            3'b011 :    rom_data_rock = 8'b11111111;     //|********|
            3'b100 :    rom_data_rock = 8'b11111111;     //|********|
            3'b101 :    rom_data_rock = 8'b11111111;     //|********|
            3'b110 :    rom_data_rock = 8'b11111111;     //|********|
            3'b111 :    rom_data_rock = 8'b11111111;     //|********|
        endcase
    
    assign rom_addr_rock = y[2:0] - y_rock_top_coordinate[2:0];
    assign rom_col_rock = y[2:0] - x_rock_left_coordinate[2:0];
    assign rom_bit_rock = rom_data_rock[rom_col_rock];


    wire rock_position_area;
     
    assign rock_position_area = current_display_in_rock_coordinate & rom_bit_rock;
    
    // choose color of pixel of diplay
    always @(posedge clk) begin
        if(~video_on_flag_in) begin
            graphic_rgb_out = 12'h000;
        end else begin
            if(ship_position_area) begin
                graphic_rgb_out = ship_rgb_color;
            end else if (rock_position_area) begin
                graphic_rgb_out <= rock_rgb_color;
            end else begin
                graphic_rgb_out = background_rgb_color;
            end
        end
    end

endmodule

//**********************************************//
//                  FINAL VIDEO                 //
//**********************************************//

module generate_video(
                      input wire clk,
                      input wire [0:0]sw,
                      input wire [11:0] video_data_in,
                      input wire clk_25Mhz,

                      output reg [11:0] rgb_out
);
    reg [11:0] rgb_next_state;

    always @(video_data_in) begin
        rgb_next_state <= video_data_in;
    end


    always @(posedge clk) begin
        if(~sw) begin
            rgb_out <= 1'b0;
        end else begin
            if (clk_25Mhz) begin
                rgb_out <= rgb_next_state;
            end
        end
    end
    
    
endmodule

//*************************************//
//          KEYBOARD                   //
//*************************************//

module Keyboard(
                input clk,
                input [0:0] sw,	
                input PS2Clk,	
                input PS2Data,
                
                output reg [7:0] led,
                output reg [1:0] btn_out
   );

	wire [7:0] ARROW_LEFT = 8'h6B;
	wire [7:0] ARROW_RIGHT = 8'h74;
	
	reg read;				
	reg [11:0] count_reading;		
	reg PREVIOUS_STATE;			
	reg scan_err;				
	reg [10:0] scan_code;			
	reg [7:0] CODEWORD;			
	reg TRIG_ARR;				
	reg [3:0]COUNT;				
	reg TRIGGER = 0;			 
	reg [7:0]DOWNCOUNTER = 0;		

	initial begin
		PREVIOUS_STATE = 1;		
		scan_err = 0;		
		scan_code = 0;
		COUNT = 0;			
		CODEWORD = 0;
		led = 0;
		read = 0;
		count_reading = 0;
	end

	always @(posedge clk) begin				
		if (DOWNCOUNTER < 249) begin			 
			DOWNCOUNTER <= DOWNCOUNTER + 1;
			TRIGGER <= 0;
		end
		else begin
			DOWNCOUNTER <= 0;
			TRIGGER <= 1;
		end
	end
	
	always @(posedge clk) begin	
		if (TRIGGER) begin
			if (read)				
				count_reading <= count_reading + 1;	
			else 						
				count_reading <= 0;			
		end
	end


	always @(posedge clk) begin		
	if (TRIGGER) begin						
		if (PS2Clk != PREVIOUS_STATE) begin			
			if (!PS2Clk) begin				
				read <= 1;				
				scan_err <= 0;				
				scan_code[10:0] <= {PS2Data, scan_code[10:1]};	
				COUNT <= COUNT + 1;			
			end
		end
		else if (COUNT == 11) begin				
			COUNT <= 0;
			read <= 0;					
			TRIG_ARR <= 1;					
			if (!scan_code[10] || scan_code[0] || !(scan_code[1]^scan_code[2]^scan_code[3]^scan_code[4]
				^scan_code[5]^scan_code[6]^scan_code[7]^scan_code[8]
				^scan_code[9]))
				scan_err <= 1;
			else 
				scan_err <= 0;
		end	
		else  begin						
			TRIG_ARR <= 0;					
			if (COUNT < 11 && count_reading >= 4000) begin	
				COUNT <= 0;				
				read <= 0;				
			end
		end
	PREVIOUS_STATE <= PS2Clk;					
	end
	end


	always @(posedge clk) begin
		if (TRIGGER) begin					
			if (TRIG_ARR) begin				
				if (scan_err) begin			
					CODEWORD <= 8'd0;		
				end
				else begin
					CODEWORD <= scan_code[8:1];
				end				
			end					
			else CODEWORD <= 8'd0;				
		end
		else CODEWORD <= 8'd0;					
	end
	
	always @(posedge clk) begin
        if (~sw) begin
            btn_out <= 3'b0;
        end else begin
            if (CODEWORD == ARROW_LEFT)	begin			
			    led <= led + 1;
                btn_out[0] <= 1'b1;
                btn_out[1] <= 1'b0;				
            end	
		    else if (CODEWORD == ARROW_RIGHT) begin		
			    led <= led - 1;
                btn_out[0] <= 1'b0;
                btn_out[1] <= 1'b1;					
            end
            else begin
                btn_out[0] <= 1'b0;
                btn_out[1] <= 1'b0;
            end
        end
    end

endmodule
