	//
	//------------------------------------------------------------------------------
	//   Copyright 2010 Mentor Graphics Corporation
	//   All Rights Reserved Worldwide
	//
	//   Licensed under the Apache License, Version 2.0 (the
	//   "License"); you may not use this file except in
	//   compliance with the License.  You may obtain a copy of
	//   the License at
	//
	//       http://www.apache.org/licenses/LICENSE-2.0
	//
	//   Unless required by applicable law or agreed to in
	//   writing, software distributed under the License is
	//   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	//   CONDITIONS OF ANY KIND, either express or implied.  See
	//   the License for the specific language governing
	//   permissions and limitations under the License.
	//------------------------------------------------------------------------------
	//
	// This example illustrates how to implement a bidirectional driver-sequence use
	// model. It uses get_next_item(), item_done() in the driver.
	//
	// It includes a bidirectional slave DUT, and the bus transactions are reported to
	// the transcript.
	//
	`define uvm_record_field(NAME,VALUE) \
	   $add_attribute(recorder.tr_handle,VALUE,NAME);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//PACKAGE
	package bidirect_bus_pkg;

		import uvm_pkg::*;
		`include "uvm_macros.svh"

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// GPIO sequence item
		class GPIO_seq_item extends uvm_sequence_item;

			logic[7:0]   gp_op_valid;
			logic[255:0] gp_op;
			logic[255:0] gp_ip;

			`uvm_object_utils(GPIO_seq_item)

			function new(string name = "GPIO_seq_item");
			  super.new(name);
			endfunction
			//constraint at_least_1 { delay inside {[1:20]};} ????
			//constraint align_32 {addr[1:0] == 0;} ????
			function void do_copy(uvm_object rhs);
			  GPIO_seq_item rhs_;

			  if(!$cast(rhs_, rhs)) begin
			    `uvm_error("do_copy", "cast failed, check types");
			  end
			  gp_op_valid = rhs_.gp_op_valid;
			  gp_op = rhs_.gp_op;
			  gp_ip = rhs_.gp_ip;
			endfunction: do_copy

			function bit do_compare(uvm_object rhs, uvm_comparer comparer);
			  GPIO_seq_item rhs_;

			  do_compare = $cast(rhs_, rhs) &&
			               super.do_compare(rhs, comparer) &&
			               gp_op_valid		===	rhs_.gp_op_valid &&
			               gp_op 	=== 	rhs_.gp_op &&
			               gp_ip 	===	rhs_.gp_ip;
			endfunction: do_compare

			function string convert2string();
			  return $sformatf("%s\n gp_op_valid:\t%0h\n gp_op:\t%256h\n gp_ip:\t%0b\n",
			                    super.convert2string(), gp_op_valid, gp_op, gp_ip);
			endfunction: convert2string

			function void do_print(uvm_printer printer);

			  if(printer.knobs.sprint == 0) begin
			    $display(convert2string());
			  end
			  else begin
			    printer.m_string = convert2string();
			  end

			endfunction: do_print

			function void do_record(uvm_recorder recorder);
			  super.do_record(recorder);

			  `uvm_record_field("gp_op_valid", gp_op_valid);
			  `uvm_record_field("gp_op", gp_op);
			  `uvm_record_field("gp_ip", gp_ip);

			endfunction: do_record
		endclass: GPIO_seq_item
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Bus sequence item
		class bus_seq_item extends uvm_sequence_item;

			// Request fields
			rand logic[31:0] addr;
			rand logic[31:0] write_data;
			rand bit read_not_write;
			rand int delay;

			// Response fields
			bit error;
			logic[31:0] read_data;

			`uvm_object_utils(bus_seq_item)

			function new(string name = "bus_seq_item");
			  super.new(name);
			endfunction

			constraint at_least_1 { delay inside {[1:20]};}

			constraint align_32 {addr[1:0] == 0;}

			//constraint read_cyc { (read_not_write == 1) -> ( write_data == 'x);}

			//constraint write_cyc { (read_not_write == 0) -> ( read_data == 'x);}

			function void do_copy(uvm_object rhs);
			  bus_seq_item rhs_;

			  if(!$cast(rhs_, rhs)) begin
			    `uvm_error("do_copy", "cast failed, check types");
			  end
			  addr = rhs_.addr;
			  write_data = rhs_.write_data;
			  read_not_write = rhs_.read_not_write;
			  delay = rhs_.delay;
			  error = rhs_.error;
			  read_data = rhs_.read_data;
			endfunction: do_copy

			function bit do_compare(uvm_object rhs, uvm_comparer comparer);
			  bus_seq_item rhs_;

			  do_compare = $cast(rhs_, rhs) &&
			               super.do_compare(rhs, comparer) &&
			               addr		===	rhs_.addr &&
			               write_data 	=== 	rhs_.write_data &&
			               read_not_write 	===	rhs_.read_not_write &&
			              // delay === rhs_.delay &&//nem tudjuk elore
			               error 		=== 	rhs_.error &&
			               read_data 	=== 	rhs_.read_data;
			endfunction: do_compare

			function string convert2string();
			  return $sformatf("%s\n addr:\t%0h\n write_data:\t%0h\n read_not_write:\t%0b\n delay:\t%0d\n error:\t%0b\n read_data:\t%0h",
			                    super.convert2string(), addr, write_data, read_not_write, delay, error, read_data);
			endfunction: convert2string

			function void do_print(uvm_printer printer);

			  if(printer.knobs.sprint == 0) begin
			    $display(convert2string());
			  end
			  else begin
			    printer.m_string = convert2string();
			  end

			endfunction: do_print

			function void do_record(uvm_recorder recorder);
			  super.do_record(recorder);

			  `uvm_record_field("addr", addr);
			  `uvm_record_field("write_data", write_data);
			  `uvm_record_field("read_not_write", read_not_write);
			  `uvm_record_field("delay", delay);
			  `uvm_record_field("error", error);
			  `uvm_record_field("read_data", read_data);

			endfunction: do_record
		endclass: bus_seq_item
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Bidirectional DRIVER - uses:
		// get_next_item() to get the next instruction item
		// item_done() to indicate that the dirver has finished with the item
		class bidirect_bus_driver extends uvm_driver #(bus_seq_item);

			`uvm_component_utils(bidirect_bus_driver)

			bus_seq_item req;

			uvm_analysis_port #(bus_seq_item) d_item_collected_port; //analizis portokat at kelll majd gondolni

			virtual bus_if BUS_agnt_vi;

			function new(string name = "bidirect_bus_driver", uvm_component parent );
			  super.new(name, parent);
			  d_item_collected_port = new("d_item_collected_port", this);
			endfunction

			task run_phase(uvm_phase phase);

			  // Default conditions:
			  BUS_agnt_vi.valid <= 0;
			  BUS_agnt_vi.rnw <= 1;
			  // Wait for reset to end
			  @(posedge BUS_agnt_vi.resetn);
			  forever
			    begin
			      seq_item_port.get_next_item(req); // Start processing req item
			      repeat(req.delay) begin
			        @(posedge BUS_agnt_vi.clk);
			      end
			      BUS_agnt_vi.valid <= 1;
			      BUS_agnt_vi.addr <= req.addr;
			      BUS_agnt_vi.rnw <= req.read_not_write;
			      if(req.read_not_write == 0) begin
			        BUS_agnt_vi.write_data <= req.write_data;
			      end
			      while(BUS_agnt_vi.ready != 1) begin
			        @(posedge BUS_agnt_vi.clk);
			      end
			      // At end of the pin level bus transaction
			      // Copy response data into the req fields:
			      if(req.read_not_write == 1) begin
			        req.read_data = BUS_agnt_vi.read_data; // If read - copy returned read data
			      end
			      req.error = BUS_agnt_vi.error; // Copy bus error status
			      BUS_agnt_vi.valid <= 0; // End the pin level bus transaction
			      d_item_collected_port.write(req); 
			      seq_item_port.item_done(); // End of req item
				`uvm_info("driver side", req.convert2string(), UVM_LOW);
			    end
			endtask: run_phase
		endclass: bidirect_bus_driver
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// MONITOR:
		class mye_momnitor extends uvm_monitor;
			`uvm_component_utils(mye_momnitor)
			bus_seq_item req;
			virtual bus_if BUS_agnt_vi;

			uvm_analysis_port #(bus_seq_item) item_collected_port; //analizis portokat at kelll majd gondolni

			uvm_analysis_port #(bus_seq_item) dut_in_tx_port; //analizis portokat at kelll majd gondolni
			uvm_analysis_port #(bus_seq_item) dut_out_tx_port; //analizis portokat at kelll majd gondolni

			function new(string name = "mye_momnitor", uvm_component parent);
			  super.new(name, parent);
			  item_collected_port = new("item_collected_port", this);
			  dut_in_tx_port = new("dut_in_tx_port", this);
			  dut_out_tx_port = new("dut_out_tx_port", this);
			endfunction

			task run_phase(uvm_phase phase);
				/* Request fields
					rand logic[31:0] addr;
					rand logic[31:0] write_data;
					rand bit read_not_write;
					rand int delay;

				 	Response fields
					bit error;
					logic[31:0] read_data;
				*/
				  forever begin
				      @(posedge BUS_agnt_vi.ready);
				    req = bus_seq_item::type_id::create ("req", this);
				      req.error = BUS_agnt_vi.error;
				      req.addr = BUS_agnt_vi.addr; // get address
				     if(BUS_agnt_vi.rnw)  begin // is it a write?
				         req.read_data = BUS_agnt_vi.read_data;  // get data   
					 req.read_not_write = 1; // set op type
				      end else begin
				         req.write_data = BUS_agnt_vi.write_data;  // get data
					 req.read_not_write = 0; // set op type
				      end
				 $display("Following goes to analysis port:");
				`uvm_info("monitor side", req.convert2string(), UVM_LOW);
				      item_collected_port.write(req); 				
				end			    
			endtask: run_phase
		endclass: mye_momnitor
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// GPIO MONITOR:
		class GPIO_momnitor extends uvm_monitor;
			`uvm_component_utils(GPIO_momnitor)
			GPIO_seq_item req;
			virtual gpio_if GPIO_agnt_vi;
			uvm_analysis_port #(GPIO_seq_item) GPIO_dut_out_tx_port; //analizis portokat at kelll majd gondolni

			function new(string name = "GPIO_momnitor", uvm_component parent);
			  super.new(name, parent);
			  GPIO_dut_out_tx_port = new("GPIO_dut_out_tx_port", this);
			endfunction

			task run_phase(uvm_phase phase);
				  forever begin
				  	req = GPIO_seq_item::type_id::create ("req", this);
				     @(	posedge GPIO_agnt_vi.gp_op_valid[0] ||
				      			GPIO_agnt_vi.gp_op_valid[1] || 
				      			GPIO_agnt_vi.gp_op_valid[2] || 
				      			GPIO_agnt_vi.gp_op_valid[3] || 
				      			GPIO_agnt_vi.gp_op_valid[4] ||
				      			GPIO_agnt_vi.gp_op_valid[5] ||
				      			GPIO_agnt_vi.gp_op_valid[6] ||
				      			GPIO_agnt_vi.gp_op_valid[7]);
				      // is it a valid output?
				         req.gp_op = GPIO_agnt_vi.gp_op;  // get data   
				         req.gp_ip = GPIO_agnt_vi.gp_ip;  // get data 
				         req.gp_op_valid = GPIO_agnt_vi.gp_op_valid; //get data  					 
				     
				    $display("Following goes to analysis port FROM GPIO_momnitor:");
					`uvm_info("GPIO monitor side", req.convert2string(), UVM_LOW);
				    GPIO_dut_out_tx_port.write(req); 				
				     
				end			    
			endtask: run_phase
		endclass: GPIO_momnitor
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Bus SequenceR:
		class bidirect_bus_sequencer extends uvm_sequencer #(bus_seq_item);

			`uvm_component_utils(bidirect_bus_sequencer)

			function new(string name = "bidirect_bus_sequencer", uvm_component parent = null);
			  super.new(name, parent);
			endfunction
		endclass: bidirect_bus_sequencer
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Predictor:	
		class my_predictor extends uvm_subscriber #(bus_seq_item);
			`uvm_component_utils(my_predictor)
			function new(string name, uvm_component parent);
    			super.new(name, parent);
  			endfunction
			GPIO_seq_item expected_txn;	
		//	GPIO_seq_item expected_txn_cop;
			uvm_analysis_port #(GPIO_seq_item) expected_port;
			//bus_seq_item t;
			//GPIO_seq_item expected_txn;
			function void build_phase(uvm_phase phase);
				expected_port = new("expected_port", this);
				expected_txn = new("expected_txn");
				expected_txn.gp_op_valid[7:0] = 0;
				expected_txn.gp_op[255:0] = 256'h0;
			endfunction  			

			function void write(input bus_seq_item t);
				//expected_txn = GPIO_seq_item::type_id::create ("expected_txn", this);
				//if($cast(expected_txn_cop, expected_txn.clone())) `uvm_fatal("COW fatal", "Can't copy sequence item in predictor") //COPY ON WRITE
				
				expected_txn.gp_op_valid[7:0] = 0;
			  	
			  	if(t.read_not_write == 0) begin // Write
				  	if(t.addr inside{[32'h0100_0000:32'h0100_001C]}) begin
						case(t.addr[7:0]) //get and save expected results
						//here logic needed to represent the behaviour of the dut
								8'h00: begin expected_txn.gp_op_valid[0] = 1; expected_txn.gp_op[31 :0  ] = t.write_data; end
					            8'h04: begin expected_txn.gp_op_valid[1] = 1; expected_txn.gp_op[63 :32 ] = t.write_data; end
					            8'h08: begin expected_txn.gp_op_valid[2] = 1; expected_txn.gp_op[95 :64 ] = t.write_data; end
					            8'h0c: begin expected_txn.gp_op_valid[3] = 1; expected_txn.gp_op[127:96 ] = t.write_data; end
					            8'h10: begin expected_txn.gp_op_valid[4] = 1; expected_txn.gp_op[159:128] = t.write_data; end
					            8'h14: begin expected_txn.gp_op_valid[5] = 1; expected_txn.gp_op[191:160] = t.write_data; end
					            8'h18: begin expected_txn.gp_op_valid[6] = 1; expected_txn.gp_op[223:192] = t.write_data; end
					            8'h1c: begin expected_txn.gp_op_valid[7] = 1; expected_txn.gp_op[255:224] = t.write_data; end
						endcase // t.addr
					end
				    expected_port.write(expected_txn); //send expected results to the evaluator
				end				
				
			endfunction  
		endclass : my_predictor
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Evaluator or Comparator:		
		class my_evaluator extends uvm_component /* base class*/;
			//Factory registration and construstor
			`uvm_component_utils(my_evaluator)
			function new(string name, uvm_component parent);
    			super.new(name, parent);
  			endfunction
			uvm_analysis_export #(GPIO_seq_item) expected_export;
			uvm_tlm_analysis_fifo #(GPIO_seq_item) expected_fifo;

			uvm_analysis_export #(GPIO_seq_item) actual_export;
			uvm_tlm_analysis_fifo #(GPIO_seq_item) actual_fifo;

			int match, mismatch;

			function void build_phase(uvm_phase phase);
				expected_fifo		= new("expected_fifo",this);
				expected_export		= new("expected_export",this);
				actual_fifo			= new("actual_fifo",this);
				actual_export		= new("actual_export",this);
			endfunction

			virtual function void connect_phase(uvm_phase phase);
				expected_export.connect(expected_fifo.analysis_export);
				actual_export.connect(actual_fifo.analysis_export);
			endfunction

			virtual task run_phase(uvm_phase phase);				
				forever begin
					GPIO_seq_item expected_txn, actual_txn;
					actual_fifo.get(actual_txn);
					expected_fifo.get(expected_txn);
					if(actual_txn.compare(expected_txn))begin
						match++;
						`uvm_info("Evaluator",$sformatf("%s EXPECTED above does /MATCH/ ACTUAL below %s",expected_txn.convert2string(), actual_txn.convert2string()), UVM_LOW)
					end
					else begin
						`uvm_error("Evaluator", $sformatf("%s EXPECTED above does not match ACTUAL below %s",expected_txn.convert2string(), actual_txn.convert2string()))
						mismatch++;
					end // else
				end // forever
			endtask : run_phase
			
			function void report_phase(uvm_phase phase);
				`uvm_info("Evaluator", $sformatf("Matched=%0d, mismatch=%0d",match, mismatch), UVM_LOW)	
			endfunction
		endclass : my_evaluator
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Flat Scoreboard:
		class mye_scoreboard extends uvm_scoreboard;		 
			`uvm_component_utils(mye_scoreboard) //reg in the factory done
			uvm_analysis_export #(bus_seq_item)sb_export_before;
			uvm_analysis_export #(bus_seq_item)sb_export_after;
			uvm_tlm_analysis_fifo #(bus_seq_item) before_fifo;
			uvm_tlm_analysis_fifo #(bus_seq_item) after_fifo;

			 //bus_seq_item req;//eredeti
			bus_seq_item tr_before;
			bus_seq_item tr_after;

			function new(string name, uvm_component parent);//ez csak sablon? vagy igy mar ready?
			  super.new(name, parent);
			  tr_before = new("tr_before");
			  tr_after = new("tr_after");
			endfunction : new
			   
			function void build_phase(uvm_phase phase);
			  super.build_phase(phase);
			  sb_export_before = new("sb_export_before",this);
			  sb_export_after  = new("sb_export_after",this);
			  before_fifo      = new("before_fifo",this); 
			  after_fifo       = new("after_fifo",this);			 
			endfunction : build_phase

			function void connect_phase(uvm_phase phase);
			    sb_export_before.connect(before_fifo.analysis_export);
			    sb_export_after.connect(after_fifo.analysis_export);				
			endfunction: connect_phase    

			    //ez nem tudom, hogy hova jojjon:
			    //
			    task run();
			      forever begin
			        before_fifo.get(tr_before);
			        after_fifo.get(tr_after);			       
					//compare();
					if(tr_before.compare(tr_after))begin
						//match++;
						`uvm_info("ITS A MATCH", $sformatf("%s does match %s", tr_after.convert2string(), tr_before.convert2string()), UVM_LOW);
					end else begin
						`uvm_error("FLAT_Evaluator", $sformatf("%s does not match %s", tr_after.convert2string(), tr_before.convert2string()));
					end
			      end
			    endtask: run
		endclass : mye_scoreboard
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Hierarchical Scoreboard:
		class mye_h_scoreboard extends uvm_scoreboard;		 
			`uvm_component_utils(mye_h_scoreboard) //reg in the factory done

			my_predictor predictor;
			my_evaluator evaluator;
			
			uvm_analysis_export #(bus_seq_item) dut_in_export;
			uvm_analysis_export #(GPIO_seq_item) dut_out_export;
			//uvm_tlm_analysis_fifo #(GPIO_seq_item) comparator_output;
			//GPIO_seq_item comp_out_item;
			function new(string name, uvm_component parent);
    			super.new(name, parent);
    		//	comp_out_item = new("comp_out_item");    			
  			endfunction

			function void build_phase(uvm_phase phase);
				super.build_phase(phase);
				dut_in_export = new("dut_in_export", this);
				dut_out_export = new("dut_out_export", this);
 				predictor = my_predictor::type_id::create("predictor", this);
 				evaluator = my_evaluator::type_id::create("evaluator", this);
 				//	GPIO_dut_out_tx_port = new("GPIO_dut_out_tx_port",this);
 				//	comparator_output       = new("comparator_output",this);
			endfunction

			function void connect_phase(uvm_phase phase);
				dut_in_export.connect(predictor.analysis_export);
				predictor.expected_port.connect(evaluator.expected_export);
				dut_out_export.connect(evaluator.actual_export);
				//	GPIO_dut_out_tx_port.connect(comparator_output.analysis_export);
			endfunction: connect_phase    

				/*task run();
			    forever begin
			 	//	comparator_output.get(comp_out_item);
			 	end
			    endtask: run*/
		/*
			//uvm_analysis_imp#(bus_seq_item, mye_scoreboard) item_collected_export; //eredeti
			uvm_analysis_export #(bus_seq_item)sb_export_before;
			uvm_analysis_export #(bus_seq_item)sb_export_after;

			uvm_analysis_export #(GPIO_seq_item) GPIO_dut_out_tx_port;
			uvm_tlm_analysis_fifo #(GPIO_seq_item) comparator_output;

			uvm_tlm_analysis_fifo #(bus_seq_item) before_fifo;
			uvm_tlm_analysis_fifo #(bus_seq_item) after_fifo;



			 //bus_seq_item req;//eredeti
			bus_seq_item tr_before;
			bus_seq_item tr_after;
			GPIO_seq_item comp_out_item;

			function new(string name, uvm_component parent);//ez csak sablon? vagy igy mar ready?
			  super.new(name, parent);
			  tr_before = new("tr_before");
			  tr_after = new("tr_after");
			  comp_out_item = new("comp_out_item");
			endfunction : new
			   
			function void build_phase(uvm_phase phase);
			  super.build_phase(phase);
			  sb_export_before = new("sb_export_before",this);
			  sb_export_after  = new("sb_export_after",this);

			  GPIO_dut_out_tx_port = new("GPIO_dut_out_tx_port",this);

			  before_fifo      = new("before_fifo",this); 
			  after_fifo       = new("after_fifo",this);
			  comparator_output       = new("comparator_output",this);
			endfunction : build_phase

			function void connect_phase(uvm_phase phase);
			    sb_export_before.connect(before_fifo.analysis_export);
			    sb_export_after.connect(after_fifo.analysis_export);
				GPIO_dut_out_tx_port.connect(comparator_output.analysis_export);
			endfunction: connect_phase    

			    //ez nem tudom, hogy hova jojjon:
			    //
			    task run();
			      forever begin
			        before_fifo.get(tr_before);
			        after_fifo.get(tr_after);
			        comparator_output.get(comp_out_item);
				//compare();
				if(tr_before.compare(tr_after))begin
					//match++;
					`uvm_info("ITS A MATCH", $sformatf("%s does match %s", tr_after.convert2string(), tr_before.convert2string()), UVM_LOW);
				end else begin
					`uvm_error("Evaluator", $sformatf("%s does not match %s", tr_after.convert2string(), tr_before.convert2string()));
				end
			      end
			    endtask: run*/
		endclass : mye_h_scoreboard
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Bus SEQUENCE, which shows how the req object contains the result at the end
		// of the control handshake with the driver via the sequencer
		class bus_seq extends uvm_sequence #(bus_seq_item);

			`uvm_object_utils(bus_seq)

			bus_seq_item req;

			rand int limit = 40; // Controls the number of iterations

			function new(string name = "bus_seq");
			  super.new(name);
			endfunction

			task body;
			  req = bus_seq_item::type_id::create("req");

			  repeat(limit)
			    begin
			      start_item(req);
			      // The address is constrained to be within the address of the GPIO function
			      // within the DUT, The result will be a request item for a read or a write
			    assert(req.randomize() with {addr inside {[32'h0100_0000:32'h0100_001C]};});
				if(req.read_not_write == 1) begin
				req.write_data = 'x;
				end else begin
				req.read_data = 'x;	
				end

			      finish_item(req);
			      // The req handle points to the object that the driver has updated with response data
			     // `uvm_info("seq_body", req.convert2string(), UVM_LOW);
				$display("##---------------------------##------------------------------##");
			    end
			endtask: body
		endclass: bus_seq
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//////CONFIG PART//////////////////////////////////////////////////////////////////////////////////////////////
		class bus_agent_config extends uvm_object;
			`uvm_object_utils(bus_agent_config)
			function new(string name = "bus_agent_config");
     				super.new(name);
  			endfunction: new
  			uvm_active_passive_enum active = UVM_ACTIVE;				
			virtual bus_if BUS_agnt_vi;	
		endclass: bus_agent_config

		class GPIO_agent_config extends  uvm_object;
			`uvm_object_utils(GPIO_agent_config)
			function new(string name = "GPIO_agent_config");
    			super.new(name);
  			endfunction: new
			virtual gpio_if GPIO_agnt_vi;
			uvm_active_passive_enum active = UVM_PASSIVE;			
		endclass: GPIO_agent_config

		class env_conf extends uvm_object;
			`uvm_object_utils(env_conf)
			//bit enable_scoreboard = 1;
			//bit enable_coverage = 1;
			function new(string name = "env_conf");
    			super.new(name);
  			endfunction: new
			bus_agent_config bus_agt_cfg; 
			GPIO_agent_config GPIO_agent_configur; 	   			
		endclass: env_conf
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// GPIO_AGENT:
		class agent_gpio extends  uvm_agent;
			`uvm_component_utils(agent_gpio)
			function new(string name, uvm_component parent);
				super.new(name, parent);
			endfunction
			
			GPIO_agent_config GPIO_agent_configur; //CHECK IF READY TO IMPLEMENT
			GPIO_momnitor m_monitor;

			uvm_analysis_port #(GPIO_seq_item) GPIO_dut_out_tx_port; 

			function void build_phase(uvm_phase phase);
				if (!uvm_config_db#(GPIO_agent_config)::get(this, "", "GPIO_agent_config", GPIO_agent_configur)) 
      				`uvm_fatal("GPIO_agent", "GPIO_agent config not found")
		
				m_monitor = GPIO_momnitor::type_id::create("m_monitor", this);
				GPIO_dut_out_tx_port = new("GPIO_dut_out_tx_port", this);
			endfunction : build_phase

			//inner connections: only inside of the agent
			function void connect_phase(uvm_phase phase); 				
				m_monitor.GPIO_dut_out_tx_port.connect(this.GPIO_dut_out_tx_port);
				m_monitor.GPIO_agnt_vi = GPIO_agent_configur.GPIO_agnt_vi;
			endfunction
		endclass : agent_gpio
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// BUS_AGENT:
		class agent_my extends  uvm_agent;
			`uvm_component_utils(agent_my)
			function new(string name, uvm_component parent);
				super.new(name, parent);
			endfunction
			
			bus_agent_config bus_agent_configur; //HIBA
			bidirect_bus_driver m_driver;
			mye_momnitor m_monitor;
			bidirect_bus_sequencer m_sequencer;	

			uvm_analysis_port #(bus_seq_item) dut_in_tx_port; 
			uvm_analysis_port #(bus_seq_item) dut_out_tx_port; 

			function void build_phase(uvm_phase phase);
				if (!uvm_config_db#(bus_agent_config)::get(this, "", "bus_agent_config", bus_agent_configur)) //HIBA
      				`uvm_fatal("opb_master_agent", "opb_master_agent config not found")
		
				m_monitor = mye_momnitor::type_id::create("m_monitor", this);
				
				if (bus_agent_configur.active == UVM_ACTIVE) begin
					//aktivitas allitas
					m_driver = bidirect_bus_driver::type_id::create("m_driver", this);
					m_sequencer = new("m_sequencer", this); //no factory			
				end
				dut_in_tx_port = new("dut_in_tx_port", this);
				dut_out_tx_port = new("dut_out_tx_port", this);
			endfunction : build_phase

			//inner connections: only inside of the agent
			function void connect_phase(uvm_phase phase); //VIRTUAL?
				
				m_monitor.dut_out_tx_port.connect(this.dut_out_tx_port);
				m_monitor.dut_in_tx_port.connect(this.dut_in_tx_port);
				
				/*if (!uvm_config_db #(virtual bus_if)::get(this, "", "BUS_vif", m_driver.BUS))
				    `uvm_error("connect_phase", "uvm_config_db #(virtual bus_if)::get(...) failed");
				  if (!uvm_config_db #(virtual bus_if)::get(this, "", "BUS_vif", m_monitor.BUS))      
				    `uvm_error("connect_phase", "uvm_config_db #(virtual bus_if)::get(...) failed");
				*/

				m_monitor.BUS_agnt_vi = bus_agent_configur.BUS_agnt_vi;

			    if (bus_agent_configur.active == UVM_ACTIVE) begin
			    	m_driver.seq_item_port.connect(m_sequencer.seq_item_export); //DONE
			    	m_driver.BUS_agnt_vi = bus_agent_configur.BUS_agnt_vi;
			    end
			endfunction
		endclass : agent_my
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// ENVIRONMENT:
		class env_my extends  uvm_env;
			`uvm_component_utils(env_my)
			function new(string name, uvm_component parent);
				super.new(name, parent);
			endfunction
			mye_scoreboard m_scoreboard;
			mye_h_scoreboard m_h_scoreboard;     
			env_conf m_env_config; // 1) deklaralunk egy ures objectet //HIBA
			agent_my agt;
			agent_gpio gpio_agt;

			function void build_phase(uvm_phase phase);
					agt = agent_my::type_id::create("agt", this);
					gpio_agt = agent_gpio::type_id::create("gpio_agt", this);
					// 2) az ures objektet tartalommal toltjuk fel a DB-bol
					if (!uvm_config_db #(env_conf)::get(this, "m_env_config", "env_conf", m_env_config)) //HIBA
				    `uvm_error("connect_phase", "uvm_config_db #(virtual env_conf)::get(...) failed");
					// 3) hozzaadni set-tel egy altalanos agent config objectet
					// 4) Ezt osszekotni a frissen deklaralt ures obj sablonbol keszitett agent peldannyal
					uvm_config_db #(bus_agent_config)::set(this, "agt", "bus_agent_config", m_env_config.bus_agt_cfg);
					uvm_config_db #(GPIO_agent_config)::set(this, "gpio_agt", "GPIO_agent_config", m_env_config.GPIO_agent_configur);
										
					m_scoreboard = mye_scoreboard::type_id::create("m_scoreboard", this);
					m_h_scoreboard = mye_h_scoreboard::type_id::create("m_h_scoreboard", this);
			endfunction : build_phase

			function void connect_phase(uvm_phase phase);
					agt.m_monitor.item_collected_port.connect(m_scoreboard.sb_export_after);
					agt.m_driver.d_item_collected_port.connect(m_scoreboard.sb_export_before);
					agt.m_driver.d_item_collected_port.connect(m_h_scoreboard.dut_in_export);
					gpio_agt.m_monitor.GPIO_dut_out_tx_port.connect(m_h_scoreboard.dut_out_export);
			endfunction
		endclass : env_my
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// TEST class which instantiates, builds and connects the sequencer and the driver
		class bidirect_bus_test extends uvm_test;
			`uvm_component_utils(bidirect_bus_test)
			function new(string name = "bidirect_bus_test", uvm_component parent = null);
			  super.new(name, parent);
			endfunction
			
			env_my env; //env
			env_conf m_env_config; //env conf obj

			bus_agent_config bus_agt_cfg; //agt conf obj
			GPIO_agent_config GPIO_agent_configur;

			function void build_phase(uvm_phase phase);
				env = env_my::type_id::create("env", this);

				m_env_config = env_conf::type_id::create("m_env_config");

				bus_agt_cfg = bus_agent_config::type_id::create("bus_agt_cfg");
				GPIO_agent_configur = GPIO_agent_config::type_id::create("GPIO_agent_configur");
				/*CONFIGURE*/
				//configure_agent(bus_agt_cfg);
				/*SET VIF*/ //?: BUS_agnt_vi neven adjuk at?

				if (!uvm_config_db#(virtual bus_if)::get(this, "", "BUS_agnt_vif", bus_agt_cfg.BUS_agnt_vi))
			      `uvm_fatal("Config fatal", "Can't get BUS_agnt_vi interface");
			    if (!uvm_config_db#(virtual gpio_if)::get(this, "", "GPIO_agnt_vif", GPIO_agent_configur.GPIO_agnt_vi))
			      `uvm_fatal("Config fatal", "Can't get GPIO_agnt_vi interface");

			    /* Set handle in env config */
			    m_env_config.bus_agt_cfg = bus_agt_cfg;
			    m_env_config.GPIO_agent_configur = GPIO_agent_configur;
				/* Set env config object to config_db */
				uvm_config_db#(env_conf)::set(this, "env.m_env_config", "env_conf", m_env_config);
				
			endfunction : build_phase

			/*function void configure_agent(bus_agt_cfg ref_bus_agt_cfg = null);			    
			    ref_bus_agt_cfg.active = UVM_ACTIVE;			   
			endfunction*/

			task run_phase(uvm_phase phase);
			bus_seq test_seq;
			test_seq = bus_seq::type_id::create("test_seq");
			  phase.raise_objection(this, "Starting test_seq");
			  test_seq.start(env.agt.m_sequencer);
			  phase.drop_objection(this, "Finished test_seq");
			endtask: run_phase
		endclass: bidirect_bus_test
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	endpackage: bidirect_bus_pkg
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Interfaces for the bus and the DUT GPIO output
	interface bus_if;
		logic clk;
		logic resetn;
		logic[31:0] addr;
		logic[31:0] write_data;
		logic rnw;
		logic valid;
		logic ready;
		logic[31:0] read_data;
		logic error;
	endinterface: bus_if

	interface gpio_if;		
		logic[7:0]   gp_op_valid;        
		logic[255:0] gp_op;
		logic[255:0] gp_ip;
		logic clk;
	endinterface: gpio_if
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// DUT - A semi-real GPIO interface with a scratch RAM
	//
	module bidirect_bus_slave(interface bus, interface gpio);

			logic[1:0] delay;

			always @(posedge bus.clk)
			  begin
			    if(bus.resetn == 0) begin
			      delay <= 0;
			      bus.ready <= 0;
			      gpio.gp_op <= 0;
			    end
			    if(bus.valid == 1) begin // Valid cycle
			      if(bus.rnw == 0) begin // Write
			        if(delay == 2) begin
			          bus.ready <= 1;
			          delay <= 0;
			          if(bus.addr inside{[32'h0100_0000:32'h0100_001C]}) begin // GPO range - 8 words or 255 bits
			            case(bus.addr[7:0])
			              8'h00: begin gpio.gp_op_valid[0] <= 1; gpio.gp_op[31 :0  ] <= bus.write_data; end
			              8'h04: begin gpio.gp_op_valid[1] <= 1; gpio.gp_op[63 :32 ] <= bus.write_data; end
			              8'h08: begin gpio.gp_op_valid[2] <= 1; gpio.gp_op[95 :64 ] <= bus.write_data; end
			              8'h0c: begin gpio.gp_op_valid[3] <= 1; gpio.gp_op[127:96 ] <= bus.write_data; end
			              8'h10: begin gpio.gp_op_valid[4] <= 1; gpio.gp_op[159:128] <= bus.write_data; end
			              8'h14: begin gpio.gp_op_valid[5] <= 1; gpio.gp_op[191:160] <= bus.write_data; end
			              8'h18: begin gpio.gp_op_valid[6] <= 1; gpio.gp_op[223:192] <= bus.write_data; end
			              8'h1c: begin gpio.gp_op_valid[7] <= 1; gpio.gp_op[255:224] <= bus.write_data; end
			            endcase
			            bus.error <= 0;
			          end
			          else begin
			            bus.error <= 1; // Outside valid write address range
			          end
			        end
			        else begin
			          delay <= delay + 1;
			          bus.ready <= 0;
			          gpio.gp_op_valid <= 0;
			        end
			      end
			      else begin // Read cycle
			        if(delay == 3) begin
			          bus.ready <= 1;
			          delay <= 0;
			          if(bus.addr inside{[32'h0100_0000:32'h0100_001C]}) begin // GPO range - 8 words or 255 bits
			            case(bus.addr[7:0])
			              8'h00: bus.read_data <= gpio.gp_op[31:0];
			              8'h04: bus.read_data <= gpio.gp_op[63:32];
			              8'h08: bus.read_data <= gpio.gp_op[95:64];
			              8'h0c: bus.read_data <= gpio.gp_op[127:96];
			              8'h10: bus.read_data <= gpio.gp_op[159:128];
			              8'h14: bus.read_data <= gpio.gp_op[191:160];
			              8'h18: bus.read_data <= gpio.gp_op[223:192];
			              8'h1c: bus.read_data <= gpio.gp_op[255:224];
			            endcase
			            bus.error <= 0;
			          end
			          else if(bus.addr inside{[32'h0100_0020:32'h0100_003C]}) begin // GPI range - 8 words or 255 bits - read only
			            case(bus.addr[7:0])
			              8'h20: bus.read_data <= gpio.gp_ip[31:0];
			              8'h24: bus.read_data <= gpio.gp_ip[63:32];
			              8'h28: bus.read_data <= gpio.gp_ip[95:64];
			              8'h2c: bus.read_data <= gpio.gp_ip[127:96];
			              8'h30: bus.read_data <= gpio.gp_ip[159:128];
			              8'h34: bus.read_data <= gpio.gp_ip[191:160];
			              8'h38: bus.read_data <= gpio.gp_ip[223:192];
			              8'h3c: bus.read_data <= gpio.gp_ip[255:224];
			            endcase
			            bus.error <= 0;
			          end
			          else begin
			            bus.error <= 1;
			          end
			        end
			        else begin
			          delay <= delay + 1;
			          bus.ready <= 0;
			        end
			      end
			    end
			    else begin
			      bus.ready <= 0;
			      bus.error <= 0;
			      delay <= 0;
			    end
			  end
	endmodule: bidirect_bus_slave

			// Top level test bench module
	module top_tb;
		import uvm_pkg::*;
		import bidirect_bus_pkg::*;

		bus_if BUS();
		gpio_if GPIO();
		bidirect_bus_slave DUT(.bus(BUS), .gpio(GPIO));

			// Free running clock
		initial
		  begin
		    BUS.clk = 0;
		    forever begin
		      #10 BUS.clk = ~BUS.clk;
		    end
		  end

			// Reset
		initial
		  begin
		    BUS.resetn = 0;
		    repeat(3) begin
		      @(posedge BUS.clk);
		    end
		    BUS.resetn = 1;
		  end

		// UVM start up:
		initial
		  begin
		    uvm_config_db #(virtual bus_if)::set(null, "uvm_test_top", "BUS_agnt_vif" , BUS);
		    uvm_config_db #(virtual gpio_if)::set(null, "uvm_test_top", "GPIO_agnt_vif" , GPIO);
		    run_test("bidirect_bus_test");
		  end
	endmodule: top_tb
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////