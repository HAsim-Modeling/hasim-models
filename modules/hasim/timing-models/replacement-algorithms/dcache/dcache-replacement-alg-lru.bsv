// Implements LRU algorithm

import Vector::*;

`include "asim/provides/hasim_common.bsh"
`include "asim/provides/soft_connections.bsh"
`include "asim/provides/hasim_modellib.bsh"
`include "asim/provides/module_local_controller.bsh"

`include "asim/provides/hasim_isa.bsh"

`include "asim/provides/hasim_dcache_types.bsh"

typedef Bit#(TLog#(`DCACHE_ASSOC)) DCACHE_LRU_BITS;
typedef Bit#(1) DCACHE_DIRTY_BIT;
typedef Tuple3#(DCACHE_DIRTY_BIT, DCACHE_LRU_BITS, DCACHE_TAG) DCACHE_LINE;

typedef struct{
   Bool accessed_hit;
   Vector#(`DCACHE_ASSOC, Maybe#(DCACHE_LINE)) accessed_set;
   DCACHE_WAY accessed_way;
   }
DCacheReplacementAlgorithmInput deriving (Eq, Bits);

typedef struct{
   Vector#(`DCACHE_ASSOC, Maybe#(DCACHE_LINE)) updated_set;
   DCACHE_WAY replaced_way;
   Bool conflict_miss;	       
   }
DCacheReplacementAlgorithmOutput deriving (Eq, Bits);

module [HASIM_MODULE] mkDCacheReplacementAlgorithm();
      
   // states
   Reg#(TOKEN) resp_tok <- mkRegU();
   Reg#(Vector#(`DCACHE_ASSOC, Maybe#(DCACHE_LINE))) resp_set <- mkRegU();
   Reg#(DCACHE_WAY) resp_way <- mkRegU();
      
   // incoming ports
   Port_Receive#(Tuple2#(TOKEN, DCacheReplacementAlgorithmInput)) port_from_dcache_core <- mkPort_Receive("dcache_core_to_replacement_algorithm", 0);

   // outgoing ports
   Port_Send#(Tuple2#(TOKEN, DCacheReplacementAlgorithmOutput)) port_to_dcache_core <- mkPort_Send("replacement_algorithm_to_dcache_core");  
   
   // communication with local controller
   Vector#(1, Port_Control) inports = newVector();
   Vector#(1, Port_Control) outports = newVector();
   inports[0] = port_from_dcache_core.ctrl;
   outports[0] = port_to_dcache_core.ctrl;
   LocalController local_ctrl <- mkLocalController(inports, outports);
   
   // rules
   rule handlereq;
      
     
      // read input port from icache core
      let msg_from_dcache_core <- port_from_dcache_core.receive();
        
      // react to input message from icache core
      case(msg_from_dcache_core) matches
	 // if NoMessage
	 tagged Invalid:
	    begin
	       // send a NoMessage back to icache core and loop in the same state
	       port_to_dcache_core.send(tagged Invalid);
	       
	    end
	 // if icache core is making a request
	 tagged Valid {.tok_from_dcache_core, .req_from_dcache_core}:
	    begin
	       
	       // check the request type
	       if (req_from_dcache_core.accessed_hit)
		  // access was a hit, update LRU status bits
		  begin
		     Vector#(`DCACHE_ASSOC, Maybe#(DCACHE_LINE)) accessed_set_data = req_from_dcache_core.accessed_set;
		     let accessed_way_number = req_from_dcache_core.accessed_way;
		     if (accessed_set_data[accessed_way_number] matches tagged Valid {.accessed_dirty, .accessed_lru, .accessed_tag})
			begin
		     
			   // loop through all the ways in the set updating LRU bits
			   for (Integer w = 0; w < `DCACHE_ASSOC ; w = w + 1)
			      begin
				 case (accessed_set_data[w]) matches
				    // update LRU bits only if cache line is valid
				    tagged Valid {.test_dirty, .test_lru, .test_tag}: 
				       begin
					  if (test_lru < accessed_lru)
					     begin
						accessed_set_data[w] = tagged Valid tuple3(test_dirty, test_lru + 1, test_tag);	   
					     end
				       end
				 endcase
			      end
		  
			   // update the accessed line LRU bits
			   accessed_set_data[accessed_way_number] = tagged Valid tuple3(accessed_dirty, 0, accessed_tag);
		     
			   // send respone to icache core 
			   port_to_dcache_core.send(tagged Valid tuple2(tok_from_dcache_core, DCacheReplacementAlgorithmOutput{updated_set:accessed_set_data, replaced_way:accessed_way_number, conflict_miss: False}));
			end
		  end	
      
	       else
		  // access was a miss, update LRU status bits and pick candidate for eviction
		  begin  
		     Vector#(`DCACHE_ASSOC, Maybe#(DCACHE_LINE)) accessed_set_data = req_from_dcache_core.accessed_set;
		     DCACHE_WAY victim_way = maxBound;
		     Bool victim_found = False;
		     Bool conflict = False;
		     
		     // loop through all the ways in the set updating LRU bits
		     // loop until first invalid or the oldest way (number of cache ways - 1)
		     for (Integer w = 0; w < `DCACHE_ASSOC; w = w + 1)
			begin
			   case (accessed_set_data[w]) matches
			      // if we find a free way in the cache
			      tagged Invalid:
				 begin
				    if (!victim_found)
				       begin
					  //match {.test_lru, .test_tag} = accessed_set_data[w];
					  accessed_set_data[w] = tagged Valid tuple3(?, 0, ?);
					  victim_way = fromInteger(w);
					  victim_found = True;
				       end
				 end
			      
			      // if there are no free ways
			      tagged Valid {.test_dirty, .test_lru, .test_tag}:
				 begin
				    // found the oldest victim
				    if (test_lru == maxBound)
				       begin
					  if(!victim_found)
					     begin
						accessed_set_data[w] = tagged Valid tuple3(test_dirty, 0, test_tag);
						victim_found = True;
						victim_way = fromInteger(w);
						conflict = True;
					     end
				       end
		       		    
				    
				    // victim already found
				    else
				       begin
					  accessed_set_data[w] = tagged Valid tuple3(test_dirty, test_lru + 1, test_tag);
				       end
				 end
			   endcase
			end
		     // send output to icache core
		     port_to_dcache_core.send(tagged Valid tuple2(tok_from_dcache_core, DCacheReplacementAlgorithmOutput{updated_set:accessed_set_data, replaced_way:victim_way, conflict_miss: conflict}));					 
		  end
	    end 
	 
      endcase
   endrule
   
endmodule

			
      
     
