(define (problem deliver-content)
  (:domain task31numericdomain)
  (:objects
    agent1 agent2 - agent
    cart1 cart2 - carrier
    valve bolt tool - contenttype
    warehouse loc1 loc2 loc3 loc4 - location
    ws1 ws2 ws3 ws4 ws5 ws6 ws7 - workstation
    box1 box2 box3 - box
    valve1 valve2 valve3 bolt1 bolt2 tool1 tool2 - content
  )

  (:init
      ;start agent
      (at agent1 warehouse)
      (at agent2 warehouse)
      (free agent1)
      (free agent2)
      (agent_has_carrier agent1 cart1)
      (agent_has_carrier agent2 cart2)
      
      ;start box
      (at box1 warehouse)
      (at box2 warehouse)
      (at box3 warehouse)
      (is_empty box1)
      (is_empty box2)
      (is_empty box3)
      
      ;start supplies
      (at valve1 warehouse)
      (at valve2 warehouse)
      (at valve3 warehouse)
      (at bolt1 warehouse)
      (at bolt2 warehouse)
      (at tool1 warehouse)
      (at tool2 warehouse)
      
      ;content type
      (is_type valve1 valve)
      (is_type valve2 valve)
      (is_type valve3 valve)
      (is_type bolt1 bolt)
      (is_type bolt2 bolt)
      (is_type tool1 tool)
      (is_type tool2 tool)
      
      ;workstations
      (at ws1 loc1)
      (at ws2 loc2)
      (at ws3 loc1)
      (at ws4 loc2)
      (at ws5 loc3)
      (at ws6 loc4)
      (at ws7 loc4)
      
      ;locations connections
      (connected warehouse loc1)
      (connected warehouse loc3)
      (connected warehouse loc4)
      (connected loc3 loc2)
      (connected loc1 warehouse)
      (connected loc3 warehouse)
      (connected loc4 warehouse)
      (connected loc2 loc3)

      ;capacity and load
      (= (capacity cart1) 2)
      (= (capacity cart2) 2)
      (= (curr_carrier_load cart1) 0)
      (= (curr_carrier_load cart2) 0)
  )

  (:goal
      (and
         (workstation_has_type ws1 bolt)
         (workstation_has_type ws2 valve)
         (workstation_has_type ws3 valve)
         (workstation_has_type ws3 tool)
         (workstation_has_type ws6 tool)
      )
  )
)


