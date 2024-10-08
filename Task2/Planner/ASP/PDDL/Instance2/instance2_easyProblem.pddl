(define (problem deliver-content)
  (:domain task22domain)
  (:objects
    agent1 agent2 - agent
    cart1 cart2 - carrier
    valve bolt tool - contentType
    warehouse loc1 loc2 - location
    ws1 ws2 ws3 ws4 - workstation
    box1 box2 box3 - box
    valve1 valve2 bolt1 bolt2 tool1 tool2 - content  
    slot1 slot2 slot3 slot4 - slot
  )

  (:init
      ;start agent
      (at agent1 warehouse)
      (at agent2 warehouse)
      (agent-has-carrier agent1 cart1)
      (agent-has-carrier agent2 cart2)
      
      ;start box
      (at box1 warehouse)
      (at box2 warehouse)
      (at box3 warehouse)
      (is-empty box1)
      (is-empty box2)
      (is-empty box3)
      
      ;start supplies
      (at valve1 warehouse)
      (at valve2 warehouse)
      (at bolt1 warehouse)
      (at bolt2 warehouse)
      (at tool1 warehouse)
      (at tool2 warehouse)

      ;content type
      (is-type valve1 valve)
      (is-type valve2 valve)
      (is-type bolt1 bolt)
      (is-type bolt2 bolt)
      (is-type tool1 tool)
      (is-type tool2 tool)
      
      ;workstations
      (at ws1 loc1)
      (at ws2 loc2)
      (at ws3 loc1)
      (at ws4 loc2)
      
      ;locations connections
      (connected warehouse loc1)
      (connected warehouse loc2)
      (connected loc2 warehouse)
      (connected loc1 warehouse)
      
      ;slot
      (is-empty slot1)
      (is-empty slot2)
      (is-empty slot3)
      (is-empty slot4)
      
      ;carrier
      (carrier-has-slot cart1 slot1)
      (carrier-has-slot cart1 slot2)
      (carrier-has-slot cart2 slot3)
      (carrier-has-slot cart2 slot4)
    
  )

  (:goal
      (and
	       (workstation-has-type ws1 bolt)
	       (workstation-has-type ws2 bolt)
	       (workstation-has-type ws3 valve)
	       (workstation-has-type ws4 tool)
      )
  )
)

