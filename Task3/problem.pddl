(define (problem istanzatemporale)
  (:domain task31domain)
  (:objects
      agent1 agent2 - agent
      box1 box2 box3 - box

      bolt1 - content
      bolt2 - content
      bolt3 - content

      valve1 - content
      valve2 - content
      valve3 - content

      tool1 - content
      tool2 - content

      warehouse loc2 loc3 loc4 loc5 loc6 - location

      ws1 ws2 ws3 ws4 ws5 ws6 - workstation

      valve bolt tool - contentType

      cart1 cart2 - carrier

      slot1 slot2 slot3 slot4 slot5 - slot 
  )

  (:init
      ;start robot
      (at agent1 warehouse)
      (at agent2 warehouse)

      (free agent1)
      (free agent2)
      
      ;start box
      (at box1 warehouse)
      (at box2 warehouse)
      (at box3 warehouse)
      (is-empty box1)
      (is-empty box2)
      (is-empty box3)
      
      ;start supplies
      (at bolt1 warehouse)
      (at bolt2 warehouse)
      (at bolt3 warehouse)
      (at valve1 warehouse)
      (at valve2 warehouse)
      (at valve3 warehouse)
      (at tool1 warehouse)
      (at tool2 warehouse)
      
      ;position of workstations
      (at ws1 loc2)
      (at ws2 loc2)
      (at ws3 loc3)
      (at ws4 loc4)
      (at ws5 loc5)

      (at ws6 loc6)

      
      ; Connessioni tra le location
      (connected warehouse loc2)
      (connected loc2 warehouse)
      (connected warehouse loc3)
      (connected loc3 warehouse)
      (connected loc3 loc4)
      (connected loc4 loc3)
      ;per il momento sfrutto l'or per non scrivere tanto
      (connected warehouse loc5)
      (connected loc5 warehouse)
      (connected loc2 loc5)
      (connected loc5 loc2)

      (connected loc2 loc6)
      (connected loc6 loc2)
      (connected loc4 loc6)
      (connected loc6 loc4)


 
      ; Tipo dei contenuti
      (is-type valve1 valve)
      (is-type valve2 valve)
      (is-type valve3 valve)
      (is-type bolt1 bolt)
      (is-type bolt2 bolt)
      (is-type bolt3 bolt)
      (is-type tool1 tool)
      (is-type tool2 tool)
 
 
      ;carrier and slot
      (agent-has-carrier agent1 cart1)
      (agent-has-carrier agent2 cart2)

      (carrier-has-slot cart1 slot1)
      (carrier-has-slot cart1 slot2)
      (carrier-has-slot cart2 slot3)
      (carrier-has-slot cart2 slot4)
      (carrier-has-slot cart2 slot5)

      (is-empty slot1)
      (is-empty slot2)
      (is-empty slot3)
      (is-empty slot4)
      (is-empty slot5)
  )

  (:goal
      (and
      (workstation-has-type ws2 valve)
      (workstation-has-type ws2 bolt)
      (workstation-has-type ws3 tool)
      (workstation-has-type ws6 tool)
      (workstation-has-type ws4 bolt)

      )
  )
)
