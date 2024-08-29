(define (problem deliver-volt-content)
  (:domain task31domain)
  (:objects
      agent1 agent2 - robot
      box1 box2 box3 - box

      bolt1 - content
      bolt2 - content
      bolt3 - content

      valve1 - content
      valve2 - content

      warehouse loc2 loc3 loc4  - location

      ws1 ws2 ws3 ws4  - workstation

      valve bolt - contentType

      cart1 cart2 - carrier

      slot1 slot2 slot3 slot4 slot5 - slot 
  )

  (:init
      ;start robot
      (at agent1 warehouse)
      (at agent2 warehouse)
      
      ;start box
      (at box1 warehouse)
      (at box2 warehouse)
      (at box3 warehouse)
      (box-is-empty box1)
      (box-is-empty box2)
      (box-is-empty box3)
      
      ;start supplies
      (at bolt1 warehouse)
      (at bolt2 warehouse)
      (at bolt3 warehouse)
      (at valve1 warehouse)
      (at valve2 warehouse)
      
      ;position of workstations
      (at ws1 loc2)
      (at ws2 loc2)
      (at ws3 loc3)
      (at ws4 loc4)
      
      ; Connessioni tra le location
      (connected warehouse loc2)
      (connected loc2 warehouse)
      (connected warehouse loc3)
      (connected loc3 warehouse)
      (connected loc3 loc4)
      (connected loc4 loc3)
 
      ; Tipo dei contenuti
      (is-type valve1 valve)
      (is-type valve2 valve)
      (is-type bolt1 bolt)
      (is-type bolt2 bolt)
      (is-type bolt3 bolt)
 
 
      ;carrier and slot
      (agent-has-carrier agent1 cart1)
      (agent-has-carrier agent2 cart2)

      (carrier-has-slot cart1 slot1)
      (carrier-has-slot cart1 slot2)
      (carrier-has-slot cart2 slot3)
      (carrier-has-slot cart2 slot4)
      (carrier-has-slot cart2 slot5)

      (free slot1)
      (free slot2)
      (free slot3)
      (free slot4)
      (free slot5)
  )

  (:goal
      (and
      (workstation-has-type ws2 valve)
      (workstation-has-type ws2 bolt)
      (workstation-has-type ws4 valve)
      )
  )
)
