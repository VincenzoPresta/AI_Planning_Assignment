(define (problem instance2)
  (:domain task31domain)

  (:objects
    agent1 - agent
    box1 - box
    box2 - box
    contentA contentB - content
    warehouse loc2 loc3 - location
    ws1 ws2 - workstation
    valve bolt - contentType
    cart1 - carrier
    slot1 - slot 
    slot2 - slot 
  )

  (:init
    ; Locations
    (connected warehouse loc2)
    (connected loc2 loc3)
    (connected loc3 loc2)
    (connected loc2 warehouse)
    
    ; Workstations
    (at ws1 loc2)
    (at ws2 loc3)
    
    ; Agent
    (at agent1 warehouse)
    (agent-has-carrier agent1 cart1)

    ; Boxes
    (at box1 warehouse)
    (at box2 warehouse) 
    (box-is-empty box1)
    (box-is-empty box2)
    
    ; Contents
    (at contentA warehouse)
    (at contentB warehouse)
    (is-type contentA bolt)
    (is-type contentB valve)
    
    ; Carrier and Slots
    (carrier-has-slot cart1 slot1)
    (carrier-has-slot cart1 slot2)
    (free slot1)
    (free slot2)
  )

  (:goal
    (and
      (workstation-has-type ws1 bolt)
      (workstation-has-type ws2 valve)
    )
  )
)
