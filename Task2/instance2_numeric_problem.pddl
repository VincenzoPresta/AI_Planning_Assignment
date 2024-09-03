(define (problem deliver-volt-content)
  (:domain task22domainwithfluents)
  (:objects
    agent1 agent2 - agent
    cart1 cart2 - carrier
    valve bolt tool - contentType
    warehouse loc1 loc2 loc3 loc4 - location
    ws1 ws2 ws3 ws4 ws5 ws6 ws7 - workstation
    box1 box2 box3 - box
    valve1 valve2 valve3 bolt1 bolt2 tool1 tool2 - content
  )

  (:init
      ;Posizione elementi
      (at agent1 warehouse)
      (at agent2 warehouse)
      
      (at box1 warehouse)
      (at box2 warehouse)
      (at box3 warehouse)
      
      (at valve1 warehouse)
      (at valve2 warehouse)
      (at bolt1 warehouse)
      (at bolt2 warehouse)
      (at tool1 warehouse)
      (at tool2 warehouse)

      (is-type valve1 valve)
      (is-type valve2 valve)
      (is-type valve3 valve)
      (is-type bolt1 bolt)
      (is-type bolt2 bolt)
      (is-type tool1 tool)
      (is-type tool2 tool)
      
      (at ws1 loc1)
      (at ws2 loc2)
      (at ws3 loc1)
      (at ws4 loc2)
      (at ws5 loc3)
      (at ws6 loc4)
      (at ws7 loc4)
      
      (connected warehouse loc1)
      ;(connected warehouse loc2)
      (connected warehouse loc3)
      (connected warehouse loc4)
      (connected loc3 loc2)
      ;(connected loc1 loc4)

      ;(connected loc2 warehouse)
      (connected loc1 warehouse)
      (connected loc3 warehouse)
      (connected loc4 warehouse)
      (connected loc2 loc3)
      ;(connected loc4 loc1)

      (is-empty box1)
      (is-empty box2)
      (is-empty box3)

      (agent-has-carrier agent1 cart1)
      (agent-has-carrier agent2 cart2)

      (= (capacity cart1) 2)
      (= (capacity cart2) 2)
      (= (curr-carrier-load cart1) 0)
      (= (curr-carrier-load cart2) 0)
  )

  (:goal
      (and
         (workstation-has-type ws1 bolt)
         (workstation-has-type ws2 valve)
         (workstation-has-type ws3 valve)
         (workstation-has-type ws3 tool)
         (workstation-has-type ws6 tool)
      )
  )
)


