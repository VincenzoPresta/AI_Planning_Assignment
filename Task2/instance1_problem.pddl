(define (problem deliver-content)
  (:domain task21domain)
  (:objects
      agent1 - agent
      box1 box2 box3 - box

      bolt1 - content
      bolt2 - content
      bolt3 - content

      valve1 - content
      valve2 - content
      valve3 - content

      tool1 - content
      tool2 - content

      warehouse loc1 loc2 - location

      ws1 ws2 ws3 ws4 - workstation

      valve bolt tool - contentType

  )

  (:init
      ;start agent
      (at agent1 warehouse)
      (free agent1)

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
      (at ws1 loc1)
      (at ws2 loc2)
      (at ws3 loc1)
      (at ws4 loc2)
      
      ;connections between locations
      (connected warehouse loc1)
      (connected warehouse loc2)
      (connected loc1 warehouse)
      (connected loc2 warehouse)

      ;content type
      (is-type valve1 valve)
      (is-type valve2 valve)
      (is-type valve3 valve)
      (is-type bolt1 bolt)
      (is-type bolt2 bolt)
      (is-type bolt3 bolt)
      (is-type tool1 tool)
      (is-type tool2 tool)

  )

  (:goal
      (and
      (workstation-has-type ws1 bolt)
      (workstation-has-type ws2 bolt)
      (workstation-has-type ws2 valve)
      )
  )
)
